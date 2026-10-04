#include "arion/BatteryFrame.hpp"

#include <common/mavlink.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;
// 신호 핸들러는 종료 플래그만 바꾼다; I/O와 동기화 자원 정리는 주 루프가 수행한다.
void handle_signal(int) { stop_requested = 1; }

struct Options {
    std::string topic;
    uint16_t port{0};
    uint8_t system_id{0};
    unsigned duration_s{20};
    bool wait_when_stale{false};
};

// 10진 무부호 입력을 범위 안에서만 변환한다; 부호·남은 문자·overflow는 거부한다.
unsigned number(const std::string &text, unsigned minimum, unsigned maximum)
{
    unsigned value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
        value < minimum || value > maximum) {
        throw std::invalid_argument("Invalid numeric argument: " + text);
    }
    return value;
}

// CLI의 초·포트·system ID와 명시 송신 의도를 검증한다; 검증 전 소켓은 만들지 않는다.
Options parse_options(int argc, char **argv)
{
    Options options;
    unsigned seen = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string key(argv[i]);
        const unsigned bit = key == "--send" ? 1 : key == "--topic" ? 2 :
                             key == "--px4-port" ? 4 : key == "--system-id" ? 8 :
                             key == "--duration" ? 16 : key == "--stale-policy" ? 32 : 0;
        if (!bit || (seen & bit)) { throw std::invalid_argument("Unknown/duplicate option: " + key); }
        seen |= bit;
        if (key == "--send") { continue; }
        if (++i == argc) { throw std::invalid_argument("Missing value for " + key); }
        const std::string value(argv[i]);
        if (key == "--topic") { options.topic = value; }
        if (key == "--px4-port") { options.port = static_cast<uint16_t>(number(value, 1024, 65535)); }
        if (key == "--system-id") { options.system_id = static_cast<uint8_t>(number(value, 1, 255)); }
        if (key == "--duration") { options.duration_s = number(value, 0, 120); }
        if (key == "--stale-policy") {
            if (value != "wait" && value != "exit") { throw std::invalid_argument("stale-policy must be wait or exit"); }
            options.wait_when_stale = value == "wait";
        }
    }
    if ((seen & 15) != 15 || options.topic.empty() || options.topic.front() != '/') {
        throw std::invalid_argument("Explicit --send, --topic, --px4-port and --system-id are required");
    }
    const char *partition = std::getenv("GZ_PARTITION");
    if (!partition || !*partition) { throw std::invalid_argument("Set an isolated GZ_PARTITION first"); }
    return options;
}

class UdpSocket {
public:
    // 명시된 UDP 포트로 loopback 전용 비차단 소켓을 연결한다; 실패 시 열린 FD를 회수한다.
    explicit UdpSocket(uint16_t port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        if (fd_ < 0) { throw std::runtime_error(std::string("socket: ") + std::strerror(errno)); }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::connect(fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
            const int error = errno;
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(std::string("UDP connect: ") + std::strerror(error));
        }
    }
    // 이 객체가 소유한 UDP FD만 회수한다; 다른 프로세스 소켓에는 접근하지 않는다.
    ~UdpSocket() { if (fd_ >= 0) { ::close(fd_); } }
    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    // 인코딩된 byte 배열 한 패킷을 보낸다; 부분 전송/네트워크 오류는 성공으로 숨기지 않는다.
    void send(const uint8_t *bytes, std::size_t size)
    {
        const auto count = ::send(fd_, bytes, size, 0);
        if (count < 0) { throw std::runtime_error(std::string("UDP send: ") + std::strerror(errno)); }
        if (static_cast<std::size_t>(count) != size) { throw std::runtime_error("Partial UDP send"); }
    }

private:
    int fd_{-1};
};

struct Sample {
    arion::BatteryWireValues values;
    int64_t sim_ns{0};
    Clock::time_point received{};
};
struct Mailbox {
    std::mutex mutex;
    std::condition_variable ready;
    Sample latest;
    uint64_t generation{0}, invalid{0}, duplicates{0}, resets{0};
    bool active{true};
};

// 검증된 프레임을 ns 시각과 함께 최신 한 칸에 넣는다. 호출자는 mailbox mutex를 보유한다.
// sim-time 역행은 reset이며 0도 허용한다. 같은 시각 재발행은 새 측정으로 보내지 않는다.
void accept_sample(Mailbox &state, const Sample &sample)
{
    if (!state.active) { return; }
    if (state.generation && sample.sim_ns == state.latest.sim_ns) { ++state.duplicates; return; }
    if (state.generation && sample.sim_ns < state.latest.sim_ns) { ++state.resets; }
    state.latest = sample;
    ++state.generation;
    state.ready.notify_one();
}

struct DeliveryDecision {
    Sample sample;
    bool send{false}, missing_event{false}, recovered_event{false};
};

struct DeliveryState {
    Clock::time_point last_valid;
    uint64_t generation{0}, superseded{0}, stale{0};
    bool missing{false};

    // 시작 steady 시각을 받아 최초 무수신 공백의 기준을 세운다. sim-time과 무관하다.
    explicit DeliveryState(Clock::time_point start) : last_valid(start) {}

    // mutex로 보호한 mailbox와 현재 steady 시각을 입력받아 새 프레임/공백 전이를 판단한다.
    // 수신부터 1초 초과한 프레임은 버리며 5초 공백은 전이당 한 번만 알린다.
    DeliveryDecision poll(const Mailbox &state, Clock::time_point now)
    {
        DeliveryDecision decision;
        if (state.generation != generation) {
            superseded += state.generation - generation - 1;
            generation = state.generation;
            if (now - state.latest.received <= std::chrono::seconds(1)) {
                decision.sample = state.latest;
                decision.send = true;
                decision.recovered_event = missing;
                last_valid = state.latest.received;
                missing = false;
            } else { ++stale; }
        }
        if (now - last_valid >= std::chrono::seconds(5) && !missing) {
            missing = decision.missing_event = true;
        }
        return decision;
    }
};

// Float_V schema·sim ns·9개 물리단위 입력을 검사해 MAVLink wire 값과 steady 수신 시각을 얻는다.
bool decode_message(const gz::msgs::Float_V &message, Sample &sample)
{
    if (!message.has_header() || !message.header().has_stamp()) { return false; }
    unsigned schema_entries = 0;
    for (const auto &entry : message.header().data()) {
        if (entry.key() != "schema") { continue; }
        ++schema_entries;
        if (entry.value_size() != 1 || entry.value(0) != arion::kBatterySchema) { return false; }
    }
    if (schema_entries != 1) { return false; }
    const auto &stamp = message.header().stamp();
    constexpr int64_t billion = 1000000000;
    if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= billion ||
        stamp.sec() > (std::numeric_limits<int64_t>::max() - stamp.nsec()) / billion) {
        return false;
    }
    if (!arion::decode_battery_frame(message.data().data(), message.data_size(), sample.values)) { return false; }
    sample.sim_ns = stamp.sec() * billion + stamp.nsec();
    sample.received = Clock::now();
    return true;
}

class Subscription {
public:
    // topic에 구독하고 공유 mailbox만 callback에 넘긴다; 콜백은 소켓이나 소유자의 주소를 잡지 않는다.
    explicit Subscription(std::string topic) : mailbox(std::make_shared<Mailbox>()), topic_(std::move(topic))
    {
        // 지연 콜백도 독립 shared state만 접근하므로 Subscription 파괴 뒤 dangling this가 없다.
        const auto state = mailbox;
        const std::function<void(const gz::msgs::Float_V &)> callback = [state](const auto &message) {
            Sample sample;
            const bool valid = decode_message(message, sample);
            std::lock_guard<std::mutex> lock(state->mutex);
            if (!state->active) { return; }
            if (!valid) { ++state->invalid; return; }
            accept_sample(*state, sample);
        };
        if (!node_.Subscribe(topic_, callback)) { throw std::runtime_error("Gazebo subscription failed"); }
    }
    // mailbox 쓰기를 막고 구독을 해제한다; 이미 전달된 callback은 공유 state로 안전하게 끝난다.
    ~Subscription()
    {
        {
            std::lock_guard<std::mutex> lock(mailbox->mutex);
            mailbox->active = false;
        }
        node_.Unsubscribe(topic_);
    }
    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;
    const std::shared_ptr<Mailbox> mailbox;

private:
    std::string topic_;
    gz::transport::Node node_;
};

// 표준 mV/cA/cdegC/mAh/% 값을 BATTERY_STATUS로 인코딩한다; 선택 진단은 관측 provenance용이다.
void send_sample(UdpSocket &socket, uint8_t system_id, const Sample &sample, bool log_sample = true)
{
    const auto &value = sample.values;
    const std::array<uint16_t, 4> unused_cells{};
    mavlink_message_t message{};
    mavlink_msg_battery_status_pack(system_id, MAV_COMP_ID_BATTERY, &message,
        value.id, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
        value.temperature_cdeg, value.voltages.data(), value.current_ca,
        value.consumed_mah, -1, value.remaining_pct, 0,
        MAV_BATTERY_CHARGE_STATE_UNDEFINED, unused_cells.data(), MAV_BATTERY_MODE_UNKNOWN, 0);
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    socket.send(bytes.data(), length);
    if (!log_sample) { return; }
    // 로컬 UDP 성공은 PX4 수신 증명이 아니다. 연속 모드 진단은 표본 추출된다.
    std::cout << "{\"event\":\"udp_sent\",\"sim_ns\":" << sample.sim_ns
              << ",\"system_id\":" << static_cast<unsigned>(system_id)
              << ",\"component_id\":" << MAV_COMP_ID_BATTERY
              << ",\"battery_id\":" << static_cast<unsigned>(value.id)
              << ",\"voltages_mv\":[" << value.voltages[0] << ',' << value.voltages[1]
              << ',' << value.voltages[2] << ',' << value.voltages[3]
              << "],\"current_ca\":" << value.current_ca
              << ",\"remaining_pct\":" << static_cast<int>(value.remaining_pct)
              << ",\"temperature_cdeg\":" << value.temperature_cdeg
              << ",\"consumed_mah\":" << value.consumed_mah << "}" << std::endl;
}

// 한 칸 mailbox를 wall-clock으로 소비한다. duration0은 신호까지, wait는 긴 pause를 허용한다.
// UDP 성공은 PX4 수신 증명이 아니며 새 generation의 수신 나이 1초 이하 값만 보낸다.
int forward_frames(const Options &options, const std::shared_ptr<Mailbox> &state, UdpSocket &socket)
{
    const auto start = Clock::now();
    const auto deadline = options.duration_s == 0 ? Clock::time_point::max()
                        : start + std::chrono::seconds(options.duration_s);
    DeliveryState delivery(start);
    auto last_logged = start;
    uint64_t sent = 0;
    while (!stop_requested && Clock::now() < deadline) {
        DeliveryDecision decision;
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            state->ready.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return state->generation != delivery.generation || stop_requested;
            });
            decision = delivery.poll(*state, Clock::now());
        }
        const auto now = Clock::now();
        if (stop_requested || now >= deadline) { break; }
        if (decision.missing_event) {
            std::cout << "{\"event\":\"missing\",\"age_s\":"
                      << std::chrono::duration<double>(now - delivery.last_valid).count()
                      << ",\"stale_policy\":\"" << (options.wait_when_stale ? "wait" : "exit")
                      << "\"}" << std::endl;
        }
        if (delivery.missing && !options.wait_when_stale) {
            throw std::runtime_error("No new valid battery frame for 5 wall-clock seconds; no stale replay");
        }
        if (decision.recovered_event) {
            std::cout << "{\"event\":\"recovered\",\"sim_ns\":" << decision.sample.sim_ns << "}" << std::endl;
        }
        if (!decision.send) { continue; }
        // 서비스의 송신 로그만 최대 1Hz로 제한한다. 실제 전송률은 producer를 따른다.
        const bool log_sample = options.duration_s != 0 || !sent || now - last_logged >= std::chrono::seconds(1);
        send_sample(socket, options.system_id, decision.sample, log_sample);
        if (log_sample) { last_logged = now; }
        ++sent;
    }
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->active = false;
        std::cerr << "UDP sends=" << sent << " invalid=" << state->invalid
                  << " duplicates=" << state->duplicates << " superseded=" << delivery.superseded
                  << " stale=" << delivery.stale << " sim_resets=" << state->resets << '\n';
    }
    if (stop_requested) { return 0; }
    if (!sent) { throw std::runtime_error("No frames sent before the duration limit"); }
    return 0;
}

// 종료 신호·loopback 소켓·독립 구독 수명을 소유한다; 반환/예외 시 RAII로 회수한다.
int run(const Options &options)
{
    if (std::signal(SIGINT, handle_signal) == SIG_ERR || std::signal(SIGTERM, handle_signal) == SIG_ERR) {
        throw std::runtime_error("Cannot install shutdown handlers");
    }
    UdpSocket socket(options.port);
    Subscription subscription(options.topic);
    return forward_frames(options, subscription.mailbox, socket);
}
}  // namespace

// 도움말/인자/실행 오류를 CLI exit code로 전달한다; PX4 설정이나 비행 제어는 수행하지 않는다.
int main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: arion_battery_sender --send --topic /arion/MODEL/battery_state "
                     "--px4-port PORT --system-id ID [--duration SECONDS] [--stale-policy wait|exit]\n"
                     "Loopback only; component 180; duration 0=continuous, 1..120 (default 20); stale-policy default exit; requires GZ_PARTITION.\n"
                     "Sends only BATTERY_STATUS. No PX4 configuration, heartbeat, control or arming.\n"
                     "Requires matching PX4 external-battery configuration/conversions; see the run guide.\n";
        return 0;
    }
    try {
        return run(parse_options(argc, argv));
    } catch (const std::exception &error) {
        std::cerr << "battery sender: " << error.what() << '\n';
        return 1;
    }
}
