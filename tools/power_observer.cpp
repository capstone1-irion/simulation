// Read-only producer evidence. No commands, messages or forces are published.
#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }
struct State { std::mutex mutex; bool active{true}; unsigned count{0}; };
}

int main(int argc, char **argv)
{
    if (argc != 3 || !std::getenv("GZ_PARTITION")) {
        std::cerr << "Usage: arion_power_observer MODEL DURATION_SECONDS (requires GZ_PARTITION)\n";
        return 2;
    }
    unsigned duration = 0;
    const std::string text(argv[2]);
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), duration);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || duration < 1 || duration > 120) { return 2; }
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    const auto state = std::make_shared<State>();
    gz::transport::Node node;
    const std::string prefix = std::string("/arion/") + argv[1];
    for (const std::string kind : {"motor", "battery"}) {
        const std::function<void(const gz::msgs::Float_V &)> callback = [state, kind](const auto &msg) {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (!state->active) { return; }
            long long ns = -1;
            if (msg.has_header() && msg.header().has_stamp()) {
                const auto &s = msg.header().stamp();
                if (s.sec() >= 0 && s.sec() < 9000000000LL && s.nsec() >= 0 && s.nsec() < 1000000000) {
                    ns = s.sec() * 1000000000LL + s.nsec();
                }
            }
            bool schema_ok = false;
            for (const auto &entry : msg.header().data()) {
                if (entry.key() == "schema" && entry.value_size() == 1 &&
                    entry.value(0) == "arion." + kind + "_state.v1") { schema_ok = true; }
            }
            const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            std::cout << "{\"kind\":\"" << kind << "\",\"sim_ns\":" << ns
                      << ",\"receive_monotonic_ns\":" << now
                      << ",\"schema_ok\":" << (schema_ok ? "true" : "false") << ",\"values\":[";
            for (int i = 0; i < msg.data_size(); ++i) {
                if (i) { std::cout << ','; }
                if (std::isfinite(msg.data(i))) { std::cout << std::setprecision(9) << msg.data(i); }
                else { std::cout << "null"; }
            }
            std::cout << "]}" << std::endl;
            ++state->count;
        };
        if (!node.Subscribe(prefix + '/' + kind + "_state", callback)) { return 1; }
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    while (!stop && std::chrono::steady_clock::now() < deadline) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
    unsigned count = 0;
    { std::lock_guard<std::mutex> lock(state->mutex); state->active = false; count = state->count; }
    node.Unsubscribe(prefix + "/motor_state");
    node.Unsubscribe(prefix + "/battery_state");
    std::cerr << "producer_frames=" << count << '\n';
    return count ? 0 : 1;
}
