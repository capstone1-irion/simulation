#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>
#include "arion/MotorFrame.hpp"

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

class ArionMotorBridge final : public ModuleBase<ArionMotorBridge> {
    struct CallbackState {
        std::mutex mutex;
        std::condition_variable changed;
        arion::MotorFrame frame{};
        uint64_t received_at{0};
        bool pending{false};
        bool stopping{false};
        std::atomic<unsigned> received{0}, rejected{0};
    };

    struct LifecycleState {
        std::mutex mutex;
        std::condition_variable changed;
        pthread_t worker{};
        bool handle_ready{false}, owner_released{false}, joining{false}, reaped{false};
        int join_error{0};
    };

public:
    // 목적: 구독할 Gazebo 토픽 이름을 보관하고 멤버 기본 상태를 구성한다.
    // 입출력: 토픽 C 문자열 → bridge 객체(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 토픽 검증과 실제 구독은 instantiate/init에서 수행한다.
    explicit ArionMotorBridge(const char *topic) : _topic(topic) {}

    // 목적: CLI의 stop/status를 전용 수명 관리 경로로 보내고 나머지는 ModuleBase에 위임한다.
    // 입출력: argc/argv → 명령 처리 결과 int(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 기본 강제 중지 대신 소유권을 유지하는 협력적 중지를 사용한다.
    static int main(int argc, char *argv[])
    {
        // ModuleBase::main calls its own (forced) stop, not T::stop_command.
        // Route the actual CLI through our cooperative-only ownership policy.
        if (argc > 1 && std::strcmp(argv[1], "stop") == 0) { return stop_command(); }
        if (argc > 1 && std::strcmp(argv[1], "status") == 0) { return status_command(); }
        return ModuleBase<ArionMotorBridge>::main(argc, argv);
    }

    // 목적: 중지를 요청하고 worker의 정상 종료·native join·등록 해제를 조정한다.
    // 입출력: 인자 없음 → PX4_OK/PX4_ERROR; 전체 대기는 단조 벽시계 기준 5 s 예산이다.
    // 변경/제약: 설명만 추가, 동작 유지. 시간 초과 시 강제 삭제하지 않고 남은 소유권/등록을 유지해 재시작을 막는다.
    static int stop_command()
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(deadline.time_since_epoch());
        const timespec until{seconds.count(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.time_since_epoch() - seconds).count()};
        // Linux steady_clock and the native timed lock both use CLOCK_MONOTONIC.
        // One wall-clock budget includes command coordination; never wait on PX4's clock.
        if (lock_until(until) != 0) {
            PX4_ERR("stop coordination timeout; ownership unchanged");
            return PX4_ERROR;
        }
        auto *instance = _object.load();
        auto state = _lifecycle;
        if (instance) { instance->request_stop(); }
        pthread_mutex_unlock(&px4_modules_mutex);
        if (!state) { return PX4_OK; }
        std::unique_lock<std::mutex> lock(state->mutex);
        // 목적: stop 대기가 이미 회수됐거나 단일 join을 시작할 수 있는 시점에만 끝나도록 판정한다.
        // 입출력: 캡처한 lifecycle 상태 → bool(물리 단위 없음).
        // 변경/제약: 설명만 추가, 동작 유지. state mutex를 잡은 상태에서 handle·owner 해제·joining을 함께 확인한다.
        if (!state->changed.wait_until(lock, deadline, [&] {
                return state->reaped || (state->handle_ready && state->owner_released && !state->joining);
            })) {
            PX4_ERR("stop timeout; ownership retained, restart denied until native reap");
            return PX4_ERROR;
        }
        if (!state->reaped) {
            // Exactly one command owns the join. Never hold the module/state lock
            // across it: the pinned adapter must finish taskmap/argv cleanup too.
            state->joining = true;
            const pthread_t worker = state->worker;
            lock.unlock();
            const int result = join_until(worker, until);
            lock.lock();
            state->joining = false;
            state->join_error = result;
            state->reaped = result == 0;
            state->changed.notify_all();
            if (result != 0) {
                PX4_ERR("native reap failed (%d); ownership retained, retry stop before restart", result);
                return PX4_ERROR;
            }
        }
        lock.unlock();
        if (lock_until(until) != 0) {
            PX4_ERR("stop finalization timeout; native reaped, registration retained; retry stop");
            return PX4_ERROR;
        }
        // A concurrent stop may have finalized and started another generation.
        // This control block, not a recycled PX4 task number, identifies our worker.
        if (_lifecycle == state) { _task_id = -1; _lifecycle.reset(); }
        pthread_mutex_unlock(&px4_modules_mutex);
        return PX4_OK;
    }

    // 목적: 모듈 수명 잠금 아래 실행 중 상태 또는 native 정리 대기 상태를 출력한다.
    // 입출력: 인자 없음 → 상태 로그와 int 결과(카운터는 건수, join 오류는 오류 코드).
    // 변경/제약: 설명만 추가, 동작 유지. 객체가 없으면 PX4_ERROR이며 출력만으로 정리를 완료하지 않는다.
    static int status_command()
    {
        pthread_mutex_lock(&px4_modules_mutex);
        auto *instance = _object.load();
        int result = PX4_ERROR;
        if (instance) { result = instance->print_status(); }
        else if (_lifecycle) {
            std::lock_guard<std::mutex> guard(_lifecycle->mutex);
            PX4_WARN("stopping; native cleanup pending (last join error: %d)", _lifecycle->join_error);
        } else { PX4_INFO("not running"); }
        pthread_mutex_unlock(&px4_modules_mutex);
        return result;
    }

    // 목적: 초기화한 bridge를 등록한 뒤 worker를 만들고 생성 실패 시 소유 자원을 되돌린다.
    // 입출력: 시작 인자 argc/argv → PX4_OK/PX4_ERROR(스택 크기 인자는 byte).
    // 변경/제약: 설명만 추가, 동작 유지. Linux POSIX 및 ModuleBase 수명 잠금 전제이며 비동기 초기화 대기를 두지 않는다.
    static int task_spawn(int argc, char *argv[])
    {
#if !defined(__PX4_LINUX) || !defined(__PX4_POSIX)
        PX4_ERR("bridge requires Linux POSIX monotonic native reaping");
        return PX4_ERROR;
#endif
        // ModuleBase holds its lifecycle mutex here. Initialize before spawning
        // so start cannot succeed with a null _object (or an unowned worker on
        // initialization failure). There is no asynchronous startup wait/timeout.
        auto *instance = instantiate(argc, argv);
        if (!instance) { return PX4_ERROR; }
        _lifecycle = instance->_lifecycle_state;
        _object.store(instance);
        _task_id = px4_task_spawn_cmd("arion_motor_bridge", SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT, 8192,
                                    static_cast<px4_main_t>(&task_entry), argv);
        if (_task_id < 0) {
            _task_id = -1;
            _object.store(nullptr);
            delete instance;
            _lifecycle.reset();
            return PX4_ERROR;
        }
        return PX4_OK;
    }

    // 목적: -t 옵션을 검사하고 토픽을 구독할 bridge를 생성·초기화한다.
    // 입출력: 시작 인자 argc/argv → 초기화된 객체 포인터, 검사/초기화 실패 시 nullptr(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 절대 Gazebo 토픽과 잔여 인자 없음이 필요하며 실패 객체는 삭제한다.
    static ArionMotorBridge *instantiate(int argc, char *argv[])
    {
        const char *topic = nullptr;
        int cli_optind = 1;
        const char *cli_optarg = nullptr;
        int option;
        while ((option = px4_getopt(argc, argv, "t:", &cli_optind, &cli_optarg)) != EOF) {
            if (option == 't') { topic = cli_optarg; }
            else { print_usage("invalid option"); return nullptr; }
        }
        if (!topic || topic[0] != '/' || cli_optind != argc) {
            print_usage("an absolute Gazebo topic is required (-t)");
            return nullptr;
        }
        auto *instance = new ArionMotorBridge(topic);
        if (!instance || !instance->init()) { delete instance; return nullptr; }
        return instance;
    }

    // 목적: 지원하지 않는 추가 명령에 대해 사용법을 출력한다.
    // 입출력: 사용하지 않는 argc/argv → print_usage의 반환값(PX4_OK, 물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 알 수 없는 명령이어도 이 함수 자체는 오류 코드를 별도로 만들지 않는다.
    static int custom_command(int, char *[]) { return print_usage("unknown command"); }

    // 목적: bridge의 역할·시작 옵션·기본 명령 사용법과 선택적 사유를 출력한다.
    // 입출력: 사유 C 문자열 또는 nullptr → 로그와 PX4_OK(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 상태 전달 전용이며 actuator 출력을 만들지 않는다는 경계를 안내한다.
    static int print_usage(const char *reason = nullptr)
    {
        if (reason) { PX4_WARN("%s", reason); }
        PRINT_MODULE_DESCRIPTION("Gazebo fixed/simulated motor state -> esc_status. No actuator outputs.");
        PRINT_MODULE_USAGE_NAME("arion_motor_bridge", "simulation");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_STRING('t', nullptr, nullptr, "Absolute Gazebo motor-state topic", false);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return PX4_OK;
    }

    // 목적: 토픽·수신/거부/발행/차단 누계와 기본 ESC 발행기 비활성 여부를 출력한다.
    // 입출력: 객체 상태 → 상태 로그와 PX4_OK(카운터는 건수).
    // 변경/제약: 설명만 추가, 동작 유지. 카운터는 개별 원자 읽기이며 출력만으로 데이터 품질이나 전송 완료를 인증하지 않는다.
    int print_status() override
    {
        PX4_INFO("topic: %s", _topic.c_str());
        PX4_INFO("received: %u, rejected: %u, published: %u, source-blocked: %u",
                 _callback_state->received.load(), _callback_state->rejected.load(),
                 _published.load(), _blocked.load());
        PX4_INFO("stock status disabled: %s", stock_disabled() ? "yes" : "no");
        return PX4_OK;
    }

    // 목적: 최신 대기 모터 프레임을 받아 기본 발행기가 꺼져 있을 때 uORB esc_status로 전달한다.
    // 입출력: 공유 프레임·arming 상태 → 모터별 rpm·V·A와 수신 시각 µs의 ESC 상태, 반환값 없음.
    // 변경/제약: 설명만 추가, 동작 유지. 각 소비 프레임은 최대 한 번 발행하며 arming은 대리값, 온도 등은 미구현이다.
    void run() override
    {
        while (!should_exit()) {
            arion::MotorFrame frame{};
            uint64_t received_at = 0;
            bool pending = false;
            {
                std::unique_lock<std::mutex> guard(_callback_state->mutex);
                // 목적: 새 프레임 도착 또는 중지 요청이 있을 때 worker 대기를 해제한다.
                // 입출력: 캡처한 callback 상태 → bool(물리 단위 없음).
                // 변경/제약: 설명만 추가, 동작 유지. callback mutex 아래 판정하며 시간 경과만으로 프레임을 재발행하지 않는다.
                _callback_state->changed.wait(guard, [&] {
                    return _callback_state->pending || _callback_state->stopping;
                });
                if (_callback_state->stopping) { break; }
                pending = _callback_state->pending;
                if (pending) {
                    frame = _callback_state->frame;
                    received_at = _callback_state->received_at;
                    _callback_state->pending = false;
                }
            }
            if (pending) {
                if (stock_disabled() && received_at != 0) {
                    _armed_sub.update(&_armed);
                    esc_status_s status{};
                    status.timestamp = received_at;
                    status.counter = ++_counter;
                    status.esc_count = frame.size();
                    status.esc_online_flags = 0x0F;
                    // PX4 arming-state proxy, not an independently measured ESC state.
                    status.esc_armed_flags = _armed.armed ? 0x0F : 0;
                    for (std::size_t i = 0; i < frame.size(); ++i) {
                        status.esc[i].timestamp = received_at;
                        status.esc[i].esc_rpm = static_cast<int32_t>(std::lround(frame[i].rpm));
                        status.esc[i].esc_voltage = frame[i].voltage;
                        status.esc[i].esc_current = frame[i].current;
                        status.esc[i].esc_address = static_cast<uint8_t>(i + 1);
                        status.esc[i].actuator_function = _functions[i];
                        // Temperature, connection type, failures and power are unimplemented
                        // placeholders. In particular, temperature=0 is not a measurement.
                    }
                    if (_status_pub.publish(status)) { ++_published; }
                } else {
                    ++_blocked;
                }
            }
            // Publish each received frame at most once. Silence/invalid input does
            // not refresh stale state or silently fall back to command echo.
        }
        _node.Unsubscribe(_topic);
    }

    // 목적: ModuleBase 종료 요청과 callback 중지 플래그를 함께 설정하고 대기 worker를 깨운다.
    // 입출력: 인자 없음 → 종료 플래그·조건 변수 알림, 반환값 없음(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 중지 요청만 수행하며 이 함수가 객체 삭제나 native join을 하지는 않는다.
    void request_stop() override
    {
        {
            std::lock_guard<std::mutex> guard(_callback_state->mutex);
            ModuleBase<ArionMotorBridge>::request_stop();
            _callback_state->stopping = true;
        }
        _callback_state->changed.notify_all();
    }

private:
    static std::shared_ptr<LifecycleState> _lifecycle;

    // 목적: PX4 모듈 수명 mutex를 지정된 단조 시계 기한까지 획득한다.
    // 입출력: CLOCK_MONOTONIC 절대 기한(timespec의 s/ns) → 성공 0 또는 pthread 오류 코드.
    // 변경/제약: 설명만 추가, 동작 유지. Linux POSIX에서만 지원하며 실패 시 잠금을 소유했다고 가정하면 안 된다.
    static int lock_until(const timespec &until)
    {
#if defined(__PX4_LINUX) && defined(__PX4_POSIX)
        return pthread_mutex_clocklock(&px4_modules_mutex, CLOCK_MONOTONIC, &until);
#else
        (void)until;
        return ENOTSUP;
#endif
    }

    // 목적: 지정 worker의 native 자원을 단조 시계 기한 내 join으로 회수한다.
    // 입출력: pthread 핸들·CLOCK_MONOTONIC 절대 기한(s/ns) → 성공 0 또는 오류 코드.
    // 변경/제약: 설명만 추가, 동작 유지. 자기 join은 EDEADLK, 비지원 플랫폼은 ENOTSUP이며 강제 취소하지 않는다.
    static int join_until(pthread_t worker, const timespec &until)
    {
#if defined(__PX4_LINUX) && defined(__PX4_POSIX)
        return pthread_equal(worker, pthread_self()) ? EDEADLK :
            pthread_clockjoin_np(worker, nullptr, CLOCK_MONOTONIC, &until);
#else
        (void)worker;
        (void)until;
        return ENOTSUP;
#endif
    }

    // 목적: worker 핸들을 알리고 미리 생성한 객체를 실행한 뒤 worker 소유 객체를 정리한다.
    // 입출력: 사용하지 않는 task 인자 → lifecycle 알림과 정상 경로 PX4_OK(물리 단위 없음).
    // 변경/제약: 설명만 추가, 동작 유지. 객체 파괴 후에도 성공적인 외부 join 전까지 task 등록을 유지한다.
    static int task_entry(int, char *[])
    {
        // A successful spawn transfers owner destruction exclusively to this
        // worker. Commands can request stop but never cancel/delete its owner.
        pthread_mutex_lock(&px4_modules_mutex);
        auto state = _lifecycle;
        {
            std::lock_guard<std::mutex> guard(state->mutex);
            state->worker = pthread_self();
            state->handle_ready = true;
        }
        pthread_mutex_unlock(&px4_modules_mutex);
        state->changed.notify_all();
        auto *instance = _object.load();
        if (instance) { instance->run(); }
        // ModuleBase::exit_and_cleanup clears _task_id before the native adapter
        // exits. Keep registration until a successful product join, denying start
        // even after the owner is safely destroyed or a stop deadline expires.
        pthread_mutex_lock(&px4_modules_mutex);
        delete instance;
        _object.store(nullptr);
        pthread_mutex_unlock(&px4_modules_mutex);
        {
            std::lock_guard<std::mutex> guard(state->mutex);
            state->owner_released = true;
        }
        state->changed.notify_all();
        return PX4_OK;
    }

    // 목적: 기본 Gazebo ESC 발행기 비활성 파라미터를 현재 값으로 확인한다.
    // 입출력: 보관한 파라미터 핸들 → 읽기 성공 및 값 0일 때만 true(무차원 설정값).
    // 변경/제약: 설명만 추가, 동작 유지. 핸들 무효·읽기 실패는 false로 처리하며 파라미터를 변경하지 않는다.
    bool stock_disabled() const
    {
        int32_t enabled = 1;
        return _stock_param != PARAM_INVALID && param_get(_stock_param, &enabled) == 0 && enabled == 0;
    }

    // 목적: 기본 ESC 발행기 비활성·x500 모터 매핑을 확인하고 Gazebo 수신 callback을 구독한다.
    // 입출력: 객체 토픽·PX4 파라미터 → 초기화 성공 bool(매핑은 actuator function 식별자).
    // 변경/제약: 설명만 추가, 동작 유지. 고정 x500 매핑만 허용하며 callback은 객체 대신 공유 데이터만 소유한다.
    bool init()
    {
        _stock_param = param_find("SIM_GZ_ESC_ST");
        if (!stock_disabled()) {
            PX4_ERR("stock ESC publisher enabled; set SIM_GZ_ESC_ST 0 before starting bridge");
            return false;
        }
        // This initial fixture contract is specific to the stock x500 allocation.
        const int32_t expected[] = {101, 102, 103, 104};
        for (std::size_t i = 0; i < _functions.size(); ++i) {
            char name[24];
            snprintf(name, sizeof(name), "SIM_GZ_EC_FUNC%u", static_cast<unsigned>(i + 1));
            int32_t function = 0;
            const param_t handle = param_find(name);
            if (handle == PARAM_INVALID || param_get(handle, &function) != 0 || function != expected[i]) {
                PX4_ERR("unsupported x500 actuator mapping: %s", name);
                return false;
            }
            _functions[i] = static_cast<uint8_t>(function);
        }
        // Unsubscribe cannot retract a handler already snapshotted by dispatch.
        // Every copy owns only its data, never this module or its publications.
        // 목적: Gazebo 메시지를 객체 수명과 분리된 공유 상태의 해석 함수로 전달하는 람다이다.
        // 입출력: Float_V 참조 → on_message 호출, 반환값 없음(프레임의 rpm·V·A는 그 함수에서 해석).
        // 변경/제약: 설명만 추가, 동작 유지. shared_ptr를 값으로 캡처하여 지연 dispatch가 module의 this에 접근하지 않는다.
        std::function<void(const gz::msgs::Float_V &)> callback =
            [state = _callback_state](const gz::msgs::Float_V &message) { on_message(*state, message); };
        if (!_node.Subscribe(_topic, callback)) {
            PX4_ERR("cannot subscribe to %s", _topic.c_str());
            return false;
        }
        return true;
    }

    // 목적: schema와 모터 프레임을 검사한 뒤 최신 유효 프레임을 공유 슬롯에 기록하고 worker를 깨운다.
    // 입출력: 공유 상태·Float_V → 4모터의 rpm/V/A, 로컬 수신 시각 µs·수신/거부 건수 갱신(반환값 없음).
    // 변경/제약: 설명만 추가, 동작 유지. schema 1개와 decoder 검사를 요구하며 직접 발행하지 않고 미소비 프레임은 덮어쓸 수 있다.
    static void on_message(CallbackState &state, const gz::msgs::Float_V &message)
    {
        bool schema_ok = false;
        unsigned schema_entries = 0;
        for (const auto &entry : message.header().data()) {
            if (entry.key() == "schema") {
                ++schema_entries;
                schema_ok = entry.value_size() == 1 && entry.value(0) == "arion.motor_state.v1";
            }
        }
        arion::MotorFrame frame{};
        if (!schema_ok || schema_entries != 1 ||
            !arion::decode_motor_frame(message.data().data(), message.data_size(), frame)) {
            ++state.rejected;
            return;
        }
        const uint64_t received_at = hrt_absolute_time();
        {
            std::lock_guard<std::mutex> guard(state.mutex);
            state.frame = frame;
            state.received_at = received_at;
            state.pending = true;
            ++state.received;
        }
        state.changed.notify_all();
    }

    std::shared_ptr<CallbackState> _callback_state{std::make_shared<CallbackState>()};
    std::shared_ptr<LifecycleState> _lifecycle_state{std::make_shared<LifecycleState>()};
    std::string _topic;
    param_t _stock_param{PARAM_INVALID};
    std::array<uint8_t, 4> _functions{};
    uint16_t _counter{0};
    std::atomic<unsigned> _published{0}, _blocked{0};
    uORB::Publication<esc_status_s> _status_pub{ORB_ID(esc_status)};
    uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
    actuator_armed_s _armed{};
    gz::transport::Node _node;
};

std::shared_ptr<ArionMotorBridge::LifecycleState> ArionMotorBridge::_lifecycle;

// 목적: PX4가 찾는 C ABI 모듈 진입점에서 bridge의 CLI 처리기로 위임한다.
// 입출력: argc/argv → ArionMotorBridge::main의 명령 결과 int(물리 단위 없음).
// 변경/제약: 설명만 추가, 동작 유지. 이 진입점 자체는 구독·발행·actuator 제어를 수행하지 않는다.
extern "C" __EXPORT int arion_motor_bridge_main(int argc, char *argv[])
{
    return ArionMotorBridge::main(argc, argv);
}
