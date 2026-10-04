"""Real pthread callback/lifecycle seam tests, NOT telemetry evidence.

Compile the actual bridge/probe with ASan and pinned ModuleBase/tasks.cpp.
Only external PX4 services and transport dispatch are fixtures. The transport
keeps a callback copy just as remote dispatch can do before Unsubscribe.
Real Gazebo and disarmed PX4 are exercised separately by the integration tests.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import time
import unittest

REPO = Path(__file__).resolve().parents[1]

STUBS = {
    "px4_platform_common/atomic.h": """
#pragma once
#include <atomic>
namespace px4 { template<class T> using atomic = std::atomic<T>;
using atomic_bool = std::atomic_bool; }
""",
    "px4_platform_common/time.h": """
#pragma once
#include <cstdint>
int px4_usleep(unsigned usec);
""",
    "px4_platform_common/log.h": """
#pragma once
#include <cstdio>
#define PX4_INFO(...) do { std::printf(__VA_ARGS__); std::printf("\\n"); } while (0)
#define PX4_WARN(...) PX4_INFO(__VA_ARGS__)
#define PX4_ERR(...) PX4_INFO(__VA_ARGS__)
""",
    "px4_platform_common/tasks.h": """
#pragma once
using px4_main_t = int (*)(int, char **);
using px4_task_t = int;
constexpr int PX4_OK = 0, PX4_ERROR = -1;
constexpr int SCHED_DEFAULT = 0, SCHED_PRIORITY_DEFAULT = 0;
px4_task_t px4_task_spawn_cmd(const char *, int, int, int, px4_main_t, char *const []);
int px4_task_delete(px4_task_t);
""",
    "systemlib/px4_macros.h": """
#pragma once
#include <cstdint>
#define __EXPORT
#define __BEGIN_DECLS extern "C" {
#define __END_DECLS }
""",
    "px4_platform_common/getopt.h": """
#pragma once
#include <cstdio>
#include <cstring>
inline int px4_getopt(int argc, char **argv, const char *, int *index, const char **value) {
    if (*index >= argc) { return EOF; }
    if (*index + 1 >= argc || std::strcmp(argv[*index], "-t")) { return '?'; }
    *value = argv[*index + 1]; *index += 2; return 't';
}
""",
    "px4_platform_common/posix.h": '#include <px4_platform_common/time.h>\n',
    "parameters/param.h": """
#pragma once
#include <cstdint>
#include <cstring>
using param_t = int;
constexpr param_t PARAM_INVALID = -1;
inline int fixture_stock_enabled = 0;
inline param_t param_find(const char *name) {
    return !std::strcmp(name, "SIM_GZ_ESC_ST") ? 0 : name[14] - '0';
}
inline int param_get(param_t handle, void *value) {
    *static_cast<int32_t *>(value) = handle == 0 ? fixture_stock_enabled : 100 + handle;
    return 0;
}
""",
    "drivers/drv_hrt.h": """
#pragma once
#include <cstdint>
inline uint64_t hrt_absolute_time() { return 123456; }
""",
    "uORB/Publication.hpp": """
#pragma once
#define ORB_ID(name) 0
namespace uORB { template<class T> class Publication {
public: explicit Publication(int) {} bool publish(const T &) { return true; }
}; }
""",
    "uORB/Subscription.hpp": """
#pragma once
namespace uORB { class Subscription {
public: explicit Subscription(int) {} template<class T> void update(T *) {}
}; }
""",
    "uORB/topics/actuator_armed.h": """
#pragma once
struct actuator_armed_s { bool armed = false; };
""",
    "uORB/topics/esc_status.h": """
#pragma once
#include <cstdint>
struct esc_status_s {
    struct Esc { uint64_t timestamp; int32_t esc_rpm; float esc_voltage, esc_current;
        uint8_t esc_address, actuator_function; } esc[4];
    uint64_t timestamp; uint16_t counter; uint8_t esc_count, esc_online_flags, esc_armed_flags;
};
""",
    "gz/transport/Node.hh": """
#pragma once
#include <gz/msgs/float_v.pb.h>
#include <functional>
#include <string>
namespace gz { namespace transport {
inline std::function<void(const gz::msgs::Float_V &)> fixture_callback, fixture_snapshot;
inline bool fixture_prefill = false;
inline gz::msgs::Float_V fixture_message() {
    gz::msgs::Float_V message;
    auto *schema = message.mutable_header()->add_data();
    schema->set_key("schema"); schema->add_value("arion.motor_state.v1");
    for (float value : {1101.f, 2202.f, 3303.f, 4404.f, 15.1f, 15.2f, 15.3f, 15.4f,
                        1.1f, 2.2f, 3.3f, 4.4f}) { message.add_data(value); }
    return message;
}
class Node {
public:
    ~Node() { fixture_callback = {}; }
    template<class Message> bool Subscribe(const std::string &,
                                           std::function<void(const Message &)> callback) {
        fixture_callback = callback; fixture_snapshot = callback;
        if (fixture_prefill) { for (int i = 0; i < 5; ++i) { callback(fixture_message()); } }
        return true;
    }
    template<class Owner, class Message> bool Subscribe(const std::string &topic,
                                                        void (Owner::*callback)(const Message &), Owner *owner) {
        return Subscribe<Message>(topic, std::function<void(const Message &)>(
            [owner, callback](const Message &message) { (owner->*callback)(message); }));
    }
    bool Unsubscribe(const std::string &) { fixture_callback = {}; return true; }
};
} }
""",
}

HARNESS = r'''
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <thread>
unsigned entry_delay_us = 1200000, deleted_tasks = 0, spawn_calls = 0;
std::atomic<bool> worker_pending{false};
std::atomic<unsigned> product_joins{0};
bool spawn_failure = false;
int fixture_clockjoin_np(pthread_t handle, void **value, clockid_t clock, const timespec *deadline) {
    const int result = pthread_clockjoin_np(handle, value, clock, deadline);
    std::printf("PRODUCT_NATIVE_JOIN result=%d clock=%d\n", result, clock);
    if (!result) { ++product_joins; }
    return result;
}
#define pthread_clockjoin_np fixture_clockjoin_np
#include "ARION_BRIDGE_SOURCE"
#undef pthread_clockjoin_np
#define main fixture_probe_main
#include "ARION_PROBE_SOURCE"
#undef main
struct FixtureStart { void *(*entry)(void *); void *argument; };
void *fixture_adapter(void *raw) {
    const auto data = *static_cast<FixtureStart *>(raw);
    delete static_cast<FixtureStart *>(raw);
    // External scheduling seam: a REAL owned pthread has not entered the bridge.
    std::this_thread::sleep_for(std::chrono::microseconds(entry_delay_us));
    worker_pending.store(false);
    return data.entry(data.argument);
}
int fixture_create(pthread_t *handle, const pthread_attr_t *attr,
                   void *(*entry)(void *), void *arg) {
    auto *data = new FixtureStart{entry, arg};
    worker_pending.store(true);
    const int result = pthread_create(handle, attr, fixture_adapter, data);
    if (result) { worker_pending.store(false); delete data; }
    return result;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#define pthread_create fixture_create
#define px4_task_spawn_cmd fixture_native_spawn
#define px4_task_delete fixture_native_delete
#include "ARION_TASKS_SOURCE"
#undef pthread_create
#undef px4_task_spawn_cmd
#undef px4_task_delete
#pragma GCC diagnostic pop
pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;
int px4_usleep(unsigned) {
    std::fprintf(stderr, "unexpected simulator-clock sleep in lifecycle control; use frozen-clock regression\n");
    std::abort();
}
px4_task_t px4_task_spawn_cmd(const char *name, int scheduler, int priority, int stack,
                             px4_main_t entry, char *const argv[]) {
    ++spawn_calls;
    return spawn_failure ? -1 : fixture_native_spawn(name, scheduler, priority, stack, entry, argv);
}
int px4_task_delete(px4_task_t task) { ++deleted_tasks; return fixture_native_delete(task); }
extern "C" {
void PRINT_MODULE_DESCRIPTION(const char *) {}
void PRINT_MODULE_USAGE_NAME(const char *, const char *) {}
void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *, const char *) {}
void PRINT_MODULE_USAGE_PARAM_STRING(char, const char *, const char *, const char *, bool) {}
}

int main(int argc, char **argv) {
    if (argc != 2) { return 2; }
    std::puts("REAL PTHREAD UNIT FIXTURE: pinned ModuleBase/tasks, services/entry delay are seams; no telemetry");
    char start[] = "start", topic_flag[] = "-t", topic[] = "/unit/motor_state";
    char *options[] = {start, topic_flag, topic, nullptr};
    if (!std::strcmp(argv[1], "bridge-snapshot")) {
        auto *owner = ArionMotorBridge::instantiate(3, options);
        if (!owner) { std::fprintf(stderr, "fixture initialization failed\n"); return 2; }
        auto callback = gz::transport::fixture_snapshot;
        delete owner;
        callback(gz::transport::fixture_message());
    } else if (!std::strcmp(argv[1], "probe-snapshot")) {
        gz::transport::fixture_prefill = true;
        char name[] = "probe";
        char *probe_options[] = {name, topic, nullptr};
        if (fixture_probe_main(2, probe_options) != 0) { return 2; }
        gz::transport::fixture_snapshot(gz::transport::fixture_message());
    } else if (!std::strcmp(argv[1], "initialization-failure")) {
        fixture_stock_enabled = 1;
        const int result = ArionMotorBridge::start_command_base(3, options);
        if (result == 0 || spawn_calls || worker_pending.load() || ArionMotorBridge::is_running()) {
            std::fprintf(stderr, "start acknowledged success before failed initialization; worker remains queued\n");
            return 1;
        }
    } else if (!std::strcmp(argv[1], "immediate-stop")) {
        if (ArionMotorBridge::start_command_base(3, options) != 0) { return 2; }
        const auto began = std::chrono::steady_clock::now();
        const int result = ArionMotorBridge::stop_command();
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
        std::printf("IMMEDIATE_ASSERT stop=%d wall_seconds=%.6f product_joins=%u\n", result, elapsed, product_joins.load());
        if (result != 0 || worker_pending.load() || ArionMotorBridge::is_running()) {
            std::fprintf(stderr, "immediate stop forgot the still-starting worker\n");
            return 1;
        }
        if (elapsed < 1.1 || elapsed > 3 || deleted_tasks != 0 || product_joins.load() != 1) { return 1; }
    } else if (!std::strcmp(argv[1], "stop-timeout")) {
        entry_delay_us = 6000000;
        if (ArionMotorBridge::start_command_base(3, options) != 0) { return 2; }
        const auto began = std::chrono::steady_clock::now();
        const int result = ArionMotorBridge::stop_command();
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
        std::printf("TIMEOUT_ASSERT stop=%d wall_seconds=%.6f pending=%d product_joins=%u\n",
                    result, elapsed, worker_pending.load(), product_joins.load());
        if (result != -1 || !worker_pending.load() || !ArionMotorBridge::is_running() || deleted_tasks != 0 ||
            elapsed < 4.8 || elapsed > 5.7 || product_joins.load() != 0 ||
            ArionMotorBridge::start_command_base(3, options) != -1) {
            std::fprintf(stderr, "timeout did not retain the precreated worker or deny restart\n");
            return 1;
        }
        // Product retry (NOT harness join) finishes delayed entry and native reap.
        if (ArionMotorBridge::stop_command() != 0 || worker_pending.load() || ArionMotorBridge::is_running() ||
            gz::transport::fixture_callback || product_joins.load() != 1) { return 1; }
        gz::transport::fixture_snapshot(gz::transport::fixture_message());
    } else if (!std::strcmp(argv[1], "spawn-failure")) {
        spawn_failure = true;
        if (ArionMotorBridge::start_command_base(3, options) == 0 || worker_pending.load() ||
            ArionMotorBridge::is_running() || gz::transport::fixture_callback) {
            std::fprintf(stderr, "spawn failure retained a running module or subscription\n");
            return 1;
        }
    } else { return 2; }
    gz::transport::fixture_snapshot = {};
    std::puts("snapshot dispatch after owner destruction was safe");
    return 0;
}
'''


class BridgeLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get("ARION_TEST_ARTIFACT_ROOT", os.environ.get(
            "ARION_TEST_ROOT", REPO / "experiments" / "motor-tests")))
        root.mkdir(parents=True, exist_ok=True)
        cls.folder = Path(tempfile.mkdtemp(prefix="bridge-lifecycle-", dir=root))
        stubs = dict(STUBS)
        stubs["px4_platform_common/tasks.h"] += "\nvoid px4_task_exit(int);\nconstexpr int PR_SET_NAME = 15;\n"
        stubs["px4_platform_common/log.h"] += "\n#define PX4_DEBUG(...) do {} while (0)\n"
        stubs["px4_platform_common/defines.h"] = "#define PX4_STACK_ADJUSTED(s) (1024 * 1024)\n"
        stubs["systemlib/err.h"] = "#pragma once\n"
        for name, content in stubs.items():
            target = cls.folder / "stubs" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content)
        source = cls.folder / "harness.cpp"
        source.write_text(HARNESS.replace("ARION_BRIDGE_SOURCE", str(
            REPO / "px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp")).replace(
            "ARION_PROBE_SOURCE", str(REPO / "tests/gazebo_motor_probe.cpp")).replace(
            "ARION_TASKS_SOURCE", str(REPO / "PX4-Autopilot/platforms/posix/src/px4/common/tasks.cpp")))
        paths = [REPO / "px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp",
                 REPO / "tests/gazebo_motor_probe.cpp", Path(__file__),
                 REPO / "PX4-Autopilot/platforms/common/include/px4_platform_common/module.h",
                 REPO / "PX4-Autopilot/platforms/posix/src/px4/common/tasks.cpp"]
        (cls.folder / "source-identities.json").write_text(json.dumps({
            str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, indent=2) + "\n")
        flags = shlex.split(subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "gz-msgs10"], text=True))
        cls.executable = cls.folder / "harness"
        command = ["c++", "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                   "-D__PX4_LINUX", "-D__PX4_POSIX", "-DENABLE_LOCKSTEP_SCHEDULER",
                   "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                   "-fno-sanitize-recover=all",
                   "-I" + str(cls.folder / "stubs"), "-I" + str(REPO / "include"),
                   "-I" + str(REPO / "PX4-Autopilot/platforms/common/include"),
                   str(source), "-o", str(cls.executable), "-pthread", *flags]
        began = time.monotonic()
        from test_bridge_shutdown import recorded_run
        result = recorded_run(cls.folder, "build", command, timeout=60)
        (cls.folder / "build-command.json").write_text(json.dumps({
            "argv": command, "returncode": result.returncode, "elapsed_wall_seconds": time.monotonic() - began},
            indent=2) + "\n")
        (cls.folder / "build.log").write_text(shlex.join(command) + "\n" + result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError("harness build failed: " + str(cls.folder) + "\n" + result.stderr)
        print("BRIDGE_LIFECYCLE_EVIDENCE", cls.folder)

    def _check(self, mode):
        env = os.environ.copy()
        env["ASAN_OPTIONS"] = "detect_stack_use_after_return=1:detect_leaks=1:halt_on_error=1"
        env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
        command = [str(self.executable), mode]
        began = time.monotonic()
        started = datetime.datetime.now(datetime.timezone.utc).isoformat()
        from test_bridge_shutdown import recorded_run
        result = recorded_run(self.folder, mode, command, env=env, timeout=10)
        (self.folder / (mode + ".log")).write_text(result.stdout + result.stderr)
        (self.folder / (mode + "-command.json")).write_text(json.dumps({
            "argv": command, "returncode": result.returncode, "started_utc": started,
            "elapsed_wall_seconds": time.monotonic() - began}, indent=2) + "\n")
        self.assertEqual(result.returncode, 0, mode + ": " + str(self.folder) + "\n" + result.stderr)

    def test_bridge_snapshotted_dispatch_survives_owner_destruction(self):
        self._check("bridge-snapshot")

    def test_probe_snapshotted_dispatch_survives_stack_return(self):
        self._check("probe-snapshot")

    def test_initialization_failure_is_synchronous_and_spawns_nothing(self):
        self._check("initialization-failure")

    def test_immediate_stop_finishes_a_delayed_starting_worker(self):
        self._check("immediate-stop")

    def test_stop_timeout_retains_delayed_worker_until_cooperative_completion(self):
        self._check("stop-timeout")

    def test_spawn_failure_releases_precreated_owner_and_subscription(self):
        self._check("spawn-failure")


if __name__ == "__main__":
    unittest.main(verbosity=2)
