"""Real pthread ownership/deadline regressions, NOT telemetry evidence.

Actual bridge, pinned ModuleBase and pinned POSIX tasks.cpp with ASan/UBSan.
External load hooks delay the real worker before/after acquiring its owner.
Product stop must reap the native worker; no harness join masks missing reaping.
"""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from test_bridge_lifecycle import STUBS
from test_bridge_shutdown import recorded_run

REPO = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <thread>
#include <time.h>
#include <type_traits>

thread_local bool fixture_is_worker = false;
std::atomic<unsigned> product_joins{0};
std::atomic<bool> release_worker{false}, worker_at_load{false}, worker_acquired{false};
bool delay_before_load = false;
void fixture_before_load() {
    if (!fixture_is_worker) { return; }
    worker_at_load.store(true);
    if (delay_before_load) { while (!release_worker.load()) { std::this_thread::yield(); } }
}
void fixture_after_load() {
    if (!fixture_is_worker) { return; }
    worker_acquired.store(true);
    // No cancellation point: cancel is not proof that the delayed virtual run()
    // access has finished. Only cooperative completion and native join suffice.
    if (!delay_before_load) { while (!release_worker.load()) { std::this_thread::yield(); } }
}
int fixture_clockjoin_np(pthread_t handle, void **, clockid_t clock, const timespec *deadline) {
    void *value = nullptr;
    const int result = pthread_clockjoin_np(handle, &value, clock, deadline);
    std::printf("PRODUCT_NATIVE_JOIN result=%d canceled=%d clock=%d\n", result, value == PTHREAD_CANCELED, clock);
    if (!result && value != PTHREAD_CANCELED) { ++product_joins; }
    return result;
}
#define pthread_clockjoin_np fixture_clockjoin_np
#include "ARION_BRIDGE_SOURCE"
#undef pthread_clockjoin_np
struct FixtureStart { void *(*entry)(void *); void *arg; };
void *fixture_adapter(void *raw) {
    const auto data = *static_cast<FixtureStart *>(raw);
    delete static_cast<FixtureStart *>(raw);
    fixture_is_worker = true;
    return data.entry(data.arg);
}
int fixture_create(pthread_t *handle, const pthread_attr_t *attr, void *(*entry)(void *), void *arg) {
    auto *data = new FixtureStart{entry, arg};
    const int result = pthread_create(handle, attr, fixture_adapter, data);
    if (result) { delete data; }
    return result;
}
// Pinned POSIX px4_prctl has an unused pid argument; keep bridge -Werror.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#define pthread_create fixture_create
#include "ARION_TASKS_SOURCE"
#undef pthread_create
#pragma GCC diagnostic pop

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;
int px4_usleep(unsigned) {
    std::fprintf(stderr, "unexpected simulator-clock sleep in worker/stop control; frozen-clock regression is authoritative\n");
    std::abort();
}
extern "C" {
void PRINT_MODULE_DESCRIPTION(const char *) {}
void PRINT_MODULE_USAGE_NAME(const char *, const char *) {}
void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *, const char *) {}
void PRINT_MODULE_USAGE_PARAM_STRING(char, const char *, const char *, const char *, bool) {}
}

bool wait_flag(const std::atomic<bool> &flag) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!flag.load() && std::chrono::steady_clock::now() < until) { usleep(1000); }
    return flag.load();
}
int main(int argc, char **argv) {
    if (argc != 2) { return 2; }
    std::puts("REAL PTHREAD UNIT: pinned POSIX tasks.cpp; no simulator/telemetry/vehicle commands");
    delay_before_load = !std::strcmp(argv[1], "late-entry");
    char name[] = "arion_motor_bridge", start[] = "start", stop[] = "stop";
    char flag[] = "-t", topic[] = "/unit/worker/motor_state";
    char *start_args[] = {name, start, flag, topic, nullptr};
    char *stop_args[] = {name, stop, nullptr};
    if (arion_motor_bridge_main(4, start_args) != 0 || !wait_flag(worker_at_load)) { return 2; }
    if (!delay_before_load && !wait_flag(worker_acquired)) { return 2; }
    const auto began = std::chrono::steady_clock::now();
    const int stopped = arion_motor_bridge_main(2, stop_args);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    const bool retained = ArionMotorBridge::is_running() && !!gz::transport::fixture_callback;
    const int restart = arion_motor_bridge_main(4, start_args);
    std::printf("TIMEOUT_ASSERT stop=%d elapsed=%.6f retained=%d restart=%d acquired=%d product_joins=%u\n",
                stopped, elapsed, retained, restart, worker_acquired.load(), product_joins.load());
    std::fflush(stdout);
    const bool failed = stopped != -1 || elapsed < 4.8 || elapsed > 5.7 || !retained || restart != -1 ||
                        product_joins.load() != 0;
    if (failed) {
        std::fprintf(stderr, "ACCEPTANCE_FAILURE: timeout freed a possibly executing owner or permitted restart\n");
    }
    // Release even after RED; ASan exercises the actual delayed virtual run().
    // Retried PRODUCT stop owns native reaping, not a fixture-only pthread_join.
    release_worker.store(true);
    const int retried = arion_motor_bridge_main(2, stop_args);
    std::printf("RETRY_ASSERT stop=%d product_joins=%u acquired=%d\n", retried, product_joins.load(), worker_acquired.load());
    if (retried || product_joins.load() != 1 || ArionMotorBridge::is_running() ||
        gz::transport::fixture_callback || !worker_acquired.load()) { return 1; }
    gz::transport::fixture_snapshot(gz::transport::fixture_message());
    if (arion_motor_bridge_main(4, start_args) != 0) { return 1; }
    if (arion_motor_bridge_main(2, stop_args) != 0) { return 1; }
    if (product_joins.load() != 2 || ArionMotorBridge::is_running()) { return 1; }
    gz::transport::fixture_snapshot = {};
    std::puts("live owner retained, restart denied, PRODUCT native reaping, fresh restart cleaned safely");
    return failed ? 1 : 0;
}
'''


class BridgeWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get('ARION_TEST_ARTIFACT_ROOT', os.environ.get(
            'ARION_TEST_ROOT', REPO / 'experiments/motor-tests')))
        root.mkdir(parents=True, exist_ok=True)
        cls.folder = Path(tempfile.mkdtemp(prefix='bridge-real-pthread-', dir=root))
        stubs = dict(STUBS)
        stubs['px4_platform_common/atomic.h'] = r'''
#pragma once
#include <atomic>
#include <type_traits>
void fixture_before_load();
void fixture_after_load();
namespace px4 {
template<class T> class atomic : public std::atomic<T> {
public:
    using std::atomic<T>::atomic;
    T load() const {
        if constexpr (std::is_pointer<T>::value) { fixture_before_load(); }
        T value = std::atomic<T>::load();
        if constexpr (std::is_pointer<T>::value) { fixture_after_load(); }
        return value;
    }
};
using atomic_bool = std::atomic_bool;
}
'''
        stubs['px4_platform_common/tasks.h'] += '\nvoid px4_task_exit(int);\nconstexpr int PR_SET_NAME = 15;\n'
        stubs['px4_platform_common/log.h'] += '\n#define PX4_DEBUG(...) do {} while (0)\n'
        # ASan needs extra stack; pinned task/cancel implementations are unchanged.
        stubs['px4_platform_common/defines.h'] = '#define PX4_STACK_ADJUSTED(s) (1024 * 1024)\n'
        stubs['systemlib/err.h'] = '#pragma once\n'
        for name, content in stubs.items():
            target = cls.folder / 'stubs' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content)
        bridge = REPO / 'px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp'
        tasks = REPO / 'PX4-Autopilot/platforms/posix/src/px4/common/tasks.cpp'
        source = cls.folder / 'harness.cpp'
        source.write_text(HARNESS.replace('ARION_BRIDGE_SOURCE', str(bridge)).replace('ARION_TASKS_SOURCE', str(tasks)))
        paths = [bridge, tasks, REPO / 'PX4-Autopilot/platforms/common/include/px4_platform_common/module.h',
                 Path(__file__), REPO / 'tests/test_bridge_lifecycle.py', REPO / 'tests/test_bridge_shutdown.py']
        (cls.folder / 'source-identities.json').write_text(json.dumps({
            str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2) + '\n')
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-msgs10'], text=True))
        cls.executable = cls.folder / 'harness'
        command = ['c++', '-std=c++17', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                   '-D__PX4_LINUX', '-D__PX4_POSIX', '-DENABLE_LOCKSTEP_SCHEDULER',
                   '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                   '-fno-sanitize-recover=all',
                   '-I' + str(cls.folder / 'stubs'), '-I' + str(REPO / 'include'),
                   '-I' + str(REPO / 'PX4-Autopilot/platforms/common/include'),
                   str(source), '-o', str(cls.executable), '-pthread', *flags]
        result = recorded_run(cls.folder, 'build', command, timeout=60)
        if result.returncode:
            raise RuntimeError('real pthread build failed: ' + str(cls.folder) + '\n' + result.stderr)
        print('BRIDGE_REAL_PTHREAD_EVIDENCE', cls.folder)

    def _check(self, mode):
        env = dict(os.environ, ASAN_OPTIONS='detect_stack_use_after_return=1:detect_leaks=1:halt_on_error=1',
                   UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        result = recorded_run(self.folder, mode, [str(self.executable), mode], timeout=15, env=env)
        self.assertEqual(result.returncode, 0, str(self.folder) + '\n' + result.stdout + result.stderr)

    def test_worker_acquired_owner_is_retained_across_stop_deadline(self):
        self._check('acquired-owner')

    def test_late_entry_timeout_denies_restart_until_worker_completes(self):
        self._check('late-entry')


if __name__ == '__main__':
    unittest.main(verbosity=2)
