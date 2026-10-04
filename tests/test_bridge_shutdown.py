"""Actual bridge + real pthread regressions, not simulator/telemetry evidence.

px4_usleep below honestly models an initialized lockstep clock: a frozen
simulated clock cannot finish a sleep. Product stop must work without thawing
it. Native joins in the bridge are counted separately from fixture safety
cleanup performed only AFTER recording a failed acceptance assertion.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time
import unittest

from test_bridge_lifecycle import STUBS

REPO = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <pthread.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <vector>

std::mutex sim_mutex;
std::condition_variable sim_changed;
uint64_t sim_time = 123456;
bool sim_advancing = true;
std::atomic<bool> ticker_exit{false}, worker_started{false};
std::atomic<unsigned> publications{0}, product_joins{0}, sim_sleep_calls{0};
std::atomic<unsigned> join_attempts{0}, active_joins{0}, peak_joins{0};
std::atomic<int> injected_join_error{0};
std::atomic<bool> delay_native_exit{false}, release_native{true}, native_waiting{false};
std::atomic<bool> native_join_complete{false}, hold_module_after_join{false};
std::atomic<bool> module_held_after_join{false}, release_module{false};
std::atomic<bool> finalizer_waiting{false}, release_finalizer{false};
thread_local bool fixture_is_worker = false;
thread_local bool delay_finalization = false;
thread_local unsigned command_module_locks = 0;
extern pthread_mutex_t px4_modules_mutex;
pthread_t worker_handle{};
struct NativeRecord { pthread_t handle; long tid; void *stack; size_t size; };
std::mutex native_mutex;
std::vector<NativeRecord> native_records;
std::string stack_mapping(void *stack) {
    std::ifstream maps("/proc/self/maps");
    std::string line;
    while (std::getline(maps, line)) {
        unsigned long low = 0, high = 0;
        if (std::sscanf(line.c_str(), "%lx-%lx", &low, &high) == 2 &&
            (unsigned long)stack >= low && (unsigned long)stack < high) { return line; }
    }
    return "absent";
}
bool tid_exists(long tid) {
    return access(("/proc/self/task/" + std::to_string(tid)).c_str(), F_OK) == 0;
}
struct FixtureStart { void *(*entry)(void *); void *arg; };
void *fixture_adapter(void *raw) {
    fixture_is_worker = true;
    auto data = *static_cast<FixtureStart *>(raw);
    delete static_cast<FixtureStart *>(raw);
    pthread_attr_t attr;
    void *stack = nullptr; size_t size = 0;
    if (pthread_getattr_np(pthread_self(), &attr) || pthread_attr_getstack(&attr, &stack, &size)) {
        std::_Exit(3);
    }
    pthread_attr_destroy(&attr);
    { std::lock_guard<std::mutex> guard(native_mutex);
      native_records.push_back({pthread_self(), syscall(SYS_gettid), stack, size}); }
    worker_started.store(true);
    return data.entry(data.arg);
}
int fixture_pthread_create(pthread_t *handle, const pthread_attr_t *attr,
                           void *(*entry)(void *), void *arg) {
    auto *data = new FixtureStart{entry, arg};
    const int result = pthread_create(handle, attr, fixture_adapter, data);
    std::printf("NATIVE_CREATE explicit_attributes=%d result=%d\n", attr != nullptr, result);
    if (result) { delete data; }
    else { worker_handle = *handle; }
    return result;
}
int fixture_clockjoin_np(pthread_t handle, void **value, clockid_t clock, const timespec *deadline) {
    ++join_attempts;
    const unsigned active = ++active_joins;
    unsigned peak = peak_joins.load();
    while (active > peak && !peak_joins.compare_exchange_weak(peak, active)) {}
    const int injected = injected_join_error.exchange(0);
    const int result = injected ? injected : pthread_clockjoin_np(handle, value, clock, deadline);
    --active_joins;
    std::printf("PRODUCT_NATIVE_JOIN result=%d clock=%d injected=%d\n", result, clock, injected);
    if (!result) { ++product_joins; }
    if (!result && hold_module_after_join.load()) {
        native_join_complete.store(true);
        while (!module_held_after_join.load()) { usleep(1000); }
    }
    return result;
}
int fixture_mutex_clocklock(pthread_mutex_t *mutex, clockid_t clock, const timespec *deadline) {
    if (delay_finalization && ++command_module_locks == 2) {
        finalizer_waiting.store(true);
        while (!release_finalizer.load()) { usleep(1000); }
    }
    return pthread_mutex_clocklock(mutex, clock, deadline);
}
[[noreturn]] void fixture_pthread_exit(void *value) {
    if (fixture_is_worker && delay_native_exit.load()) {
        native_waiting.store(true);
        // External native-wrapper scheduling seam, AFTER taskmap/argv/owner
        // cleanup; the actual pthread has NOT terminated or been reaped yet.
        while (!release_native.load()) { usleep(1000); }
    }
    pthread_exit(value);
}
#define pthread_clockjoin_np fixture_clockjoin_np
#define pthread_mutex_clocklock fixture_mutex_clocklock
#include "ARION_BRIDGE_SOURCE"
#undef pthread_clockjoin_np
#undef pthread_mutex_clocklock
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#define pthread_create fixture_pthread_create
#define pthread_exit fixture_pthread_exit
#include "ARION_TASKS_SOURCE"
#undef pthread_create
#undef pthread_exit
#pragma GCC diagnostic pop

pthread_mutex_t px4_modules_mutex = PTHREAD_MUTEX_INITIALIZER;
int px4_usleep(unsigned usec) {
    ++sim_sleep_calls;
    std::unique_lock<std::mutex> lock(sim_mutex);
    const uint64_t target = sim_time + usec;
    sim_changed.wait(lock, [&] { return sim_time >= target; });
    return 0;
}
uint64_t hrt_absolute_time() {
    std::lock_guard<std::mutex> guard(sim_mutex);
    return sim_time;
}
extern "C" {
void PRINT_MODULE_DESCRIPTION(const char *) {}
void PRINT_MODULE_USAGE_NAME(const char *, const char *) {}
void PRINT_MODULE_USAGE_COMMAND_DESCR(const char *, const char *) {}
void PRINT_MODULE_USAGE_PARAM_STRING(char, const char *, const char *, const char *, bool) {}
}

template<class Predicate> bool wait_until(Predicate predicate, int milliseconds) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (!predicate() && std::chrono::steady_clock::now() < until) { usleep(1000); }
    return predicate();
}
int main(int argc, char **argv) {
    if (argc != 2) { return 2; }
    const bool frozen_mode = !std::strcmp(argv[1], "frozen-clock");
    const bool native_mode = !std::strcmp(argv[1], "native-cycles");
    const bool error_mode = !std::strcmp(argv[1], "native-error");
    const bool budget_mode = !std::strcmp(argv[1], "native-deadline-budget");
    const bool concurrent_mode = !std::strcmp(argv[1], "concurrent-reap");
    const bool unsupported_mode = !std::strcmp(argv[1], "unsupported-platform");
    const bool coordination_mode = !std::strcmp(argv[1], "coordination-timeout");
    const bool finalization_mode = !std::strcmp(argv[1], "finalization-timeout");
    const bool generation_mode = !std::strcmp(argv[1], "generation-isolation");
    if (!frozen_mode && !native_mode && !error_mode && !budget_mode && !concurrent_mode &&
        !unsupported_mode && !coordination_mode && !finalization_mode && !generation_mode) { return 2; }
    std::puts("REAL PTHREAD UNIT: actual bridge + pinned ModuleBase/tasks; initialized frozen-clock seam, NOT real PX4/Gazebo");
    std::thread ticker([] {
        while (!ticker_exit.load()) {
            { std::lock_guard<std::mutex> guard(sim_mutex);
              if (sim_advancing) { sim_time += 1000; } }
            sim_changed.notify_all();
            usleep(1000);
        }
    });
    char name[] = "arion_motor_bridge", start[] = "start", stop[] = "stop";
    char flag[] = "-t", topic[] = "/unit/frozen/motor_state";
    char *start_args[] = {name, start, flag, topic, nullptr};
    char *stop_args[] = {name, stop, nullptr};
    char status[] = "status";
    char *status_args[] = {name, status, nullptr};
    auto finish = [&](bool failed) {
        ticker_exit.store(true); ticker.join();
        gz::transport::fixture_snapshot = {};
        return failed ? 1 : 0;
    };
    if (unsupported_mode) {
        const int started = arion_motor_bridge_main(4, start_args);
        size_t native_count;
        { std::lock_guard<std::mutex> guard(native_mutex); native_count = native_records.size(); }
        const bool failed = started != -1 || ArionMotorBridge::is_running() || native_count != 0;
        std::printf("PLATFORM_ASSERT start=%d native_create_count=%zu running=%d passed=%d\n",
                    started, native_count, ArionMotorBridge::is_running(), !failed);
        if (failed) { std::fprintf(stderr, "ACCEPTANCE_FAILURE: unsupported platform started a worker without a supported native-reap contract\n"); }
        if (started == 0 && arion_motor_bridge_main(2, stop_args)) { std::_Exit(3); }
        return finish(failed);
    }
    if (coordination_mode) {
        if (arion_motor_bridge_main(4, start_args) ||
            !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
        pthread_mutex_lock(&px4_modules_mutex);
        int stopped = 2;
        double elapsed = 0;
        std::atomic<bool> returned{false};
        std::thread stopper([&] {
            const auto began = std::chrono::steady_clock::now();
            stopped = arion_motor_bridge_main(2, stop_args);
            elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
            returned.store(true);
        });
        const bool bounded = wait_until([&] { return returned.load(); }, 5500);
        pthread_mutex_unlock(&px4_modules_mutex);
        stopper.join();
        const bool retained = ArionMotorBridge::is_running();
        const int restart = arion_motor_bridge_main(4, start_args);
        const bool failed = !bounded || stopped != -1 || elapsed < 4.8 || elapsed > 5.7 || !retained || restart != -1 ||
                            product_joins.load() != 0;
        std::printf("COORDINATION_TIMEOUT_ASSERT stop=%d wall_seconds=%.6f bounded=%d retained=%d restart=%d passed=%d\n",
                    stopped, elapsed, bounded, retained, restart, !failed);
        const int retried = arion_motor_bridge_main(2, stop_args);
        return finish(failed || retried != 0 || product_joins.load() != 1 || ArionMotorBridge::is_running());
    }
    if (finalization_mode) {
        hold_module_after_join.store(true);
        if (arion_motor_bridge_main(4, start_args) ||
            !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
        std::thread blocker([] {
            while (!native_join_complete.load()) { usleep(1000); }
            pthread_mutex_lock(&px4_modules_mutex);
            module_held_after_join.store(true);
            while (!release_module.load()) { usleep(1000); }
            pthread_mutex_unlock(&px4_modules_mutex);
        });
        const auto began = std::chrono::steady_clock::now();
        const int stopped = arion_motor_bridge_main(2, stop_args);
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
        const bool retained = ArionMotorBridge::is_running();
        const bool failed = stopped != -1 || elapsed < 4.8 || elapsed > 5.7 || !retained || product_joins.load() != 1;
        std::printf("FINALIZATION_TIMEOUT_ASSERT stop=%d wall_seconds=%.6f retained=%d joins=%u passed=%d\n",
                    stopped, elapsed, retained, product_joins.load(), !failed);
        release_module.store(true); blocker.join();
        const int restart = arion_motor_bridge_main(4, start_args);
        const int status_result = arion_motor_bridge_main(2, status_args);
        const int retried = arion_motor_bridge_main(2, stop_args);
        return finish(failed || restart != -1 || status_result != -1 || retried != 0 || product_joins.load() != 1 ||
                      join_attempts.load() != 1 || ArionMotorBridge::is_running());
    }
    if (generation_mode) {
        release_native.store(false); delay_native_exit.store(true);
        if (arion_motor_bridge_main(4, start_args) ||
            !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
        auto old_snapshot = gz::transport::fixture_snapshot;
        int first = 2, second = 2;
        std::atomic<bool> second_returned{false};
        std::thread old_finalizer([&] { delay_finalization = true; first = arion_motor_bridge_main(2, stop_args); });
        if (!wait_until([] { return native_waiting.load(); }, 2000)) { std::_Exit(3); }
        std::thread other([&] { second = arion_motor_bridge_main(2, stop_args); second_returned.store(true); });
        release_native.store(true);
        if (!wait_until([] { return finalizer_waiting.load(); }, 2000) ||
            !wait_until([&] { return second_returned.load(); }, 2000)) { std::_Exit(3); }
        other.join();
        worker_started.store(false);
        const int fresh_start = arion_motor_bridge_main(4, start_args);
        if (fresh_start || !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
        release_finalizer.store(true); old_finalizer.join();
        const bool fresh_retained = ArionMotorBridge::is_running();
        // An already-snapshotted old callback must not publish through the fresh owner.
        old_snapshot(gz::transport::fixture_message());
        usleep(20000);
        const bool old_callback_isolated = publications.load() == 0;
        gz::transport::fixture_snapshot(gz::transport::fixture_message());
        const bool fresh_published = wait_until([] { return publications.load() == 1; }, 2000);
        const int fresh_stop = arion_motor_bridge_main(2, stop_args);
        const bool failed = first || second || !fresh_retained || !old_callback_isolated || !fresh_published || fresh_stop ||
                            ArionMotorBridge::is_running() || product_joins.load() != 2 || peak_joins.load() != 1;
        std::printf("GENERATION_ASSERT old_stop=%d other_stop=%d fresh_start=%d fresh_retained=%d old_callback_isolated=%d fresh_published=%d fresh_stop=%d joins=%u passed=%d\n",
                    first, second, fresh_start, fresh_retained, old_callback_isolated, fresh_published, fresh_stop, product_joins.load(), !failed);
        return finish(failed);
    }
    if (error_mode) {
        bool failed = false;
        for (const int error : {EINVAL, ESRCH, EDEADLK}) {
            worker_started.store(false);
            const unsigned before = product_joins.load();
            if (arion_motor_bridge_main(4, start_args) ||
                !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
            auto snapshot = gz::transport::fixture_snapshot;
            injected_join_error.store(error);
            const int stopped = arion_motor_bridge_main(2, stop_args);
            const bool retained = ArionMotorBridge::is_running();
            const int status_result = arion_motor_bridge_main(2, status_args);
            const int restart = arion_motor_bridge_main(4, start_args);
            const bool current_failed = stopped != -1 || !retained || restart != -1 || status_result != -1 ||
                                        product_joins.load() != before;
            std::printf("NATIVE_ERROR_ASSERT injected=%d stop=%d retained=%d restart=%d status=%d product_joins=%u passed=%d\n",
                        error, stopped, retained, restart, status_result, product_joins.load(), !current_failed);
            if (current_failed) { failed = true; }
            const int retried = arion_motor_bridge_main(2, stop_args);
            if (retried || ArionMotorBridge::is_running() || product_joins.load() != before + 1) { failed = true; }
            snapshot(gz::transport::fixture_message());
        }
        return finish(failed);
    }
    if (budget_mode) {
        release_native.store(false); delay_native_exit.store(true);
        if (arion_motor_bridge_main(4, start_args) ||
            !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
        NativeRecord record;
        { std::lock_guard<std::mutex> guard(native_mutex); record = native_records.back(); }
        // Spend part of the SAME budget on real module coordination, then
        // leave the native wrapper pending. A fresh five-second join is wrong.
        pthread_mutex_lock(&px4_modules_mutex);
        int stopped = 2;
        double elapsed = 0;
        std::atomic<bool> returned{false}, entered{false};
        std::thread stopper([&] {
            const auto began = std::chrono::steady_clock::now();
            entered.store(true);
            stopped = arion_motor_bridge_main(2, stop_args);
            elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
            returned.store(true);
        });
        if (!wait_until([&] { return entered.load(); }, 2000)) { std::_Exit(3); }
        usleep(1200000);
        pthread_mutex_unlock(&px4_modules_mutex);
        const bool pending_native = wait_until([] { return native_waiting.load(); }, 2000);
        const bool bounded = wait_until([&] { return returned.load(); }, 5500);
        if (!bounded) { std::_Exit(3); }
        stopper.join();
        const bool retained = ArionMotorBridge::is_running();
        const bool native_alive = tid_exists(record.tid);
        const int restart = arion_motor_bridge_main(4, start_args);
        const int status_result = arion_motor_bridge_main(2, status_args);
        const bool failed = stopped != -1 || elapsed < 4.8 || elapsed > 5.7 || !pending_native || !retained ||
                            !native_alive || restart != -1 || status_result != -1 || product_joins.load() != 0;
        std::printf("NATIVE_DEADLINE_ASSERT stop=%d wall_seconds=%.6f coordination_seconds=1.2 native_pending=%d native_alive=%d retained=%d restart=%d status=%d joins=%u passed=%d\n",
                    stopped, elapsed, pending_native, native_alive, retained, restart, status_result, product_joins.load(), !failed);
        std::fflush(stdout);
        release_native.store(true);
        const int retried = arion_motor_bridge_main(2, stop_args);
        std::printf("NATIVE_DEADLINE_RETRY stop=%d joins=%u native_alive=%d\n", retried, product_joins.load(), tid_exists(record.tid));
        return finish(failed || retried != 0 || product_joins.load() != 1 || tid_exists(record.tid));
    }
    if (concurrent_mode) {
        bool failed = false;
        for (unsigned cycle = 0; cycle < 8; ++cycle) {
            release_native.store(false); delay_native_exit.store(true); native_waiting.store(false);
            worker_started.store(false);
            const unsigned before = product_joins.load(), attempts_before = join_attempts.load();
            if (arion_motor_bridge_main(4, start_args) ||
                !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
            auto snapshot = gz::transport::fixture_snapshot;
            int first = 2, second = 2, restart = 2;
            std::thread one([&] { first = arion_motor_bridge_main(2, stop_args); });
            std::thread two([&] { second = arion_motor_bridge_main(2, stop_args); });
            if (!wait_until([] { return native_waiting.load(); }, 2000)) { std::_Exit(3); }
            std::thread starter([&] { restart = arion_motor_bridge_main(4, start_args); });
            starter.join();
            usleep(20000); // let both callers coordinate with the same control block
            release_native.store(true);
            one.join(); two.join();
            const bool current_failed = first || second || restart != -1 || ArionMotorBridge::is_running() ||
                                        product_joins.load() != before + 1 || join_attempts.load() != attempts_before + 1 || peak_joins.load() != 1;
            std::printf("CONCURRENT_REAP_ASSERT cycle=%u stop1=%d stop2=%d restart_during_join=%d joins=%u attempts=%u peak_joiners=%u passed=%d\n",
                        cycle, first, second, restart, product_joins.load(), join_attempts.load(), peak_joins.load(), !current_failed);
            if (current_failed) { failed = true; }
            snapshot(gz::transport::fixture_message());
        }
        return finish(failed);
    }
    if (native_mode) {
        bool failed = false;
        for (unsigned cycle = 0; cycle < 16; ++cycle) {
            worker_started.store(false);
            if (arion_motor_bridge_main(4, start_args) != 0 ||
                !wait_until([] { return worker_started.load(); }, 2000)) { std::_Exit(3); }
            NativeRecord record;
            { std::lock_guard<std::mutex> guard(native_mutex); record = native_records.back(); }
            const std::string before = stack_mapping(record.stack);
            const int result = arion_motor_bridge_main(2, stop_args);
            const bool tid_gone = wait_until([&] { return !tid_exists(record.tid); }, 2000);
            bool taskmap_clear = true;
            pthread_mutex_lock(&task_mutex);
            for (const auto &task : taskmap) { if (task.isused) { taskmap_clear = false; } }
            pthread_mutex_unlock(&task_mutex);
            std::printf("NATIVE_CYCLE cycle=%u tid=%ld handle=%lu stack=%p bytes=%zu stop=%d tid_gone=%d taskmap_clear=%d product_joins=%u\n",
                        cycle, record.tid, (unsigned long)record.handle, record.stack, record.size,
                        result, tid_gone, taskmap_clear, product_joins.load());
            std::printf("NATIVE_MAP before=%s after=%s\n", before.c_str(), stack_mapping(record.stack).c_str());
            if (result || !tid_gone || !taskmap_clear || ArionMotorBridge::is_running()) { failed = true; }
        }
        const bool reaped = product_joins.load() == 16;
        failed = failed || !reaped;
        std::printf("NATIVE_REAP_ASSERT expected=16 actual_product_joins=%u passed=%d\n", product_joins.load(), !failed);
        std::fflush(stdout);
        if (failed) {
            std::fprintf(stderr, "ACCEPTANCE_FAILURE: successful product stop did not reap each real native worker\n");
        }
        if (!product_joins.load()) {
            // No fixture join before any acceptance/native-map measurement.
            for (const auto &record : native_records) {
                timespec deadline{}; clock_gettime(CLOCK_MONOTONIC, &deadline); deadline.tv_sec += 2;
                const int result = pthread_clockjoin_np(record.handle, nullptr, CLOCK_MONOTONIC, &deadline);
                std::printf("FIXTURE_SAFETY_JOIN_AFTER_FAILED_NATIVE_ASSERTION tid=%ld result=%d\n", record.tid, result);
                if (result) { std::_Exit(3); }
            }
        }
        ticker_exit.store(true); ticker.join();
        gz::transport::fixture_snapshot = {};
        return failed ? 1 : 0;
    }
    const int started = arion_motor_bridge_main(4, start_args);
    bool setup_ok = started == 0 && wait_until([] { return worker_started.load(); }, 2000);
    if (setup_ok) {
        gz::transport::fixture_snapshot(gz::transport::fixture_message());
        setup_ok = wait_until([] { return publications.load() != 0; }, 2000);
    }
    uint64_t frozen_at;
    { std::lock_guard<std::mutex> guard(sim_mutex); sim_advancing = false; frozen_at = sim_time; }
    // Give an old simulated-time worker sleep a chance to block in the seam.
    usleep(20000);
    std::atomic<bool> returned{false};
    int stopped = 2;
    const auto began = std::chrono::steady_clock::now();
    std::thread stopper([&] { stopped = arion_motor_bridge_main(2, stop_args); returned.store(true); });
    const bool bounded = wait_until([&] { return returned.load(); }, 5400);
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    uint64_t observed;
    { std::lock_guard<std::mutex> guard(sim_mutex); observed = sim_time; }
    const bool failed = !setup_ok || !bounded || stopped != 0 || observed != frozen_at;
    std::printf("FROZEN_ASSERT setup=%d stop_returned=%d stop=%d wall_seconds=%.6f initialized_sim=%llu observed_sim=%llu sim_sleep_calls=%u\n",
                setup_ok, bounded, stopped, elapsed, (unsigned long long)frozen_at,
                (unsigned long long)observed, sim_sleep_calls.load());
    std::fflush(stdout);
    if (failed) {
        std::fprintf(stderr, "ACCEPTANCE_FAILURE: same stop caller did not succeed while initialized simulation clock stayed frozen\n");
    }
    // Assertion/evidence above is final. Thaw ONLY for RED safety cleanup.
    if (!bounded) {
        { std::lock_guard<std::mutex> guard(sim_mutex); sim_advancing = true; }
        sim_changed.notify_all();
        std::puts("FIXTURE_SAFETY_CLEANUP_AFTER_FAILED_ASSERTION: clock thawed, not product success");
    }
    if (!wait_until([&] { return returned.load(); }, 2000)) { std::_Exit(3); }
    stopper.join();
    if (!product_joins.load()) {
        timespec deadline{}; clock_gettime(CLOCK_MONOTONIC, &deadline); deadline.tv_sec += 2;
        const int result = pthread_clockjoin_np(worker_handle, nullptr, CLOCK_MONOTONIC, &deadline);
        std::printf("FIXTURE_SAFETY_JOIN_AFTER_ASSERTION result=%d (NOT product reap evidence)\n", result);
        if (result) { std::_Exit(3); }
    }
    ticker_exit.store(true); ticker.join();
    gz::transport::fixture_snapshot = {};
    return failed ? 1 : 0;
}
'''


def recorded_run(folder, name, argv, *, timeout, env=None):
    started = time.monotonic()
    utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    child = None
    first_error = None
    cleanup = []
    stdout = stderr = ''
    try:
        child = subprocess.Popen(argv, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 env=env, start_new_session=True)
        stdout, stderr = child.communicate(timeout=timeout)
    except BaseException as error:
        first_error = repr(error)
        if child is not None:
            try:
                os.killpg(child.pid, signal.SIGTERM)
                cleanup.append('owned process group SIGTERM after first error')
            except ProcessLookupError:
                pass
            try:
                stdout, stderr = child.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                cleanup.append('owned process group SIGKILL after cleanup deadline')
                stdout, stderr = child.communicate(timeout=3)
        if child is None or not isinstance(error, subprocess.TimeoutExpired):
            raise
    finally:
        if child is not None:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                cleanup.append('owned process group killed in final safety guard')
                child.wait(timeout=3)
            for stream in (child.stdout, child.stderr):
                if stream is not None:
                    stream.close()
        (folder / (name + '.stdout.log')).write_text(stdout)
        (folder / (name + '.stderr.log')).write_text(stderr)
        (folder / (name + '.execution.json')).write_text(json.dumps({
            'argv': argv, 'cwd': str(Path.cwd()), 'returncode': None if child is None else child.poll(),
            'owned_pid': None if child is None else child.pid,
            'owned_pgid': None if child is None else child.pid,
            'started_utc': utc, 'elapsed_wall_seconds': time.monotonic() - started,
            'timeout_seconds': timeout, 'first_error': first_error, 'cleanup': cleanup,
            'environment': {key: (env or os.environ).get(key) for key in (
                'PYTHONDONTWRITEBYTECODE', 'TMPDIR', 'ARION_TEST_ARTIFACT_ROOT', 'ARION_TEST_ROOT',
                'ASAN_OPTIONS', 'UBSAN_OPTIONS')},
            'scope': 'actual bridge + pinned ModuleBase/tasks, real pthread; external services are unit seams',
        }, indent=2) + '\n')
    if child is None:
        raise RuntimeError('fixture process was not created; see execution evidence')
    if first_error:
        raise RuntimeError('fixture execution failed: ' + first_error + '; see ' + str(folder))
    result = subprocess.CompletedProcess(argv, child.returncode, stdout, stderr)
    return result


class BridgeShutdownTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get('ARION_TEST_ARTIFACT_ROOT', os.environ.get(
            'ARION_TEST_ROOT', REPO / 'experiments/motor-tests')))
        root.mkdir(parents=True, exist_ok=True)
        cls.folder = Path(tempfile.mkdtemp(prefix='bridge-shutdown-', dir=root))
        stubs = dict(STUBS)
        stubs['drivers/drv_hrt.h'] = '#pragma once\n#include <cstdint>\nuint64_t hrt_absolute_time();\n'
        stubs['uORB/Publication.hpp'] = '''
#pragma once
#define ORB_ID(name) 0
namespace uORB { template<class T> class Publication {
public: explicit Publication(int) {} bool publish(const T &) { ++publications; return true; }
}; }
'''
        stubs['px4_platform_common/tasks.h'] += '\nvoid px4_task_exit(int);\nconstexpr int PR_SET_NAME = 15;\n'
        stubs['px4_platform_common/tasks.h'] = stubs['px4_platform_common/tasks.h'].replace(
            'SCHED_DEFAULT = 0, SCHED_PRIORITY_DEFAULT = 0', 'SCHED_DEFAULT = SCHED_FIFO, SCHED_PRIORITY_DEFAULT = 50')
        stubs['px4_platform_common/log.h'] += '\n#define PX4_DEBUG(...) do {} while (0)\n'
        stubs['px4_platform_common/defines.h'] = '#define PX4_STACK_ADJUSTED(s) (1024 * 1024)\n'
        stubs['systemlib/err.h'] = '#pragma once\n'
        for name, content in stubs.items():
            target = cls.folder / 'stubs' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content)
        bridge = REPO / 'px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp'
        tasks = REPO / 'PX4-Autopilot/platforms/posix/src/px4/common/tasks.cpp'
        source = cls.folder / 'harness.cpp'
        source.write_text(HARNESS.replace('ARION_BRIDGE_SOURCE', str(bridge)).replace(
            'ARION_TASKS_SOURCE', str(tasks)))
        paths = [bridge, tasks, REPO / 'PX4-Autopilot/platforms/common/include/px4_platform_common/module.h',
                 Path(__file__), REPO / 'tests/test_bridge_lifecycle.py']
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
            raise RuntimeError('shutdown harness build failed: ' + str(cls.folder) + '\n' + result.stderr)
        cls.native_executable = cls.folder / 'harness-native'
        native_command = [arg for arg in command if arg not in (
            '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all')]
        native_command[native_command.index(str(cls.executable))] = str(cls.native_executable)
        result = recorded_run(cls.folder, 'build-native', native_command, timeout=60)
        if result.returncode:
            raise RuntimeError('native shutdown harness build failed: ' + str(cls.folder) + '\n' + result.stderr)
        cls.unsupported_executable = cls.folder / 'harness-unsupported'
        unsupported_command = [arg for arg in native_command if arg != '-D__PX4_LINUX']
        unsupported_command[unsupported_command.index(str(cls.native_executable))] = str(cls.unsupported_executable)
        result = recorded_run(cls.folder, 'build-unsupported', unsupported_command, timeout=60)
        if result.returncode:
            raise RuntimeError('unsupported-platform build failed: ' + str(cls.folder) + '\n' + result.stderr)
        print('BRIDGE_SHUTDOWN_EVIDENCE', cls.folder)

    def _check(self, mode, native=False):
        env = dict(os.environ, ASAN_OPTIONS='detect_stack_use_after_return=1:detect_leaks=1:halt_on_error=1',
                   UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        executable = self.unsupported_executable if mode == 'unsupported-platform' else (
            self.native_executable if native else self.executable)
        result = recorded_run(self.folder, mode, [str(executable), mode], timeout=15, env=env)
        self.assertEqual(result.returncode, 0, str(self.folder) + '\n' + result.stdout + result.stderr)

    def test_stop_finishes_without_advancing_initialized_frozen_simulation_clock(self):
        self._check('frozen-clock')

    def test_each_successful_stop_reaps_real_native_thread_without_harness_join(self):
        self._check('native-cycles', native=True)

    def test_native_join_error_retains_registration_until_product_retry_reaps(self):
        self._check('native-error')

    def test_one_wall_budget_includes_coordination_and_actual_native_exit(self):
        self._check('native-deadline-budget')

    def test_concurrent_stop_callers_serialize_native_reaping_and_deny_start(self):
        self._check('concurrent-reap')

    def test_unsupported_platform_refuses_spawn_without_native_resources(self):
        self._check('unsupported-platform', native=True)

    def test_module_coordination_timeout_retains_owner_until_product_retry(self):
        self._check('coordination-timeout')

    def test_finalization_timeout_retains_reaped_generation_without_double_join(self):
        self._check('finalization-timeout')

    def test_old_stop_cannot_clear_fresh_generation_or_publish_a_late_callback(self):
        self._check('generation-isolation')


if __name__ == '__main__':
    unittest.main(verbosity=2)
