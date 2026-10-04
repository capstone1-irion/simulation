"""Labeled local process/descriptor fixtures, NOT Gazebo or telemetry evidence."""
from argparse import Namespace
from contextlib import ExitStack, redirect_stdout
import errno
import io
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import weakref

import run_motor_fixed_e2e as e2e
import test_gazebo_motor as gazebo_tests
import test_px4_console as console_tests

REPO = Path(__file__).resolve().parents[1]


def unit_folder(prefix):
    root = Path(os.environ.get("ARION_TEST_ROOT", REPO / "experiments/motor-tests"))
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=prefix, dir=root))


def labeled_child(real_popen, mode="clean"):
    """A local Python process with a ready barrier; no sockets or telemetry."""
    handler = "signal.SIG_IGN" if mode == "stubborn" else "lambda *_: sys.exit(0)"
    script = ("import signal,sys,time; signal.signal(signal.SIGINT," + handler + "); "
              "print('LOCAL PROCESS UNIT FIXTURE READY',flush=True); " +
              ("raise SystemExit(7)" if mode == "nonzero" else "time.sleep(60)"))
    child = real_popen([sys.executable, "-u", "-c", script], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        if child.stdout is None or not select.select([child.stdout], [], [], 3)[0]:
            raise RuntimeError("local process fixture did not become ready")
        if child.stdout.readline().strip() != "LOCAL PROCESS UNIT FIXTURE READY":
            raise RuntimeError("unexpected local process fixture output")
        if mode == "nonzero":
            child.wait(timeout=3)
        return child
    except BaseException:
        if e2e.group_alive(child.pid):
            os.killpg(child.pid, signal.SIGKILL)
        child.wait(timeout=3)
        raise
    finally:
        if child.stdout is not None:
            child.stdout.close()


def reap_unit_child(child):
    if e2e.group_alive(child.pid):
        os.killpg(child.pid, signal.SIGKILL)
    child.wait(timeout=3)


class GazeboShutdownVerdictTest(unittest.TestCase):
    def test_successful_probe_cannot_hide_nonzero_server_exit(self):
        self._exercise("nonzero")

    def test_successful_probe_cannot_hide_forced_server_kill(self):
        self._exercise("stubborn")

    def test_exited_launcher_still_cleans_its_live_descendant_group(self):
        self._exercise("exited-launcher")

    def test_exited_launcher_cannot_hide_forced_descendant_kill(self):
        self._exercise("exited-stubborn-launcher")

    def _exercise(self, mode):
        folder = unit_folder("gazebo-shutdown-process-unit-")
        real_popen = subprocess.Popen
        children = []
        stopped = folder / "descendant-stopped.unit"
        descendant_pid = folder / "descendant-pid.unit"
        observation: dict = {"scope": "local Python process and probe-result fixtures; no Gazebo or telemetry",
                             "mode": mode}

        def launch(argv, **_):
            if mode.startswith("exited-"):
                child_script = (
                    "import os,signal,sys,time; from pathlib import Path; "
                    "signal.signal(signal.SIGINT," +
                    ("signal.SIG_IGN" if mode == "exited-stubborn-launcher" else
                     "lambda *_: (Path(" + repr(str(stopped)) + ").write_text('UNIT stopped'),sys.exit(0))") +
                    "); Path(" + repr(str(descendant_pid)) + ").write_text(str(os.getpid())); time.sleep(60)")
                launcher_script = ("import subprocess,sys; subprocess.Popen([sys.executable,'-u','-c'," +
                                   repr(child_script) + "]); print('LOCAL PROCESS UNIT FIXTURE READY',flush=True)")
                child = real_popen([sys.executable, "-u", "-c", launcher_script], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, start_new_session=True)
                children.append(child)
                if child.stdout is None or not select.select([child.stdout], [], [], 3)[0]:
                    raise RuntimeError("launcher unit fixture not ready")
                self.assertEqual(child.stdout.readline().strip(), "LOCAL PROCESS UNIT FIXTURE READY")
                child.stdout.close()
                child.wait(timeout=3)
                until = time.monotonic() + 3
                while not descendant_pid.exists() and time.monotonic() < until:
                    time.sleep(0.01)
                self.assertTrue(descendant_pid.exists())
                self.assertTrue(e2e.group_alive(child.pid), "fixture must have an exited launcher and a live group")
                observation["descendant_pid"] = int(descendant_pid.read_text())
            else:
                child = labeled_child(real_popen, mode)
                children.append(child)
            if mode in ("stubborn", "exited-stubborn-launcher"):
                real_wait = child.wait
                child.wait = lambda timeout=None: real_wait(timeout=min(timeout, 0.1) if timeout else timeout)
            return child

        def unit_tool(argv, **_):
            return subprocess.CompletedProcess(argv, 0, "UNIT probe-result fixture ONLY; no motor frames\n", "")

        try:
            with ExitStack() as stack:
                stack.enter_context(mock.patch.dict(os.environ, ARION_TEST_ROOT=str(folder)))
                stack.enter_context(mock.patch.object(gazebo_tests.subprocess, "check_output", return_value=""))
                stack.enter_context(mock.patch.object(gazebo_tests.subprocess, "run", side_effect=unit_tool))
                stack.enter_context(mock.patch.object(gazebo_tests.subprocess, "Popen", side_effect=launch))
                stack.enter_context(redirect_stdout(io.StringIO()))
                case = gazebo_tests.GazeboMotorTest("test_plugin_publishes_four_identifiable_motor_states")
                if mode == "exited-launcher":
                    case._run_probe()
                else:
                    with self.assertRaisesRegex(AssertionError, "unclean.*shutdown"):
                        case._run_probe()
            self.assertFalse(e2e.group_alive(children[0].pid), "owned descendants survived server cleanup")
            if mode == "exited-launcher":
                self.assertTrue(stopped.exists(), "exited launcher caused its live descendant to be skipped")
            paths = list(folder.glob("gazebo-contract-*/cleanup.json"))
            self.assertEqual(len(paths), 1)
            cleanup = json.loads(paths[0].read_text())
            self.assertFalse(cleanup["group_alive"])
            if "stubborn" in mode:
                self.assertTrue(cleanup["forced_kill"])
            if mode.startswith("exited-"):
                self.assertEqual(cleanup["returncode"], 0)
            elif mode == "nonzero":
                self.assertEqual(cleanup["returncode"], 7)
            observation["cleanup"] = cleanup
        finally:
            observation["groups_before_test_safety_cleanup"] = [e2e.group_alive(c.pid) for c in children]
            for child in children:
                child.wait = subprocess.Popen.wait.__get__(child, type(child))
                reap_unit_child(child)
            (folder / "unit-observation.json").write_text(json.dumps(observation, indent=2) + "\n")
        print("GAZEBO_SHUTDOWN_PROCESS_UNIT_EVIDENCE", folder)


class InterruptedCleanupTest(unittest.TestCase):
    def test_px4_cleanup_interrupt_still_cleans_both_groups_and_descriptors(self):
        self._run_case("px4")

    def test_gazebo_cleanup_interrupt_still_finishes_cleanup(self):
        self._run_case("gazebo")

    def test_receiver_cleanup_interrupt_retries_and_finishes_descriptors(self):
        self._run_case("receiver")

    def test_master_cleanup_interrupt_retries_and_finalizes_failed_report(self):
        self._run_case("master")

    def test_slave_cleanup_interrupt_after_start_failure_closes_both_ptys(self):
        self._run_case("slave")

    def test_repeated_process_cleanup_interrupt_uses_owned_group_fallback(self):
        self._run_case("px4", repeated=True)

    def test_interrupt_inside_process_wait_does_not_skip_the_other_group(self):
        self._run_case("wait")

    def _run_case(self, stage, repeated=False):
        folder = unit_folder("interrupted-cleanup-unit-")
        decoder = folder / "setup_fixture.py"
        decoder.write_text("# UNIT SETUP ONLY: no packets or telemetry generated.\n"
                           "mavlink_map = {291: None}\nclass MAVLink:\n"
                           "    def __init__(self, _): pass\n")
        args = Namespace(px4_source=REPO / "PX4-Autopilot", px4_binary=folder / "local-fixture",
                         plugin_build=folder, decoder=decoder, artifacts=folder, instance=31)
        real_popen, real_stop = subprocess.Popen, e2e.stop_owned
        real_socket, real_close, real_openpty = e2e.socket.socket, os.close, e2e.pty.openpty
        children, sockets, descriptors = [], [], []
        injected = []
        observation: dict = {"scope": "local process/descriptor fixtures; no telemetry or vehicle commands"}

        def interrupt():
            if not injected or repeated:
                injected.append(stage)
                raise KeyboardInterrupt("UNIT cleanup interruption at " + stage)

        def launch(argv, **kwargs):
            role = "gazebo" if argv[0] == "gz" else "px4"
            if role == "px4" and stage == "slave":
                raise FileNotFoundError("UNIT partial-start fixture; no PX4 launched")
            child = labeled_child(real_popen)
            child.unit_role = role
            children.append(child)
            if stage == "wait" and role == "px4":
                real_wait = child.wait

                def interrupted_wait(*wait_args, **wait_kwargs):
                    interrupt()
                    return real_wait(*wait_args, **wait_kwargs)

                child.wait = interrupted_wait
            return child

        def interrupted_stop(process, *stop_args, **stop_kwargs):
            if process is not None and process.unit_role == stage:
                interrupt()
            return real_stop(process, *stop_args, **stop_kwargs)

        def tracked_pty():
            pair = real_openpty()
            descriptors.extend(pair)
            return pair

        def interrupted_close(fd):
            target = 0 if stage == "master" else 1
            if stage in ("master", "slave") and descriptors and fd == descriptors[target]:
                interrupt()
            return real_close(fd)

        class TrackedSocket:
            def __init__(self, *socket_args, **socket_kwargs):
                self.inner = real_socket(*socket_args, **socket_kwargs)
                self.is_receiver = not sockets
                sockets.append(self)

            def __getattr__(self, name):
                return getattr(self.inner, name)

            def __enter__(self):
                return self

            def __exit__(self, *_):
                self.close()

            def close(self):
                if self.is_receiver and stage == "receiver":
                    interrupt()
                self.inner.close()

        before = len(os.listdir("/proc/self/fd"))
        try:
            with ExitStack() as stack:
                stack.enter_context(mock.patch.object(e2e.subprocess, "Popen", side_effect=launch))
                stack.enter_context(mock.patch.object(e2e, "stop_owned", side_effect=interrupted_stop))
                stack.enter_context(mock.patch.object(e2e.socket, "socket", TrackedSocket))
                stack.enter_context(mock.patch.object(e2e.pty, "openpty", side_effect=tracked_pty))
                stack.enter_context(mock.patch.object(e2e.os, "close", side_effect=interrupted_close))
                stack.enter_context(mock.patch.object(e2e.Console, "read_until",
                                                     side_effect=RuntimeError("UNIT stops before telemetry")))
                stack.enter_context(redirect_stdout(io.StringIO()))
                try:
                    result = e2e.run(args)
                except BaseException as error:
                    observation["escaped"] = type(error).__name__ + ": " + str(error)
                    result = None
            observation["groups_before_test_safety_cleanup"] = {
                child.unit_role: e2e.group_alive(child.pid) for child in children}
            observation["fds_before"] = before
            observation["fds_after"] = len(os.listdir("/proc/self/fd"))
            self.assertNotIn("escaped", observation, "interruption escaped cleanup: " + str(observation))
            self.assertEqual(result, 1)
            self.assertTrue(injected)
            self.assertFalse(any(observation["groups_before_test_safety_cleanup"].values()))
            self.assertEqual(observation["fds_after"], before)
            reports = list(folder.glob("px4-motor-fixed-*/report.json"))
            self.assertEqual(len(reports), 1)
            report = json.loads(reports[0].read_text())
            self.assertFalse(report["passed"])
            self.assertTrue(any("KeyboardInterrupt" in error for error in report["errors"]))
            self.assertIn("px4", report["cleanup"])
            self.assertIn("gazebo", report["cleanup"])
            for outcome in report["cleanup"].values():
                self.assertFalse(outcome["group_alive"])
            if repeated:
                self.assertTrue(report["cleanup"]["px4"]["forced_kill"])
        finally:
            for child in children:
                # Restore the fixture's wait before independent safety cleanup.
                child.wait = subprocess.Popen.wait.__get__(child, type(child))
                reap_unit_child(child)
            for tracked in sockets:
                tracked.inner.close()
            for fd in descriptors:
                try:
                    real_close(fd)
                except OSError:
                    pass
            (folder / "unit-observation.json").write_text(json.dumps(observation, indent=2) + "\n")
        print("INTERRUPTED_CLEANUP_UNIT_EVIDENCE", folder)


class ReceiverSetupFailureTest(unittest.TestCase):
    def test_setup_os_failures_release_owned_sockets_with_retained_tracebacks(self):
        for stage in ("receiver bind", "receiver timeout", "reservation create", "reservation bind",
                      "gazebo.log", "px4.log", "mavlink_raw.jsonl"):
            with self.subTest(stage=stage):
                self._exercise(stage)

    def _exercise(self, stage):
        folder = unit_folder("receiver-setup-resource-unit-")
        decoder = folder / "setup_fixture.py"
        decoder.write_text("# LOCAL RESOURCE SETUP ONLY: no packets or telemetry.\n"
                           "mavlink_map = {291: None}\nclass MAVLink:\n"
                           "    def __init__(self, _): pass\n")
        args = Namespace(px4_source=REPO / "PX4-Autopilot", px4_binary=folder / "unused-px4",
                         plugin_build=folder, decoder=decoder, artifacts=folder, instance=31)
        code = errno.EADDRINUSE if stage.endswith("bind") else \
               errno.EINVAL if stage == "receiver timeout" else errno.EMFILE
        failure = OSError(code, "LOCAL RESOURCE FIXTURE at " + stage)
        real_socket, real_open, real_record = e2e.socket.socket, Path.open, e2e.record_error
        sockets, retained = [], []
        observation = {"scope": "local owned loopback sockets/log setup; no Gazebo/PX4/telemetry",
                       "stage": stage, "injected_errno": code}

        class OwnedSocket(real_socket):
            def bind(self, address):
                role = "receiver" if sockets[0]() is self else "reservation"
                if stage == role + " bind":
                    raise failure
                return super().bind(address)

            def settimeout(self, value):
                if stage == "receiver timeout":
                    raise failure
                return super().settimeout(value)

        def acquire_socket(*args, **kwargs):
            if sockets and stage == "reservation create":
                raise failure
            owned = OwnedSocket(*args, **kwargs)
            sockets.append(weakref.ref(owned))  # The test does not keep sockets alive.
            return owned

        def open_log(path, *args, **kwargs):
            if path.name == stage and path.parent.parent == folder:
                raise failure
            return real_open(path, *args, **kwargs)

        def retain_error(report, where, error):
            if where == "run":
                retained.append(error)
            return real_record(report, where, error)

        before = len(os.listdir("/proc/self/fd"))
        try:
            with ExitStack() as stack:
                stack.enter_context(mock.patch.object(e2e.socket, "socket", side_effect=acquire_socket))
                stack.enter_context(mock.patch.object(Path, "open", new=open_log))
                stack.enter_context(mock.patch.object(e2e, "record_error", side_effect=retain_error))
                launch = stack.enter_context(mock.patch.object(e2e.subprocess, "Popen",
                    side_effect=AssertionError("local setup failure must not launch a process")))
                stack.enter_context(redirect_stdout(io.StringIO()))
                try:
                    result = e2e.run(args)
                except OSError as error:
                    retained.append(error)  # Keep the genuine escaping traceback alive for RED.
                    result = None
            observation["fds_before"] = before
            observation["fds_after_failure"] = len(os.listdir("/proc/self/fd"))
            observation["retained_errors"] = [str(error) for error in retained]
            observation["retained_tracebacks"] = [error.__traceback__ is not None for error in retained]
            observation["socket_fds_before_test_safety_cleanup"] = [
                None if ref() is None else ref().fileno() for ref in sockets]
            observation["result"] = result
            self.assertEqual(len(retained), 1)
            self.assertIs(retained[0], failure, "setup replaced the original OS error")
            self.assertIsNotNone(retained[0].__traceback__)
            launch.assert_not_called()
            self.assertEqual(observation["fds_after_failure"], before,
                             "setup failure retained a live socket in its traceback: " + str(observation))
            self.assertTrue(all(ref() is None or ref().fileno() == -1 for ref in sockets))
            self.assertEqual(result, 1)
            reports = list(folder.glob("px4-motor-fixed-*/report.json"))
            self.assertEqual(len(reports), 1)
            report = json.loads(reports[0].read_text())
            self.assertFalse(report["passed"])
            self.assertIn(str(failure), report["error"])
            self.assertTrue(all(not outcome["started"] for outcome in report["cleanup"].values()))
        finally:
            # Reclaim only these owned sockets, even when a RED assertion fails.
            for ref in sockets:
                if ref() is not None:
                    real_socket.close(ref())
            observation["fds_after_test_safety_cleanup"] = len(os.listdir("/proc/self/fd"))
            (folder / "unit-observation.json").write_text(json.dumps(observation, indent=2) + "\n")
            print("RECEIVER_SETUP_LOCAL_RESOURCE_EVIDENCE", folder)
            self.assertEqual(observation["fds_after_test_safety_cleanup"], before)


class ConsoleSetupFailureTest(unittest.TestCase):
    def test_both_console_methods_release_ptys_on_configuration_and_popen_failure(self):
        for method in ("test_command_waits_for_result_not_input_redraws",
                       "test_command_strips_color_but_preserves_raw_log"):
            for stage in ("setraw", "Popen"):
                for attempt in range(3):
                    with self.subTest(method=method, stage=stage, attempt=attempt):
                        self._exercise(method, stage)

    def _exercise(self, method, stage):
        folder = unit_folder("console-setup-resource-unit-")
        real_openpty, real_setraw, real_close = console_tests.pty.openpty, console_tests.tty.setraw, os.close
        descriptors, retained = [], []
        failure = OSError(errno.EAGAIN if stage == "Popen" else errno.EINVAL,
                          "LOCAL PTY RESOURCE FIXTURE at " + stage)
        observation = {"scope": "actual ConsoleTest methods with local PTYs; no PX4/telemetry/child launch",
                       "method": method, "stage": stage, "injected_errno": failure.errno}

        def acquire_pty():
            pair = real_openpty()
            descriptors.extend(pair)
            return pair

        def configure(fd):
            if stage == "setraw":
                raise failure
            return real_setraw(fd)

        before = len(os.listdir("/proc/self/fd"))
        try:
            with mock.patch.object(console_tests.pty, "openpty", side_effect=acquire_pty), \
                 mock.patch.object(console_tests.tty, "setraw", side_effect=configure), \
                 mock.patch.object(console_tests.subprocess, "Popen", side_effect=failure) as launch:
                try:
                    getattr(console_tests.ConsoleTest(method), method)()
                except OSError as error:
                    retained.append(error)  # Retain the actual method's traceback throughout observation.
            observation["pty_fds"] = descriptors
            observation["fds_before"] = before
            observation["fds_after_failure"] = len(os.listdir("/proc/self/fd"))
            observation["retained_errors"] = [str(error) for error in retained]
            observation["retained_tracebacks"] = [error.__traceback__ is not None for error in retained]
            self.assertEqual(len(descriptors), 2, "the actual fixture did not acquire its PTY pair")
            self.assertEqual(len(retained), 1)
            self.assertIs(retained[0], failure, "fixture replaced the original setup error")
            self.assertIsNotNone(retained[0].__traceback__)
            if stage == "Popen":
                launch.assert_called_once()
            else:
                launch.assert_not_called()
            self.assertEqual(observation["fds_after_failure"], before,
                             "actual console fixture leaked PTYs on partial setup: " + str(observation))
            for fd in descriptors:
                with self.assertRaises(OSError) as closed:
                    os.fstat(fd)
                self.assertEqual(closed.exception.errno, errno.EBADF)
        finally:
            # The baseline leaks integer descriptors; reclaim exactly this pair after RED.
            for fd in descriptors:
                try:
                    real_close(fd)
                except OSError as error:
                    if error.errno != errno.EBADF:
                        raise
            observation["fds_after_test_safety_cleanup"] = len(os.listdir("/proc/self/fd"))
            (folder / "unit-observation.json").write_text(json.dumps(observation, indent=2) + "\n")
            print("CONSOLE_SETUP_LOCAL_RESOURCE_EVIDENCE", folder)
            self.assertEqual(observation["fds_after_test_safety_cleanup"], before)


class StartupDescriptorTest(unittest.TestCase):
    def test_repeated_missing_and_nonexecutable_px4_start_does_not_leak_fds(self):
        folder = unit_folder("startup-descriptor-unit-")
        decoder = folder / "setup_fixture.py"
        decoder.write_text("# UNIT SETUP ONLY: no packets or telemetry generated.\n"
                           "mavlink_map = {291: None}\nclass MAVLink:\n"
                           "    def __init__(self, _): pass\n")
        nonexecutable = folder / "nonexecutable-px4-fixture"
        nonexecutable.write_text("UNIT STARTUP FIXTURE, NOT PX4\n")
        nonexecutable.chmod(0o600)
        real_popen = subprocess.Popen
        children = []

        def launch(argv, **kwargs):
            if argv[0] == "gz":
                child = labeled_child(real_popen)
                children.append(child)
                return child
            return real_popen(argv, **kwargs)

        counts = []
        try:
            with mock.patch.object(e2e.subprocess, "Popen", side_effect=launch), redirect_stdout(io.StringIO()):
                before = len(os.listdir("/proc/self/fd"))
                for binary in (folder / "missing-px4-fixture", nonexecutable):
                    for _ in range(6):
                        args = Namespace(px4_source=REPO / "PX4-Autopilot", px4_binary=binary,
                                         plugin_build=folder, decoder=decoder, artifacts=folder, instance=31)
                        self.assertEqual(e2e.run(args), 1)
                        after = len(os.listdir("/proc/self/fd"))
                        counts.append({"binary": str(binary), "before": before, "after": after})
                        self.assertEqual(after, before, "startup failure leaked a PTY descriptor")
                        self.assertFalse(e2e.group_alive(children[-1].pid))
            reports = sorted(folder.glob("px4-motor-fixed-*/report.json"))
            self.assertEqual(len(reports), 12)
            for path in reports:
                report = json.loads(path.read_text())
                self.assertFalse(report["passed"])
                self.assertTrue(any(kind in report["error"] for kind in
                                    ("FileNotFoundError", "PermissionError")))
                self.assertEqual(report["cleanup"]["gazebo"]["returncode"], 0)
        finally:
            for child in children:
                reap_unit_child(child)
            (folder / "descriptor-counts.json").write_text(json.dumps(
                {"scope": "labeled local startup fixtures; no telemetry", "counts": counts}, indent=2) + "\n")
        print("STARTUP_DESCRIPTOR_UNIT_EVIDENCE", folder)


if __name__ == "__main__":
    unittest.main(verbosity=2)
