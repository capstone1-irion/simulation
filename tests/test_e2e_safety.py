"""Fail-closed E2E verdict tests; unit fixtures never fabricate telemetry.

The opt-in shutdown regression uses real, disarmed PX4/Gazebo and the trusted
pinned decoder. Unit cleanup tests use only labeled local Python processes.
"""
from argparse import Namespace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_motor_fixed_e2e as e2e

REPO = Path(__file__).resolve().parents[1]


def evidence_folder(prefix):
    root = Path(os.environ.get("ARION_TEST_ROOT", REPO / "experiments" / "motor-tests"))
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=prefix, dir=root))


class CleanupTest(unittest.TestCase):
    def test_forced_kill_is_reported_for_an_owned_stubborn_process(self):
        folder = evidence_folder("cleanup-unit-")
        child = subprocess.Popen([
            sys.executable, "-u", "-c",
            "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "print('UNIT FIXTURE READY',flush=True); time.sleep(60)"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        try:
            if child.stdout is None:
                self.fail("unit fixture stdout pipe was not created")
            self.assertEqual(child.stdout.readline().strip(), "UNIT FIXTURE READY")
            real_wait = child.wait
            with mock.patch.object(child, "wait", side_effect=lambda timeout=None:
                                   real_wait(timeout=min(timeout, 0.2) if timeout else timeout)):
                outcome = e2e.stop_owned(child)
            (folder / "outcome.json").write_text(json.dumps(
                {"scope": "unit cleanup fixture only; no telemetry", "outcome": outcome,
                 "actual_returncode": child.poll()}, indent=2) + "\n")
            self.assertIsInstance(outcome, dict, "cleanup discarded forced-kill evidence")
            self.assertTrue(outcome["forced_kill"])
            self.assertEqual(outcome["returncode"], -signal.SIGKILL)
            self.assertFalse(outcome["group_alive"])
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=3)
            if child.stdout is not None:
                child.stdout.close()
        print("CLEANUP_UNIT_EVIDENCE", folder)

    def test_real_nonzero_exit_is_rejected_without_losing_returncode(self):
        child = subprocess.Popen([sys.executable, "-c", "raise SystemExit(7)"], start_new_session=True)
        child.wait(timeout=5)
        outcome = e2e.stop_owned(child)
        self.assertIsInstance(outcome, dict)
        self.assertEqual(outcome["returncode"], 7)
        report = {"passed": True, "cleanup": {"px4": outcome, "gazebo": self._clean()}}
        e2e.finalize_report(report, acceptance_complete=True)
        self.assertFalse(report["passed"])
        self.assertIn("7", report["error"])

    @staticmethod
    def _clean():
        # Cleanup metadata fixture only, not a simulator/telemetry result.
        return {"started": True, "returncode": 0, "sigint_sent": False,
                "forced_kill": False, "group_alive": False}

    def test_final_verdict_rejects_kill_survivor_exception_or_incomplete_acceptance(self):
        cases = [({"forced_kill": True}, True, None),
                 ({"group_alive": True}, True, None),
                 ({}, False, None), ({}, True, "shutdown exception"),
                 ({"started": False, "returncode": None}, True, None)]
        for changes, accepted, error in cases:
            with self.subTest(changes=changes, accepted=accepted, error=error):
                outcome = dict(self._clean(), **changes)
                report = {"passed": True, "cleanup": {"px4": outcome, "gazebo": self._clean()}}
                if error:
                    report["error"] = error
                e2e.finalize_report(report, acceptance_complete=accepted)
                self.assertFalse(report["passed"])

    def test_only_completed_acceptance_with_clean_cleanup_can_pass(self):
        report = {"passed": False, "cleanup": {"px4": self._clean(), "gazebo": self._clean()}}
        e2e.finalize_report(report, acceptance_complete=True)
        self.assertTrue(report["passed"])

    def test_cleanup_error_preserves_original_exception_evidence(self):
        report: dict = {"passed": False}
        e2e.record_error(report, "run", RuntimeError("original acceptance failure"))
        original = report["error"]
        e2e.record_error(report, "px4 cleanup", OSError("later cleanup failure"))
        e2e.finalize_report(report, acceptance_complete=False)
        self.assertEqual(report["error"], original)
        self.assertIn("original acceptance failure", report["errors"][0])
        self.assertIn("later cleanup failure", report["errors"][1])
        self.assertFalse(report["passed"])


class OptimizationTest(unittest.TestCase):
    def test_optimized_execution_rejects_before_decoder_or_process_setup(self):
        for flag, optimize in (("-O", None), ("-OO", None), (None, "1"), (None, "2")):
            with self.subTest(flag=flag, optimize=optimize):
                folder = evidence_folder("optimization-unit-")
                decoder = folder / "unit_decoder.py"
                decoder.write_text("# UNIT ONLY: intentionally no ESC_STATUS; no packets generated.\n"
                                   "mavlink_map = {}\nclass MAVLink:\n    def __init__(self, _): pass\n")
                options = {"px4_source": str(REPO / "PX4-Autopilot"),
                           "px4_binary": str(folder / "unused-px4"),
                           "plugin_build": str(folder), "decoder": str(decoder),
                           "artifacts": str(folder / "must-not-be-created")}
                code = (
                    "import sys; from pathlib import Path; from argparse import Namespace; "
                    "from unittest import mock; sys.path.insert(0," + repr(str(REPO / "tests")) + "); "
                    "import run_motor_fixed_e2e as e\n"
                    "args=Namespace(instance=31, **{k:Path(v) for k,v in " + repr(options) + ".items()})\n"
                    "with mock.patch.object(e.subprocess, 'Popen', side_effect=RuntimeError('PROCESS_LAUNCH_ATTEMPT')):\n"
                    "    try: e.run(args)\n"
                    "    except RuntimeError as error:\n"
                    "        if 'optimized' in str(error).lower():\n"
                    "            print('OPTIMIZED_REJECTED_BEFORE_SETUP'); raise SystemExit(0)\n"
                    "        raise\n"
                    "raise SystemExit('optimized run failed to reject disabled safety checks')\n")
                env = os.environ.copy()
                env.pop("PYTHONOPTIMIZE", None)
                if optimize:
                    env["PYTHONOPTIMIZE"] = optimize
                command = [sys.executable] + ([flag] if flag else []) + ["-c", code]
                result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=15)
                (folder / "result.log").write_text(result.stdout + result.stderr)
                self.assertEqual(result.returncode, 0, str(folder) + "\n" + result.stdout + result.stderr)
                self.assertIn("OPTIMIZED_REJECTED_BEFORE_SETUP", result.stdout)
                self.assertFalse(Path(options["artifacts"]).exists())


@unittest.skipUnless(os.environ.get("ARION_RUN_E2E_REGRESSION") == "1",
                     "opt in with ARION_RUN_E2E_REGRESSION=1 and binary/decoder paths")
class RealShutdownRegressionTest(unittest.TestCase):
    def test_shutdown_write_exception_cannot_leave_a_passing_report(self):
        self._run_shutdown_exception(OSError("injected shutdown write failure"))

    def test_shutdown_interrupt_cannot_leave_a_passing_report(self):
        self._run_shutdown_exception(KeyboardInterrupt("injected shutdown interrupt"))

    def test_px4_cleanup_interrupt_after_real_acceptance_finalizes_a_failed_report(self):
        self._run_shutdown_exception(KeyboardInterrupt("injected px4 cleanup interrupt"), cleanup_role="px4")

    def test_gazebo_cleanup_interrupt_after_real_acceptance_finalizes_a_failed_report(self):
        self._run_shutdown_exception(KeyboardInterrupt("injected gazebo cleanup interrupt"), cleanup_role="gazebo")

    def _run_shutdown_exception(self, failure, cleanup_role=None):
        root = evidence_folder("shutdown-regression-")
        args = Namespace(px4_source=REPO / "PX4-Autopilot",
                         px4_binary=Path(os.environ["ARION_PX4_BINARY"]).resolve(),
                         plugin_build=Path(os.environ["ARION_PLUGIN_BUILD"]).resolve(),
                         decoder=Path(os.environ["ARION_DECODER"]).resolve(),
                         artifacts=root, instance=31)
        real_write = os.write
        real_stop = e2e.stop_owned
        interrupted = []

        def fail_shutdown(fd, data):
            if data == b"shutdown\n":
                raise failure
            return real_write(fd, data)

        def interrupt_cleanup(process, *stop_args, **stop_kwargs):
            if process is not None:
                role = "gazebo" if process.args[0] == "gz" else "px4"
                if role == cleanup_role and not interrupted:
                    interrupted.append(role)
                    raise failure
            return real_stop(process, *stop_args, **stop_kwargs)

        target, attribute, callback = (e2e.os, "write", fail_shutdown) if cleanup_role is None else \
                                     (e2e, "stop_owned", interrupt_cleanup)
        with mock.patch.object(target, attribute, side_effect=callback):
            try:
                result = e2e.run(args)
            except BaseException:
                result = 1
        reports = list(root.glob("px4-motor-fixed-*/report.json"))
        self.assertEqual(len(reports), 1)
        report = json.loads(reports[0].read_text())
        self.assertFalse(report["passed"], "shutdown failed but report retained passed=true: " + str(reports[0]))
        self.assertIn(str(failure), report["error"])
        self.assertEqual(result, 1)
        self.assertGreaterEqual(report["phases"]["custom"]["esc_packets"], 5)
        for name, outcome in report["cleanup"].items():
            self.assertFalse(outcome["group_alive"], name + " owned group survived")
            self.assertFalse(e2e.group_alive(report["owned_pids"][name]))
        if cleanup_role is not None:
            self.assertEqual(interrupted, [cleanup_role])
            self.assertTrue(all(outcome["returncode"] == 0 and not outcome["forced_kill"]
                                for outcome in report["cleanup"].values()))
        print("REAL_SHUTDOWN_REGRESSION_EVIDENCE", reports[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
