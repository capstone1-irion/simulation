"""Real headless Gazebo test. No PX4, MAVLink, or flight commands."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from run_motor_fixed_e2e import cleanup_owned_process

REPO = Path(__file__).resolve().parents[1]


class GazeboMotorTest(unittest.TestCase):
    def test_plugin_publishes_four_identifiable_motor_states(self):
        self._run_probe()

    def test_plugin_publishes_at_twenty_hz_simulated_time(self):
        self._run_probe("cadence")

    def _run_probe(self, mode=None):
        root = Path(os.environ.get("ARION_TEST_ROOT", REPO / "experiments" / "motor-tests"))
        root.mkdir(parents=True, exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix="gazebo-contract-", dir=root))
        flags = shlex.split(subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "gz-transport13", "gz-msgs10"], text=True))
        probe = run / "gazebo_motor_probe"
        subprocess.run(["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                        str(REPO / "tests" / "gazebo_motor_probe.cpp"), "-o", str(probe), *flags],
                       check=True, timeout=60)
        env = os.environ.copy()
        env["GZ_PARTITION"] = "arion_motor_test_" + run.name
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.environ.get("ARION_PLUGIN_BUILD", "")
        env["GZ_SIM_RESOURCE_PATH"] = str(REPO / "tests")
        with (run / "gazebo.log").open("w") as log:
            server = subprocess.Popen(["gz", "sim", "-s", "-r", str(REPO / "tests" / "motor_fixture.sdf")],
                                      stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            try:
                args = [str(probe), "/arion/arion_fixture/motor_state"]
                if mode:
                    args.append(mode)
                result = subprocess.run(args, env=env,
                                        text=True, capture_output=True, timeout=15)
                (run / "probe.log").write_text(result.stdout + result.stderr)
                self.assertEqual(result.returncode, 0,
                                 "Real Gazebo motor fixture failed. Evidence: " + str(run) + "\n" +
                                 result.stdout + result.stderr)
            finally:
                report = {}
                outcome = cleanup_owned_process(report, "gazebo", server)
                (run / "cleanup.json").write_text(json.dumps(outcome, indent=2) + "\n")
                (run / "cleanup-errors.json").write_text(json.dumps(report.get("errors", []), indent=2) + "\n")
                self.assertTrue(outcome["returncode"] == 0 and not outcome["forced_kill"] and
                                not outcome["group_alive"] and not report.get("errors"),
                                "unclean Gazebo shutdown. Evidence: " + str(run) + ": " + str(report))
        print("GAZEBO_EVIDENCE", run)


if __name__ == "__main__":
    unittest.main(verbosity=2)
