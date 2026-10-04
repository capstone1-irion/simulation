"""Real local PTY regression for PX4's input-redraw prompts (no flight).

This child is a labeled console fixture, not PX4 or telemetry evidence.
The full integration test separately exercises the real PX4 console.
"""
import io
import os
from pathlib import Path
import pty
import subprocess
import sys
import tty
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_motor_fixed_e2e import Console


class ConsoleTest(unittest.TestCase):
    def test_command_waits_for_result_not_input_redraws(self):
        child = None
        master, slave = pty.openpty()
        try:
            tty.setraw(slave)
            child = subprocess.Popen([
                sys.executable, "-u", "-c",
                "import sys,time; sys.stdin.readline(); "
                "sys.stdout.write('\\x1b[2K\\rpxh> c'); sys.stdout.flush(); "
                "time.sleep(0.2); "
                "sys.stdout.write('\\x1b[2K\\rpxh> commander status\\r\\n'"
                " 'Fixture arming state: Disarmed\\r\\npxh> '); sys.stdout.flush(); "
                "time.sleep(0.5)"], stdin=slave, stdout=slave, stderr=slave)
            os.close(slave)
            slave = None
            reply = Console(child, master, io.StringIO()).command("commander status")
            self.assertIn("Fixture arming state: Disarmed", reply,
                          "an input-redraw prompt was mistaken for command completion")
        finally:
            try:
                if child is not None:
                    if child.poll() is None:
                        child.terminate()
                    child.wait(timeout=3)
            finally:
                try:
                    if slave is not None:
                        os.close(slave)
                finally:
                    os.close(master)

    def test_command_strips_color_but_preserves_raw_log(self):
        child = None
        master, slave = pty.openpty()
        try:
            tty.setraw(slave)
            child = subprocess.Popen([
                sys.executable, "-u", "-c",
                "import sys,time; sys.stdin.readline(); "
                "sys.stdout.write('pxh> listener esc_status -n 1\\r\\n' "
                "'\\x1b[0m    esc_count: \\x1b[0m4\\x1b[0m\\r\\npxh> '); "
                "sys.stdout.flush(); time.sleep(0.5)"],
                stdin=slave, stdout=slave, stderr=slave)
            os.close(slave)
            slave = None
            log = io.StringIO()
            reply = Console(child, master, log).command("listener esc_status -n 1")
            self.assertIn("esc_count: 4", reply)
            self.assertIn("\x1b[0m", log.getvalue(), "raw console evidence was lost")
        finally:
            try:
                if child is not None:
                    if child.poll() is None:
                        child.terminate()
                    child.wait(timeout=3)
            finally:
                try:
                    if slave is not None:
                        os.close(slave)
                finally:
                    os.close(master)


if __name__ == "__main__":
    unittest.main()
