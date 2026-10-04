"""Disarmed, receive-only motor telemetry test against real PX4 and Gazebo.

Only the owned PX4 console is used for telemetry settings/module lifecycle.
No MAVLink packet, heartbeat, arm, or flight command is transmitted by Python.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import socket
import statistics
import subprocess
import tempfile
import time

REPO = Path(__file__).resolve().parents[1]
EXPECTED = {"rpm": [1101, 2202, 3303, 4404],
            "voltage": [15.1, 15.2, 15.3, 15.4],
            "current": [1.1, 2.2, 3.3, 4.4]}


def group_alive(pgid):
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def cleanup_outcome(process):
    return {"started": process is not None, "returncode": None,
            "sigint_sent": False, "forced_kill": False, "group_alive": False}


def force_owned_stop(process, outcome):
    """Bounded fallback for the recorded session, even if its launcher exited."""
    if process is None:
        return
    if group_alive(process.pid):
        # Preserve escalation evidence if an interrupt occurs after delivery.
        outcome["forced_kill"] = True
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    deadline = time.monotonic() + 5
    while group_alive(process.pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    outcome.update(returncode=process.poll(), group_alive=group_alive(process.pid))


def stop_owned(process, outcome=None):
    # Retries share this dictionary: an interrupted wait must not erase an
    # earlier forced kill of descendants whose launcher had already exited 0.
    if outcome is None:
        outcome = cleanup_outcome(process)
    if process is None:
        return outcome
    deadline = time.monotonic() + 10
    if group_alive(process.pid) and not outcome["sigint_sent"]:
        outcome["sigint_sent"] = True
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        # A timed-out parent must not make a forced kill look like success.
        deadline = time.monotonic()
    while group_alive(process.pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    if group_alive(process.pid):
        force_owned_stop(process, outcome)
    outcome.update(returncode=process.poll(), group_alive=group_alive(process.pid))
    return outcome


def record_error(report, stage, error):
    evidence = stage + ": " + type(error).__name__ + ": " + str(error)
    report.setdefault("errors", []).append(evidence)
    report.setdefault("error", evidence)  # Keep the original failure, too.


def retry_cleanup(report, stage, action):
    """Record interrupts as failure, retry the same resource, then continue."""
    for _ in range(3):
        try:
            action()
            return True
        except BaseException as error:
            record_error(report, stage, error)
    return False


def cleanup_owned_process(report, name, process):
    outcome = cleanup_outcome(process)
    report.setdefault("cleanup", {})[name] = outcome
    completed = retry_cleanup(report, name + " cleanup", lambda: stop_owned(process, outcome))
    if process is not None and (not completed or outcome["group_alive"]):
        # Do not catch-and-skip a repeatedly interrupted first group. Escalate
        # that exact owned group, retaining evidence, before the next resource.
        retry_cleanup(report, name + " emergency cleanup", lambda: force_owned_stop(process, outcome))
    return outcome


def finalize_report(report, acceptance_complete):
    for name in ("px4", "gazebo"):
        outcome = report.get("cleanup", {}).get(name)
        if not outcome or not outcome["started"] or outcome["returncode"] != 0 or \
                outcome["forced_kill"] or outcome["group_alive"]:
            record_error(report, name + " cleanup", RuntimeError("unclean shutdown: " + str(outcome)))
    report["passed"] = acceptance_complete and not report.get("error") and not report.get("errors")


class Console:
    def __init__(self, process, fd, log):
        self.process, self.fd, self.log = process, fd, log

    def read_until(self, marker, timeout, after_line=None):
        deadline = time.monotonic() + timeout
        text = ""
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("PX4 exited before " + marker + ": " + text)
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if ready:
                block = os.read(self.fd, 65536).decode(errors="replace")
                text += block
                self.log.write(block)
                self.log.flush()
                start = 0
                if after_line is not None:
                    # PX4 redraws "pxh> <partial input>" on every keystroke.
                    # Wait for the complete echoed command line before accepting
                    # a completion prompt, including when both arrive together.
                    echoed = re.search(re.escape(after_line) + r"\r?\n", text)
                    if echoed is None:
                        continue
                    start = echoed.end()
                if marker in text[start:]:
                    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)
        raise TimeoutError("PX4 console wait exceeded " + str(timeout) + "s for " + marker)

    def command(self, command):
        os.write(self.fd, (command + "\n").encode())
        return self.read_until("pxh>", 20, after_line=command)


def run(args):
    if not __debug__:
        raise RuntimeError("optimized Python disables safety/acceptance checks; run without -O/-OO or PYTHONOPTIMIZE")
    args.artifacts.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix="px4-motor-fixed-", dir=args.artifacts))
    rootfs = folder / "rootfs"
    rootfs.mkdir()
    spec = importlib.util.spec_from_file_location("arion_pinned_decoder", args.decoder)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load the supplied decoder")
    decoder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(decoder)
    assert 291 in decoder.mavlink_map, "decoder lacks ESC_STATUS"
    parser = decoder.MAVLink(None)
    parser.robust_parsing = True
    gazebo = px4 = None
    acceptance_complete = False
    receiver = master = slave = None
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "passed": False,
              "scope": "fixed transport fixture; disarmed; no flight/AI/physical model validation",
              "receiver_transmits": False, "instance": args.instance,
              "phases": {}}
    # Own every acquired resource before bind, reservation or log setup can fail.
    try:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.1)
        destination_port = receiver.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            source_port = reservation.getsockname()[1]
        report.update(destination_port=destination_port, source_port=source_port)
        env = os.environ.copy()
        env.update({"GZ_PARTITION": "arion_e2e_" + folder.name, "GZ_IP": "127.0.0.1",
                    "HEADLESS": "1", "PX4_SYS_AUTOSTART": "4001",
                    "PX4_SIM_MODEL": "gz_custom_x500", "PX4_GZ_WORLD": "default",
                    "PX4_GZ_STANDALONE": "1", "PX4_GZ_MODELS": str(REPO / "models"),
                    "PX4_GZ_WORLDS": str(args.px4_source / "Tools/simulation/gz/worlds"),
                    "GZ_SIM_RESOURCE_PATH": str(REPO / "models") + ":" +
                                            str(args.px4_source / "Tools/simulation/gz/models"),
                    "GZ_SIM_SYSTEM_PLUGIN_PATH": str(args.plugin_build)})
        with (folder / "gazebo.log").open("w") as gz_log, (folder / "px4.log").open("w") as px_log, \
             (folder / "mavlink_raw.jsonl").open("w") as raw:
            def capture(phase, duration, require_esc=True):
                rows = []
                counts = Counter()
                until = time.monotonic() + duration
                while time.monotonic() < until:
                    try:
                        packet, peer = receiver.recvfrom(65535)
                    except socket.timeout:
                        continue
                    for message in parser.parse_buffer(packet) or []:
                        name = message.get_type()
                        counts[name] += 1
                        record = {"phase": phase, "receive_monotonic_ns": time.monotonic_ns(),
                                  "receive_utc_ns": time.time_ns(), "peer": peer,
                                  "source_system": message.get_srcSystem(),
                                  "source_component": message.get_srcComponent(),
                                  "fields": message.to_dict()}
                        raw.write(json.dumps(record, allow_nan=False) + "\n")
                        raw.flush()
                        if name == "ESC_STATUS":
                            rows.append(record)
                        if name == "HEARTBEAT":
                            assert not message.base_mode & decoder.MAV_MODE_FLAG_SAFETY_ARMED, "PX4 became armed"
                assert counts["BAD_DATA"] == 0 and not any(k.startswith("UNKNOWN_") for k in counts), counts
                if require_esc:
                    assert len(rows) >= 5, "too few real ESC_STATUS packets: " + str(counts)
                assert all(r["source_system"] == args.instance + 1 and r["source_component"] == 1 for r in rows), "unexpected sender"
                times = sorted({r["fields"]["time_usec"] for r in rows})
                intervals = [b - a for a, b in zip(times, times[1:])]
                summary = {"seen_by_type": dict(counts), "esc_packets": len(rows),
                           "unique_source_times": len(times), "first_actual_packet": rows[0] if rows else None}
                if intervals:
                    summary["median_source_period_us"] = statistics.median(intervals)
                report["phases"][phase] = summary
                return rows

            gazebo = subprocess.Popen(["gz", "sim", "-s", "-r",
                                       str(args.px4_source / "Tools/simulation/gz/worlds/default.sdf")],
                                      stdout=gz_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            master, slave = pty.openpty()
            px4 = subprocess.Popen([str(args.px4_binary), "-i", str(args.instance), "-w", str(rootfs),
                                    str(args.px4_binary.parent.parent / "etc")],
                                   stdin=slave, stdout=slave, stderr=slave, env=env,
                                   cwd=rootfs, start_new_session=True)
            os.close(slave)
            slave = None  # The parent no longer owns this descriptor.
            report["owned_pids"] = {"px4": px4.pid, "gazebo": gazebo.pid}
            console = Console(px4, master, px_log)
            console.read_until("pxh>", 90)
            status = console.command("commander status")
            assert "Disarmed" in status, status
            console.command("mavlink stop-all")
            console.command("param set MAV_PROTO_VER 2")
            console.command(f"mavlink start -u {source_port} -o {destination_port} -t 127.0.0.1 -m custom -r 100000")
            for stream, rate in [("HEARTBEAT", 2), ("ESC_STATUS", 20), ("ESC_INFO", 2), ("SERVO_OUTPUT_RAW_0", 20)]:
                console.command(f"mavlink stream -u {source_port} -s {stream} -r {rate}")
            report["link_status"] = console.command("mavlink status")
            baseline = capture("default", 2)
            assert all(all(v == 0 for v in row["fields"][key])
                       for row in baseline for key in ["voltage", "current"]), "default source changed"
            report["start_with_stock_enabled"] = console.command(
                f"arion_motor_bridge start -t /arion/custom_x500_{args.instance}/motor_state")
            assert "stock ESC publisher enabled" in report["start_with_stock_enabled"], \
                "bridge start returned before the ownership refusal was known"
            console.command("arion_motor_bridge stop")
            report["failed_start_status"] = console.command("arion_motor_bridge status")
            assert "not running" in report["failed_start_status"], report["failed_start_status"]
            console.command("param set SIM_GZ_ESC_ST 0")
            report["immediate_stop_statuses"] = []
            for _ in range(3):
                console.command(f"arion_motor_bridge start -t /arion/custom_x500_{args.instance}/motor_state")
                console.command("arion_motor_bridge stop")
                stopped = console.command("arion_motor_bridge status")
                report["immediate_stop_statuses"].append(stopped)
                assert "not running" in stopped, stopped
            console.command(f"arion_motor_bridge start -t /arion/custom_x500_{args.instance}/motor_state")
            capture("handover", 1)
            custom = capture("custom", 3)
            for row in custom:
                fields = row["fields"]
                assert fields["index"] == 0, fields
                for key, values in EXPECTED.items():
                    assert all(math.isclose(actual, wanted, rel_tol=0, abs_tol=0.001)
                               for actual, wanted in zip(fields[key], values)), (key, fields)
            report["uorb_custom"] = console.command("listener esc_status -n 1")
            assert "esc_count: 4" in report["uorb_custom"], report["uorb_custom"]
            for rpm in EXPECTED["rpm"]:
                assert f"esc_rpm: {rpm}" in report["uorb_custom"], report["uorb_custom"]
            report["bridge_status"] = console.command("arion_motor_bridge status")
            console.command("arion_motor_bridge stop")
            # A wrong/absent input topic must not silently restore command echo.
            console.command(f"arion_motor_bridge start -t /arion/custom_x500_{args.instance}/absent_motor_state")
            capture("missing_source_handover", 0.5, require_esc=False)
            quiet = capture("missing_source", 1, require_esc=False)
            assert not quiet, "stale state or command echo was published without input"
            report["missing_source_status"] = console.command("arion_motor_bridge status")
            assert "received: 0" in report["missing_source_status"] and \
                "published: 0" in report["missing_source_status"], report["missing_source_status"]
            console.command("arion_motor_bridge stop")
            console.command("param set SIM_GZ_ESC_ST 1")
            capture("restore_handover", 1)
            restored = capture("restored", 2)
            assert all(all(v == 0 for v in row["fields"][key])
                       for row in restored for key in ["voltage", "current"]), "stock source did not resume"
            final_status = console.command("commander status")
            assert "Disarmed" in final_status, final_status
            report["final_commander_status"] = final_status
            acceptance_complete = True
            os.write(master, b"shutdown\n")
            px4.wait(timeout=10)
    except BaseException as error:
        record_error(report, "run", error)
    finally:
        report["cleanup"] = {}
        for name, process in (("px4", px4), ("gazebo", gazebo)):
            cleanup_owned_process(report, name, process)
        report["descriptor_cleanup"] = {}
        if receiver is not None:
            report["descriptor_cleanup"]["receiver"] = retry_cleanup(report, "receiver cleanup", receiver.close)
        for name, fd in (("console master", master), ("console slave", slave)):
            if fd is not None:
                report["descriptor_cleanup"][name] = retry_cleanup(
                    report, name + " cleanup", lambda fd=fd: os.close(fd))
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        report["process_returncodes"] = {"px4": None if px4 is None else px4.poll(),
                                          "gazebo": None if gazebo is None else gazebo.poll()}
        finalize_report(report, acceptance_complete)
        (folder / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print("E2E_EVIDENCE", folder)
    print(json.dumps({"passed": report["passed"], "error": report.get("error"),
                      "phases": {k: v["esc_packets"] for k, v in report["phases"].items()}}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--px4-source", type=Path, required=True)
    ap.add_argument("--px4-binary", type=Path, required=True)
    ap.add_argument("--plugin-build", type=Path, required=True)
    ap.add_argument("--decoder", type=Path, required=True, help="Trusted generated Python decoder from pinned PX4 XML")
    ap.add_argument("--artifacts", type=Path, required=True)
    ap.add_argument("--instance", type=int, default=31)
    options = ap.parse_args()
    for key in ["px4_source", "px4_binary", "plugin_build", "decoder", "artifacts"]:
        setattr(options, key, getattr(options, key).resolve())
    if not 1 <= options.instance <= 253:
        ap.error("instance must be 1..253")
    raise SystemExit(run(options))
