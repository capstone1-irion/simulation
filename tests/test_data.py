"""역할: 수집·재생·인과 전처리 계약의 집중 검사(작성만, 실행 보류).
입력: 로컬 합성 UDP, 임시 JSONL/최소 DataFlash BIN. 기업 원자료는 읽지 않는다.
출력: unittest 결과와 hermes/test-data 아래의 일시 파일.
호출자: 전체 코드·문서가 준비된 뒤 부모 작업자의 단 한 번 통합 검증.
"""

import json
import math
from pathlib import Path
import socket
import struct
import tempfile
import time
import unittest
from unittest.mock import patch

from capstone.collector import Collector, RAW_SCHEMA, decode_json, encode_json, load_dialect
from capstone.preprocess import Preprocessor, preprocess_file
from capstone.replay import iter_records


ROOT = Path(__file__).resolve().parents[1]


def record(name, time_ns, fields, *, source_kind="mavlink_udp", run_id="synthetic", component=1):
    """합성 입력임을 식별하고 원천 시계와 수신 시계를 따로 만든다."""
    native = None
    if source_kind == "ardupilot_bin":
        if "TimeUS" in fields:
            native = {"field": "TimeUS", "value": fields["TimeUS"], "unit": "us", "clock": "boot"}
    elif "time_usec" in fields:
        native = {"field": "time_usec", "value": fields["time_usec"], "unit": "us", "clock": "boot_or_unix"}
    return {
        "schema": RAW_SCHEMA, "kind": "message", "run_id": run_id, "epoch": 0,
        "source_kind": source_kind, "message_name": name, "message_id": None,
        "source_system": 7 if source_kind == "mavlink_udp" else None,
        "source_component": component if source_kind == "mavlink_udp" else None,
        "packet_sequence": None, "peer": None, "receive_wall_ns": None,
        "receive_monotonic_ns": time_ns if source_kind == "mavlink_udp" else None,
        "source_time": native, "fields": fields, "raw_hex": "",
    }


def battery(voltage=4000, current=100, soc=80, battery_id=255):
    """PX4 재출력과 같은 ID의 합성 네 셀 배터리 필드를 만든다."""
    return {"id": battery_id, "voltages": [voltage] * 4 + [65535] * 6,
            "current_battery": current, "battery_remaining": soc, "current_consumed": 10}


class DataTests(unittest.TestCase):
    """모든 임시 산출물을 V2 소유 hermes 경로 안에 한정한다."""

    def setUp(self):
        base = ROOT / "hermes" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=base)
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)

    def wait_until(self, predicate):
        """비동기 수집 검사의 대기를 제한한다."""
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("수집 스레드가 제한 시간 안에 조건을 충족하지 못함")

    def test_tagged_json_keeps_nonfinite_and_bytes(self):
        text = encode_json({"v": [math.nan, math.inf, -math.inf], "b": b"\x00\xff"})
        self.assertNotIn("NaN", text)
        restored = decode_json(text)
        self.assertTrue(math.isnan(restored["v"][0]))
        self.assertEqual(restored["v"][1:], [math.inf, -math.inf])
        self.assertEqual(restored["b"], b"\x00\xff")
        with self.assertRaises(ValueError):
            decode_json('{"bad": NaN}')

    def test_custom_dialect_does_not_replace_pymavlink_global(self):
        from pymavlink import mavutil
        original = mavutil.mavlink
        fixture = self.path / "test_dialect.py"
        fixture.write_text("class MAVLink:\n    pass\n", encoding="utf-8")
        loaded = load_dialect(fixture)
        self.assertEqual(loaded.MAVLink.__name__, "MAVLink")
        self.assertIs(mavutil.mavlink, original)
        self.assertIsNot(load_dialect(fixture).MAVLink, loaded.MAVLink)

    def test_collector_is_exclusive_receive_only_and_identity_filtered(self):
        from pymavlink.dialects.v20 import common
        target = self.path / "capture"
        collector = Collector("127.0.0.1", 0, target)
        collector.start()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.bind(("127.0.0.1", 0))
        sender.settimeout(0.1)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as competitor:
                with self.assertRaises(OSError):
                    competitor.bind(("127.0.0.1", collector.port))
            address = ("127.0.0.1", collector.port)
            for system, component in ((7, 180), (7, 1), (8, 1)):
                encoder = common.MAVLink(None, srcSystem=system, srcComponent=component)
                heartbeat = common.MAVLink_heartbeat_message(2, 12, 0, system, 4, 3)
                sender.sendto(heartbeat.pack(encoder), address)
            encoder = common.MAVLink(None, srcSystem=7, srcComponent=1)
            attitude = common.MAVLink_attitude_message(10, math.nan, 0, 0, 0, 0, 0)
            sender.sendto(attitude.pack(encoder), address)
            sender.sendto(b"not a MAVLink frame", address)
            self.wait_until(lambda: collector.snapshot()["datagrams"] == 5)
            self.wait_until(lambda: "roll" in collector.latest("ATTITUDE"))
            self.assertEqual(collector.snapshot()["system_id"], 7)
            self.assertEqual(collector.latest("HEARTBEAT")["custom_mode"], 7)
            self.assertTrue(math.isnan(collector.latest("ATTITUDE")["roll"]))
            with self.assertRaises(socket.timeout):
                sender.recvfrom(4096)
        finally:
            sender.close()
            summary = collector.stop()
        self.assertIsNone(summary["error"])
        saved = list(iter_records(target))
        packets = [item for item in saved if item["kind"] == "datagram"]
        self.assertEqual(len(packets), 5)
        self.assertEqual(bytes.fromhex(packets[-1]["raw_hex"]), b"not a MAVLink frame")
        messages = [item for item in saved if item["kind"] == "message"]
        self.assertIn(180, [item["source_component"] for item in messages])
        saved_attitude = next(item for item in messages if item["message_name"] == "ATTITUDE")
        self.assertTrue(math.isnan(saved_attitude["fields"]["roll"]))
        self.assertEqual(collector.latest("ATTITUDE"), {})
        self.assertEqual(collector.stop()["saved_records"], summary["saved_records"])
        self.assertEqual(collector.close()["saved_records"], summary["saved_records"])

    def test_callback_failure_is_exposed_and_persisted(self):
        def reject(item):
            if item["kind"] == "datagram":
                raise RuntimeError("합성 소비자 실패")
        collector = Collector("127.0.0.1", 0, self.path / "callback", on_record=reject)
        collector.start()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(b"x", ("127.0.0.1", collector.port))
        self.wait_until(lambda: collector.snapshot()["error"] is not None)
        with self.assertRaises(RuntimeError):
            collector.stop()
        summary = json.loads((self.path / "callback" / "summary.json").read_text())
        self.assertIn("합성 소비자 실패", summary["error"])

    def test_latest_requires_receipt_after_boundary_without_changing_stamp(self):
        collector = Collector("127.0.0.1", 0, self.path / "synthetic-latest", system_id=7)
        collector._running = True  # 수신 스레드 없이 시간/식별 필터만 검사한다.
        fields = {"base_mode": 29, "autopilot": 12}
        collector._update_latest(record("HEARTBEAT", 9_000_000_000, fields))
        with patch("capstone.collector.time.monotonic_ns", return_value=10_000_000_000):
            self.assertEqual(collector.latest("HEARTBEAT"), fields)
            self.assertEqual(collector.latest("HEARTBEAT", since_monotonic_ns=9_000_000_000), {})
            collector._update_latest(record("HEARTBEAT", 9_500_000_000, fields, component=180))
            wrong = record("HEARTBEAT", 9_500_000_000, fields)
            wrong["source_system"] = 8
            collector._update_latest(wrong)
            self.assertEqual(collector.latest("HEARTBEAT", since_monotonic_ns=9_000_000_000), {})
            collector._update_latest(record("HEARTBEAT", 9_500_000_000, fields))
            result = collector.latest("HEARTBEAT", since_monotonic_ns=9_000_000_000)
            self.assertEqual(result, fields)
            result["base_mode"] = 0
        self.assertEqual(collector._latest["HEARTBEAT"], (9_500_000_000, fields))

    def test_latest_boundary_does_not_bypass_age_stop_or_failure(self):
        collector = Collector("127.0.0.1", 0, self.path / "synthetic-age", system_id=7)
        collector._running = True
        fields = {"base_mode": 29}
        with patch("capstone.collector.time.monotonic_ns", return_value=10_000_000_000):
            for stamp in (1_000_000_000, 11_000_000_000):
                collector._update_latest(record("HEARTBEAT", stamp, fields))
                self.assertEqual(collector.latest("HEARTBEAT", since_monotonic_ns=0), {})
            collector._update_latest(record("HEARTBEAT", 9_000_000_000, fields))
            collector._running = False
            self.assertEqual(collector.latest("HEARTBEAT", since_monotonic_ns=0), {})
            collector._running = True
            collector._failure = RuntimeError("synthetic_failure")
            self.assertEqual(collector.latest("HEARTBEAT", since_monotonic_ns=0), {})

    def test_latest_rejects_invalid_receipt_boundary(self):
        collector = Collector("127.0.0.1", 0, self.path / "synthetic-boundary", system_id=7)
        for value in (-1, True, 1.5, "1", float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                collector.latest("HEARTBEAT", since_monotonic_ns=value)

    def test_writer_failure_remains_an_error_on_every_stop(self):
        collector = Collector("127.0.0.1", 0, self.path / "io")
        collector.start()
        with patch.object(collector, "_write", side_effect=OSError("합성 저장 실패")):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.sendto(b"x", ("127.0.0.1", collector.port))
            self.wait_until(lambda: collector.snapshot()["error"] is not None)
            for _ in range(2):
                with self.assertRaises(RuntimeError):
                    collector.stop()
        summary = json.loads((self.path / "io" / "summary.json").read_text())
        self.assertIn("합성 저장 실패", summary["error"])

    def test_jsonl_rotation_is_numeric_and_partial_lines_fail(self):
        for suffix, value in ((".000010", 10), ("", 0), (".000002", 2)):
            item = record("ATTITUDE", value, {"roll": value})
            (self.path / f"mavlink_raw{suffix}.jsonl").write_text(encode_json(item) + "\n")
        messages = [item for item in iter_records(self.path) if item["kind"] == "message"]
        self.assertEqual([item["fields"]["roll"] for item in messages], [0, 2, 10])
        broken = self.path / "broken.jsonl"
        broken.write_text('{"schema":')
        with self.assertRaisesRegex(ValueError, "broken.jsonl:1"):
            list(iter_records(broken))

    def test_real_dfreader_synthetic_bin_keeps_native_time_and_tail(self):
        """실제 DFReader API를 쓰되 기업 BIN 대신 작은 명시적 합성 파일을 쓴다."""
        def fmt(type_id, name, codes, columns, packing):
            return b"\xa3\x95\x80" + struct.pack(
                "<BB4s16s64s", type_id, struct.calcsize(packing) + 3,
                name.encode(), codes.encode(), columns.encode())
        data = fmt(129, "BAT", "QBfff", "TimeUS,Instance,Volt,Curr,RemPct", "<QBfff")
        data += fmt(130, "MSG", "Z", "Message", "<64s")
        data += b"\xa3\x95\x81" + struct.pack("<QBfff", 1234567, 0, 48.0, -0.5, 80.0)
        data += b"\xa3\x95\x82" + struct.pack("<64s", b"synthetic, no TimeUS")
        data += b"\x00\x01"
        source = self.path / "synthetic.BIN"
        source.write_bytes(data)
        records = list(iter_records(source))
        bat = next(item for item in records if item["message_name"] == "BAT")
        msg = next(item for item in records if item["message_name"] == "MSG")
        self.assertEqual(bat["source_kind"], "ardupilot_bin")
        self.assertEqual(bat["source_time"]["value"], 1234567)
        self.assertIsNone(bat["receive_monotonic_ns"])
        self.assertIsNone(bat["message_id"])
        self.assertEqual(bat["fields"]["Curr"], -0.5)
        self.assertIsNone(msg["source_time"])
        reconstructed = b"".join(bytes.fromhex(item["raw_hex"]) for item in records
                                 if item["kind"] in {"message", "opaque"})
        self.assertEqual(reconstructed, data)
        self.assertEqual(records[0]["reason"], "run_start")
        self.assertEqual(records[-1]["reason"], "run_end")

    def test_causal_grid_never_uses_future_update(self):
        processor = Preprocessor("px4_gz", system_id=7, battery_id=255)
        processor.consume(record("BATTERY_STATUS", 0, battery(4000)))
        events = processor.consume(record("BATTERY_STATUS", 250000000, battery(3000)))
        rows = [item for item in events if item["kind"] == "row"]
        self.assertEqual([item["tick_ns"] for item in rows], [0, 100000000, 200000000])
        self.assertEqual([item["values"]["battery_voltage_v"] for item in rows], [16.0] * 3)
        processor.consume(record("HEARTBEAT", 300000000, {}))
        last = processor.finish()[-1]
        self.assertEqual(last["values"]["battery_voltage_v"], 12.0)

    def test_battery_ready_does_not_require_motion_and_window_is_bounded(self):
        processor = Preprocessor("px4_gz", system_id=7, battery_id=255)
        for index in range(70):
            processor.consume(record("BATTERY_STATUS", index * 100000000, battery()))
        processor.finish()
        window = processor.window("battery")
        self.assertEqual(len(window), 50)
        self.assertTrue(window[-1]["window_battery_ready"])
        self.assertFalse(window[-1]["motion_ready"])
        self.assertIsNone(processor.window("motion"))
        window[-1]["values"]["battery_voltage_v"] = 999
        self.assertEqual(processor.window()[-1]["values"]["battery_voltage_v"], 16.0)

    def test_motion_ready_without_battery_and_firmware_specific_names(self):
        processor = Preprocessor("px4_gz", system_id=7)
        for index in range(50):
            t = index * 100000000
            processor.consume(record("ACTUATOR_OUTPUT_STATUS", t,
                                     {"time_usec": index + 1, "active": 4, "actuator": [500] * 32}))
            processor.consume(record("ATTITUDE", t, {"roll": 0, "pitch": 0, "yaw": 0}))
            processor.consume(record("HIGHRES_IMU", t,
                                     {"id": 0, "time_usec": index + 1, "fields_updated": 63,
                                      "xacc": 0, "yacc": 0, "zacc": -9.8,
                                      "xgyro": 0, "ygyro": 0, "zgyro": 0}))
        processor.finish()
        row = processor.window("motion")[-1]
        self.assertFalse(row["battery_ready"])
        self.assertEqual(row["values"]["px4_actuator_1_command_rad_s"], 500)
        self.assertNotIn("ap_output_1_pwm_us", row["values"])
        self.assertNotIn("motor_rpm", row["values"])

    def test_stale_invalid_and_wrong_component_are_not_normal_zero(self):
        processor = Preprocessor("px4_gz", system_id=7, battery_id=255)
        processor.consume(record("BATTERY_STATUS", 0, battery()))
        processor.consume(record("BATTERY_STATUS", 100000000, battery(5000), component=180))
        processor.consume(record("HEARTBEAT", 700000000, {}))
        stale = processor.finish()[-1]
        self.assertIsNone(stale["values"]["battery_voltage_v"])
        self.assertFalse(stale["mask"]["battery_voltage_v"])
        self.assertEqual(stale["age_ns"]["battery_voltage_v"], 700000000)
        invalid = Preprocessor("px4_gz", system_id=7, battery_id=255)
        invalid.consume(record("BATTERY_STATUS", 0, battery(65535, -1, -1)))
        row = invalid.finish()[-1]
        self.assertFalse(row["battery_ready"])
        self.assertIsNone(row["values"]["battery_current_a"])
        self.assertIsNone(row["values"]["battery_soc_pct"])

    def test_duplicate_source_stamp_and_imu_bits_do_not_refresh_age(self):
        processor = Preprocessor("px4_gz", system_id=7)
        imu = {"id": 0, "time_usec": 1, "fields_updated": 63,
               "xacc": 1, "yacc": 2, "zacc": 3, "xgyro": 4, "ygyro": 5, "zgyro": 6}
        processor.consume(record("HIGHRES_IMU", 0, imu))
        processor.consume(record("HIGHRES_IMU", 100000000, {**imu, "time_usec": 2, "fields_updated": 1}))
        processor.consume(record("HIGHRES_IMU", 300000000, {**imu, "time_usec": 2}))
        row = processor.finish()[-1]
        self.assertEqual(row["age_ns"]["imu_accel_x_m_s2"], 200000000)
        self.assertEqual(row["age_ns"]["imu_gyro_x_rad_s"], 300000000)
        self.assertIsNone(row["values"]["imu_gyro_x_rad_s"])

    def test_gap_new_run_and_source_rewind_break_windows(self):
        processor = Preprocessor("px4_gz", system_id=7, battery_id=255)
        for index in range(50):
            processor.consume(record("BATTERY_STATUS", index * 100000000, battery()))
        events = processor.consume(record("BATTERY_STATUS", 7000000000, battery()))
        self.assertIn("clock_gap", [item.get("reason") for item in events])
        self.assertIsNone(processor.window())
        events = processor.consume(record("BATTERY_STATUS", 7100000000, battery(), run_id="next"))
        self.assertIn("run_change", [item.get("reason") for item in events])
        processor.consume(record("VIBRATION", 7200000000, {"time_usec": 100}))
        events = processor.consume(record("VIBRATION", 7300000000, {"time_usec": 1}))
        self.assertIn("source_clock_rewind", [item.get("reason") for item in events])
        self.assertIsNone(processor.window())

    def test_bin_instances_unknown_time_and_distinct_pwm_vibration(self):
        processor = Preprocessor("arducopter_4_4")
        untimed = record("BAT", None, {"Instance": 0, "Volt": 48}, source_kind="ardupilot_bin")
        self.assertFalse(any(item["kind"] == "row" for item in processor.consume(untimed)))
        for name, fields in (
            ("BAT", {"Instance": 0, "Volt": 48, "Curr": -0.5, "RemPct": 80}),
            ("IMU", {"I": 1, "AccX": 999, "GH": 1, "AH": 1}),
            ("RCOU", {"C1": 1500, "C2": 1501, "C3": 1502, "C4": 1503}),
            ("VIBE", {"IMU": 0, "VibeX": 1, "VibeY": 2, "VibeZ": 3}),
        ):
            processor.consume(record(name, None, {"TimeUS": 1000000, **fields}, source_kind="ardupilot_bin"))
        row = processor.finish()[-1]
        self.assertEqual(row["clock"], "source_boot_ns")
        self.assertEqual(row["tick_ns"], 1000000000)
        self.assertTrue(row["battery_ready"])
        self.assertEqual(row["values"]["battery_current_a"], -0.5)
        self.assertEqual(row["values"]["ap_output_1_pwm_us"], 1500)
        self.assertEqual(row["values"]["ap_vibe_x_m_s2"], 1)
        self.assertIsNone(row["values"]["imu_accel_x_m_s2"])
        self.assertNotIn("px4_coning_metric_rad2", row["values"])

    def test_offline_uses_same_consumer_and_does_not_duplicate_windows(self):
        source = self.path / "raw.jsonl"
        inputs = [record("BATTERY_STATUS", index * 100000000, battery()) for index in range(55)]
        source.write_text("".join(encode_json(item) + "\n" for item in inputs))
        expected = []
        live = Preprocessor("px4_gz", system_id=7, battery_id=255, on_event=expected.append)
        for item in inputs:
            live.consume(item)
        live.finish()
        output = self.path / "rows.jsonl"
        result = preprocess_file(source, output, firmware="px4_gz", system_id=7, battery_id=255)
        saved = [decode_json(line) for line in output.read_text().splitlines()]
        rows = [item for item in saved if item["kind"] == "row"]
        self.assertEqual(rows, [item for item in expected if item["kind"] == "row"])
        self.assertEqual(result["rows"], 55)
        self.assertTrue(all("window" not in item for item in rows))
        with self.assertRaises(FileExistsError):
            preprocess_file(source, output, firmware="px4_gz", system_id=7)


if __name__ == "__main__":
    unittest.main()
