"""역할: 펌웨어별 물리 의미를 지킨 인과 10 Hz 정렬과 독립 준비 마스크.
입력: collector/replay의 원문 envelope. 미래 보간·정답·학습 통계는 쓰지 않는다.
출력: null/mask/age를 가진 행, 경계 이벤트, 최근 50행 소비자 창.
호출자: live on_record와 오프라인 preprocess_file. 모델 학습/정규화는 범위 밖이다.
"""

from collections import deque
from copy import deepcopy
import math
import os
from pathlib import Path
import threading

from .collector import RAW_SCHEMA, encode_json
from .replay import RAW_FILE, iter_records


ROW_SCHEMA = "capstone.rows.v1"
STEP_NS = 100_000_000
GAP_NS = 1_000_000_000
WINDOW_ROWS = 50
BATTERY_REQUIRED = ("battery_voltage_v", "battery_current_a", "battery_soc_pct")
IMU_FEATURES = tuple(f"imu_{sensor}_{axis}_{unit}" for sensor, unit in
                     (("accel", "m_s2"), ("gyro", "rad_s")) for axis in "xyz")


def _number(value, *, minimum=None, maximum=None, invalid=()):
    """유한한 실제 숫자만 채택한다. 결측/센티널은 0이 아니라 None이다."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value) or value in invalid:
        return None
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        return None
    return float(value)


def _scale(value, factor):
    """이미 유효성 확인한 값의 단위만 변환한다."""
    return None if value is None else value * factor


def feature_schema(firmware):
    """펌웨어별 이름·단위·의미·TTL·필수 그룹을 반환한다. 공유 scaler는 정의하지 않는다."""
    if firmware not in {"px4_gz", "arducopter_4_4"}:
        raise ValueError("firmware는 px4_gz 또는 arducopter_4_4로 명시할 것")
    result = {}

    def add(name, unit, source, age_ms, required=None):
        result[name] = {"unit": unit, "source": source, "max_age_ns": age_ms * 1_000_000,
                        "required_for": required}

    battery_source = "BATTERY_STATUS" if firmware == "px4_gz" else "BAT Instance0"
    for name, unit in (("battery_voltage_v", "V"), ("battery_current_a", "A"),
                       ("battery_soc_pct", "%"), ("battery_consumed_mah", "mAh")):
        add(name, unit, battery_source, 600, "battery" if name in BATTERY_REQUIRED else None)
    for axis in ("roll", "pitch", "yaw"):
        add(f"attitude_{axis}_rad", "rad", "ATTITUDE" if firmware == "px4_gz" else "ATT",
            300, "motion" if axis != "yaw" else None)
    for name in IMU_FEATURES:
        unit = "m/s²" if "accel" in name else "rad/s"
        add(name, unit, "HIGHRES_IMU 처리·bias 보정 body" if firmware == "px4_gz"
            else "IMU I0 body; AH/GH 건강 필드 확인", 200, "motion")
    if firmware == "px4_gz":
        for index in range(1, 5):
            add(f"px4_actuator_{index}_command_rad_s", "rad/s", "GZ 구동 명령, 측정 RPM 아님", 250, "motion")
        for axis in "xyz":
            add(f"velocity_{axis}_ned_m_s", "m/s", "LOCAL_POSITION_NED 융합 추정, 아래쪽 양수", 300)
        for name, unit in (("px4_coning_metric_rad2", "rad²"), ("px4_gyro_vibration_rad_s", "rad/s"),
                           ("px4_accel_vibration_m_s2", "m/s²")):
            add(name, unit, "pinned PX4 VIBRATION 지표, XYZ 진동 아님", 300)
    else:
        for index in range(1, 5):
            add(f"ap_output_{index}_pwm_us", "µs", f"RCOU.C{index} PWM 명령, 측정 RPM 아님", 250, "motion")
        for axis in "xyz":
            add(f"ap_vibe_{axis}_m_s2", "m/s²", "ArduPilot VIBE IMU0 축별 진동", 300)
    for name, unit in (("gps_altitude_msl_m", "m"), ("gps_speed_m_s", "m/s"),
                       ("gps_hdop", "1"), ("gps_satellites", "개")):
        add(name, unit, "GPS_RAW_INT" if firmware == "px4_gz" else "GPS I0; DFReader 단위 변환 적용", 600)
    return result


class Preprocessor:
    """한 번에 한 원문 run을 소비한다. 콜백은 빠르게 반환해야 하며 예외를 숨기지 않는다."""

    def __init__(self, firmware, *, system_id=None, component_id=1, battery_id=None,
                 battery_cells=4, imu_instance=0, gps_instance=0, on_event=None):
        self.firmware = firmware
        self.features = feature_schema(firmware)
        for name, value, upper in (("system_id", system_id, 255), ("component_id", component_id, 255),
                                   ("battery_id", battery_id, 255), ("imu_instance", imu_instance, 255),
                                   ("gps_instance", gps_instance, 255)):
            if value is None and name in {"system_id", "battery_id"}:
                continue
            minimum = 1 if name in {"system_id", "component_id"} else 0
            if type(value) is not int or not minimum <= value <= upper:
                raise ValueError(f"{name}가 유효한 정수 식별자가 아님")
        if type(battery_cells) is not int or not 1 <= battery_cells <= 14:
            raise ValueError("battery_cells는 1~14 정수여야 함")
        if firmware == "arducopter_4_4" and any(value not in (None, 0)
                for value in (battery_id, imu_instance, gps_instance)):
            raise ValueError("현재 ArduCopter 계약은 BAT/IMU/GPS instance0만 지원함")
        self.system_id = system_id
        self.component_id = component_id
        self.battery_id = battery_id if firmware == "px4_gz" else 0
        self.battery_cells = battery_cells
        self.imu_instance = imu_instance
        self.gps_instance = gps_instance
        self.on_event = on_event
        self.clock = "receive_monotonic_ns" if firmware == "px4_gz" else "source_boot_ns"
        self._required = {group: [name for name, spec in self.features.items() if spec["required_for"] == group]
                          for group in ("battery", "motion")}
        self._lock = threading.RLock()
        self._window = deque(maxlen=WINDOW_ROWS)
        self._key = None
        self._epoch = -1
        self._system = self.system_id
        self._battery = self.battery_id
        self._closed = False
        self._stats = {"rows": 0, "epochs": 0, "untimed_messages": 0, "ignored_messages": 0}
        self._clear()

    def _clear(self):
        """경계 이전의 값·원천 시각·창을 함께 비운다."""
        self._values = {}
        self._stamps = {}
        self._last_time = None
        self._next_tick = None
        self._last_consumed = None
        self._window.clear()

    def _emit(self, event, events):
        """소비자의 수정이 내부 창이나 다른 소비자에 전파되지 않게 한다."""
        events.append(deepcopy(event))
        if self.on_event is not None:
            self.on_event(deepcopy(event))

    def _boundary(self, reason):
        """행과 동일한 run/epoch를 가진 전처리 경계 메타데이터다."""
        return {"schema": ROW_SCHEMA, "kind": "boundary", "reason": reason,
                "run_id": self._key[0] if self._key else None,
                "source_epoch": self._key[1] if self._key else None,
                "source_kind": self._key[2] if self._key else None,
                "epoch": self._epoch, "firmware": self.firmware, "clock": self.clock}

    def _begin(self, key, reason, events, *, new_identity=False):
        """새 epoch를 시작하며 공백/역행 전후 창이 연결되지 않게 한다."""
        self._clear()
        self._key = key
        self._epoch += 1
        self._stats["epochs"] += 1
        if new_identity:
            self._system, self._battery = self.system_id, self.battery_id
        self._emit(self._boundary(reason), events)

    def _selected(self, record):
        """기체·component·sensor instance 선택은 freshness 계산보다 먼저 적용한다."""
        fields, name = record["fields"], record["message_name"]
        if self.firmware == "px4_gz":
            if record.get("source_component") != self.component_id:
                return False
            if self._system is None:
                if (name != "HEARTBEAT" or fields.get("autopilot") in (None, 8)
                        or type(record.get("source_system")) is not int or record["source_system"] <= 0):
                    return False
                self._system = record["source_system"]
            if record.get("source_system") != self._system:
                return False
            if name == "BATTERY_STATUS":
                if type(fields.get("id")) is not int or not 0 <= fields["id"] <= 255:
                    return False
                if self._battery is None:
                    self._battery = fields["id"]
                return fields["id"] == self._battery
            if name == "HIGHRES_IMU":
                return fields.get("id") == self.imu_instance
        else:
            instance_field = {"BAT": "Instance", "IMU": "I", "GPS": "I", "VIBE": "IMU"}.get(name)
            if instance_field is not None:
                return fields.get(instance_field) == 0
        return True

    def _time(self, record):
        """UDP는 수신 단조 시계, BIN은 그 레코드의 실제 TimeUS만 사용한다."""
        if self.firmware == "px4_gz":
            value = record.get("receive_monotonic_ns")
            return value if type(value) is int and value >= 0 else None
        source = record.get("source_time") or {}
        if (source.get("field"), source.get("unit"), source.get("clock")) != ("TimeUS", "us", "boot"):
            return None
        value = source.get("value")
        return value * 1000 if type(value) is int and value >= 0 else None

    def consume(self, record):
        """외피 하나를 받아 확정된 행/경계를 반환한다. 같은 tick 시각의 입력은 모두 먼저 반영한다."""
        with self._lock:
            if self._closed:
                raise RuntimeError("finish 후에는 reset 또는 새 Preprocessor가 필요함")
            if record.get("schema") != RAW_SCHEMA:
                raise ValueError("지원하지 않는 원문 스키마")
            events = []
            kind = record.get("kind")
            if kind not in {"message", "boundary"} or record.get("scope") == "file":
                return events
            expected = "mavlink_udp" if self.firmware == "px4_gz" else "ardupilot_bin"
            if record.get("source_kind") != expected:
                raise ValueError(f"{self.firmware}는 {expected} 원문만 소비함")
            key = (record["run_id"], record["epoch"], record["source_kind"])
            if kind == "boundary":
                if self._last_time is not None:
                    self._advance(self._last_time, events, inclusive=True)
                if record.get("reason") == "run_end":
                    if self._key is not None:
                        self._emit(self._boundary("run_end"), events)
                    self._key = None
                    self._last_time = self._next_tick = None
                else:
                    self._begin(key, record.get("reason", "explicit_boundary"), events, new_identity=True)
                return events
            if key != self._key:
                if self._last_time is not None:
                    self._advance(self._last_time, events, inclusive=True)
                self._begin(key, "run_change", events, new_identity=True)
            if not self._selected(record):
                self._stats["ignored_messages"] += 1
                return events
            t = self._time(record)
            if t is None:
                self._stats["untimed_messages"] += 1
                return events
            updates = self._px4(record) if self.firmware == "px4_gz" else self._ardupilot(record)
            name = record["message_name"]
            source = record.get("source_time") or {}
            stamp_value = _number(source.get("value"), minimum=0)
            stamp = (source.get("field"), source.get("unit"), source.get("clock"), source.get("value"))
            previous = self._stamps.get(name) if updates else None
            reason = None
            if self._last_time is not None:
                if t < self._last_time:
                    reason = "clock_rewind"
                elif t - self._last_time > GAP_NS:
                    reason = "clock_gap"
            if previous and stamp_value is not None and stamp[:3] == previous[:3] and stamp[3] < previous[3]:
                reason = reason or "source_clock_rewind"
            consumed = updates.get("battery_consumed_mah")
            if consumed is not None and self._last_consumed is not None and consumed < self._last_consumed:
                reason = reason or "consumption_rewind"
            if reason:
                self._advance(self._last_time, events, inclusive=True)
                self._begin(key, reason, events)
                previous = None
            if self._next_tick is None:
                self._next_tick = t
            self._advance(t, events)
            # 같은 원천 시각의 재송신은 필드별 나이를 새로 만들지 않는다.
            duplicate = previous is not None and stamp_value is not None and stamp == previous
            if not duplicate:
                for feature, value in updates.items():
                    self._values[feature] = (value, t)
                if updates and stamp_value is not None:
                    self._stamps[name] = stamp
                if consumed is not None:
                    self._last_consumed = consumed
            self._last_time = t
            return events

    def _advance(self, t, events, *, inclusive=False):
        """현재 입력보다 앞선 tick만 확정한다. EOF/경계에서만 마지막 같은 tick도 확정한다."""
        if t is None:
            return
        while self._next_tick is not None and (self._next_tick < t or inclusive and self._next_tick == t):
            tick = self._next_tick
            values, masks, ages = {}, {}, {}
            for name, spec in self.features.items():
                value, updated = self._values.get(name, (None, None))
                age = None if updated is None else tick - updated
                valid = value is not None and age is not None and 0 <= age <= spec["max_age_ns"]
                values[name], masks[name], ages[name] = (value if valid else None), valid, age
            row = {"schema": ROW_SCHEMA, "kind": "row", "firmware": self.firmware,
                   "run_id": self._key[0], "source_epoch": self._key[1], "source_kind": self._key[2],
                   "epoch": self._epoch, "clock": self.clock, "tick_ns": tick,
                   "values": values, "mask": masks, "age_ns": ages}
            for group, required in self._required.items():
                row[f"{group}_ready"] = all(masks[name] for name in required)
            self._window.append(row)
            for group in self._required:
                row[f"window_{group}_ready"] = (len(self._window) == WINDOW_ROWS
                    and all(item[f"{group}_ready"] for item in self._window))
            self._stats["rows"] += 1
            self._next_tick += STEP_NS
            self._emit(row, events)

    def finish(self):
        """닫힌 입력의 마지막 관측 tick까지만 확정한다. live 침묵에서 호출하지 않는다."""
        with self._lock:
            events = []
            if not self._closed:
                self._advance(self._last_time, events, inclusive=True)
                self._closed = True
            return events

    def reset(self, reason="manual_reset"):
        """호출자가 아는 reset/profile 변경은 명시적 경계로 끊는다."""
        with self._lock:
            events = []
            self._advance(self._last_time, events, inclusive=True)
            self._emit(self._boundary(reason), events)
            self._clear()
            self._key = None
            self._system, self._battery = self.system_id, self.battery_id
            self._closed = False
            return events

    def window(self, group=None):
        """최근 50행 복사본을 준다. 그룹 지정 시 그 그룹이 전부 준비된 창만 준다."""
        if group not in {None, "battery", "motion"}:
            raise ValueError("group은 None/battery/motion이어야 함")
        with self._lock:
            if len(self._window) != WINDOW_ROWS:
                return None
            if group and not self._window[-1][f"window_{group}_ready"]:
                return None
            return deepcopy(list(self._window))

    def snapshot(self):
        """미관측/제외와 출력량을 구별하는 작은 감독용 상태다."""
        with self._lock:
            return {**self._stats, "firmware": self.firmware, "clock": self.clock,
                    "system_id": self._system, "battery_id": self._battery,
                    "window_rows": len(self._window), "last_time_ns": self._last_time,
                    "epoch": self._epoch, "closed": self._closed}

    def _px4(self, record):
        """pinned PX4+GZ 명령/센서 계약만 변환한다. 다른 firmware에 이 매핑을 쓰지 않는다."""
        name, f = record["message_name"], record["fields"]
        if name == "BATTERY_STATUS":
            cells = list(f.get("voltages", [])) + list(f.get("voltages_ext", []))
            voltage = None
            if len(cells) >= self.battery_cells:
                chosen = [_number(value, minimum=0, maximum=65534, invalid=(0,) if index >= 10 else ())
                          for index, value in enumerate(cells[:self.battery_cells])]
                if all(value is not None for value in chosen):
                    voltage = sum(chosen) / 1000
            return {"battery_voltage_v": voltage,
                    "battery_current_a": _scale(_number(f.get("current_battery"), invalid=(-1,)), 0.01),
                    "battery_soc_pct": _number(f.get("battery_remaining"), minimum=0, maximum=100),
                    "battery_consumed_mah": _number(f.get("current_consumed"), minimum=0)}
        if name == "ATTITUDE":
            return {f"attitude_{axis}_rad": _number(f.get(axis)) for axis in ("roll", "pitch", "yaw")}
        if name == "HIGHRES_IMU":
            bits = f.get("fields_updated")
            if type(bits) is not int or not 0 <= bits <= 65535:
                return dict.fromkeys(IMU_FEATURES)
            return {feature: _number(f.get(field)) for bit, (feature, field) in enumerate(zip(
                IMU_FEATURES, ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro"))) if bits & (1 << bit)}
        if name == "ACTUATOR_OUTPUT_STATUS":
            outputs, count = f.get("actuator", []), f.get("active")
            valid = type(count) is int and 4 <= count <= len(outputs)
            return {f"px4_actuator_{index + 1}_command_rad_s":
                    _number(outputs[index], minimum=0) if valid else None for index in range(4)}
        if name == "LOCAL_POSITION_NED":
            return {f"velocity_{axis}_ned_m_s": _number(f.get(f"v{axis}")) for axis in "xyz"}
        if name == "VIBRATION":
            return {feature: _number(f.get(field), minimum=0) for feature, field in (
                ("px4_coning_metric_rad2", "vibration_x"), ("px4_gyro_vibration_rad_s", "vibration_y"),
                ("px4_accel_vibration_m_s2", "vibration_z"))}
        if name == "GPS_RAW_INT":
            fix = _number(f.get("fix_type"), minimum=0)
            clock_valid = _number(f.get("time_usec"), minimum=1) is not None
            fix2 = clock_valid and fix is not None and fix >= 2
            fix3 = fix2 and fix >= 3
            return {"gps_altitude_msl_m": _scale(_number(f.get("alt")), 0.001) if fix3 else None,
                    "gps_speed_m_s": _scale(_number(f.get("vel"), minimum=0, invalid=(65535,)), 0.01) if fix2 else None,
                    "gps_hdop": _scale(_number(f.get("eph"), minimum=0, invalid=(65535,)), 0.01) if fix2 else None,
                    "gps_satellites": _number(f.get("satellites_visible"), minimum=0, maximum=254) if clock_valid else None}
        return {}

    def _ardupilot(self, record):
        """ArduCopter 4.4 DataFlash 값은 DFReader의 기본 배율을 이미 적용한 단위다."""
        name, f = record["message_name"], record["fields"]
        if name == "BAT":
            return {"battery_voltage_v": _number(f.get("Volt"), minimum=0),
                    "battery_current_a": _number(f.get("Curr")),
                    "battery_soc_pct": _number(f.get("RemPct"), minimum=0, maximum=100),
                    "battery_consumed_mah": _number(f.get("CurrTot"), minimum=0)}
        if name == "ATT":
            return {f"attitude_{axis.lower()}_rad": _scale(_number(f.get(axis)), math.pi / 180)
                    for axis in ("Roll", "Pitch", "Yaw")}
        if name == "IMU":
            return {feature: _number(f.get(field)) if f.get("AH" if bit < 3 else "GH") == 1 else None
                    for bit, (feature, field) in enumerate(zip(
                        IMU_FEATURES, ("AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ")))}
        if name == "RCOU":
            return {f"ap_output_{index}_pwm_us": _number(f.get(f"C{index}"), minimum=0) for index in range(1, 5)}
        if name == "VIBE":
            return {f"ap_vibe_{axis.lower()}_m_s2": _number(f.get(f"Vibe{axis}"), minimum=0) for axis in "XYZ"}
        if name == "GPS":
            fix = _number(f.get("Status"), minimum=0)
            fix2 = fix is not None and fix >= 2
            fix3 = fix2 and fix >= 3
            return {"gps_altitude_msl_m": _number(f.get("Alt")) if fix3 else None,
                    "gps_speed_m_s": _number(f.get("Spd"), minimum=0) if fix2 else None,
                    "gps_hdop": _number(f.get("HDop"), minimum=0, invalid=(655.35,)) if fix2 else None,
                    "gps_satellites": _number(f.get("NSats"), minimum=0, maximum=254)}
        return {}


def preprocess_file(path, output_path, *, firmware, source_kind="auto", decoder_path=None, **options):
    """같은 consume 함수로 닫힌 원문을 새 행 JSONL에 저장한다. 중간 실패 파일은 남겨 진단한다."""
    source, destination = Path(path).resolve(), Path(output_path).resolve()
    if source == destination or (source.is_dir() and destination.parent == source
                                 and RAW_FILE.fullmatch(destination.name)):
        raise ValueError("출력은 입력 원문/회전 파일과 분리할 것")
    if "on_event" in options:
        raise ValueError("preprocess_file은 파일 출력을 소유하므로 on_event를 받지 않음")
    processor = Preprocessor(firmware, **options)
    destination.parent.mkdir(parents=True, exist_ok=True)
    counts = {"records": 0, "rows": 0, "boundaries": 0, "output_path": str(destination)}
    records = iter_records(source, source_kind=source_kind, decoder_path=decoder_path)
    try:
        with destination.open("x", encoding="utf-8") as output:
            metadata = {"schema": ROW_SCHEMA, "kind": "metadata", "firmware": firmware,
                        "features": processor.features, "step_ns": STEP_NS, "window_rows": WINDOW_ROWS,
                        "gap_ns": GAP_NS, "options": options, "clock": processor.clock}
            output.write(encode_json(metadata) + "\n")
            for record in records:
                counts["records"] += 1
                for event in processor.consume(record):
                    output.write(encode_json(event) + "\n")
                    counts["rows" if event["kind"] == "row" else "boundaries"] += 1
            for event in processor.finish():
                output.write(encode_json(event) + "\n")
                counts["rows"] += 1
            output.flush()
            os.fsync(output.fileno())
    finally:
        records.close()
    return {**counts, "processor": processor.snapshot()}
