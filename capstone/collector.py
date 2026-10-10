"""역할: 송신 없는 독점 UDP 수집과 원문·실패 기록.
입력: 명시한 로컬 bind 주소, MAVLink 데이터그램, 선택적 생성 dialect 파일.
출력: mavlink_raw.jsonl, summary.json, 동일 envelope의 on_record 콜백.
호출자: 실행 감독자. 제어/배터리 송신기는 이 모듈 밖에 둔다.
"""

from array import array
from collections import Counter
from copy import deepcopy
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
import uuid


RAW_SCHEMA = "capstone.raw.v1"


def _tag(value):
    """JSON에서 잃는 비유한 실수와 바이트만 명시적 태그로 바꾼다."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"$float": "nan" if math.isnan(value) else ("inf" if value > 0 else "-inf")}
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {"$bytes": bytes(value).hex()}
    if isinstance(value, dict):
        return {key: _tag(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, array)):
        return [_tag(item) for item in value]
    return value


def encode_json(value):
    """저장용 엄격 JSON 한 행을 만든다. 알 수 없는 객체를 문자열로 숨기지 않는다."""
    return json.dumps(_tag(value), ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def decode_json(text):
    """예약 태그를 복구하고 비표준 NaN/Infinity JSON은 거부한다."""
    def restore(value):
        if set(value) == {"$float"}:
            if value["$float"] not in {"nan", "inf", "-inf"}:
                raise ValueError("알 수 없는 $float 태그")
            return float(value["$float"])
        if set(value) == {"$bytes"}:
            return bytes.fromhex(value["$bytes"])
        return value

    def reject(value):
        raise ValueError(f"비표준 JSON 상수: {value}")

    return json.loads(text, object_hook=restore, parse_constant=reject)


def load_dialect(decoder_path=None):
    """별도 dialect 모듈을 읽는다. mavutil 전역 dialect나 환경변수를 바꾸지 않는다."""
    if decoder_path is None:
        from pymavlink.dialects.v20 import common
        return common
    path = Path(decoder_path).resolve(strict=True)
    spec = importlib.util.spec_from_file_location(f"_capstone_decoder_{uuid.uuid4().hex}", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Python MAVLink dialect를 읽을 수 없음: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "MAVLink", None)):
        raise ValueError(f"MAVLink 클래스가 없는 dialect: {path}")
    return module


def source_time(fields):
    """원천 시간의 값·단위·시계만 보존한다. time_usec를 UTC로 추측하지 않는다."""
    for field, unit, clock in (("time_boot_ms", "ms", "boot"),
                               ("time_usec", "us", "boot_or_unix"),
                               ("time_unix_usec", "us", "unix")):
        if field in fields:
            return {"field": field, "value": fields[field], "unit": unit, "clock": clock}
    return None


def envelope(source_kind, run_id, kind, **values):
    """UDP와 BIN이 공유하는 외피다. 해당하지 않는 식별자·시각은 null이다."""
    return {
        "schema": RAW_SCHEMA, "kind": kind, "source_kind": source_kind,
        "run_id": run_id, "epoch": 0, "record_index": None,
        "message_name": None, "message_id": None, "source_system": None,
        "source_component": None, "packet_sequence": None, "peer": None,
        "receive_wall_ns": None, "receive_monotonic_ns": None,
        "source_time": None, "fields": {}, "raw_hex": "", **values,
    }


def _frame_identity(raw):
    """UNKNOWN 메시지도 원래 wire 헤더의 ID/seq를 보존한다. CRC 검증을 뜻하지 않는다."""
    if raw[:1] == b"\xfd" and len(raw) >= 10:
        return int.from_bytes(raw[7:10], "little"), raw[5], raw[6], raw[4]
    if raw[:1] == b"\xfe" and len(raw) >= 6:
        return raw[5], raw[3], raw[4], raw[2]
    return None, None, None, None


class Collector:
    """한 run의 UDP 수신·파일·스레드를 소유한다. 재시작은 새 인스턴스로 한다."""

    def __init__(self, host, port, output_dir, *, decoder_path=None,
                 system_id=None, component_id=1, on_record=None):
        address = ipaddress.ip_address(host)
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise ValueError("port는 0~65535 정수여야 함")
        for name, value in (("system_id", system_id), ("component_id", component_id)):
            if value is None and name == "system_id":
                continue
            if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255:
                raise ValueError(f"{name}는 1~255 정수여야 함")
        self.host = str(address)
        self.port = port
        self.output_dir = Path(output_dir)
        self.decoder_path = decoder_path
        self.system_id = system_id
        self.component_id = component_id
        self.on_record = on_record
        self.run_id = uuid.uuid4().hex
        self._family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self._socket = None
        self._file = None
        self._started = False
        self._running = False
        self._owns_output = False
        self._failure = None
        self._errors = []
        self._counts = Counter(datagrams=0, messages=0, saved_records=0, decode_diagnostics=0)
        self._seen = Counter()
        self._latest = {}
        self._dialect = None
        self._decoder_info = None

    @property
    def listen_port(self):
        """port=0 요청 후 실제 bind 포트를 돌려준다. port 속성과 같은 값이다."""
        return self.port

    def start(self):
        """독점 bind 후 수신 스레드를 시작한다. 송신 소켓/자동 응답은 만들지 않는다."""
        if self._started:
            raise RuntimeError("Collector는 한 번만 시작할 수 있음")
        self._started = True
        try:
            self._dialect = load_dialect(self.decoder_path)
            decoder_file = Path(self._dialect.__file__)
            self._decoder_info = {"path": str(decoder_file),
                                  "sha256": hashlib.sha256(decoder_file.read_bytes()).hexdigest()}
            self.output_dir.mkdir(parents=True, exist_ok=True)
            if (self.output_dir / "summary.json").exists():
                raise FileExistsError("기존 summary.json이 있는 run에는 기록하지 않음")
            self._file = (self.output_dir / "mavlink_raw.jsonl").open("x", encoding="utf-8")
            self._owns_output = True
            self._socket = socket.socket(self._family, socket.SOCK_DGRAM)
            # SO_REUSEADDR/SO_REUSEPORT를 켜지 않는다. 다른 소비자는 콜백을 쓴다.
            self._socket.bind((self.host, self.port))
            self.port = self._socket.getsockname()[1]
            self._socket.settimeout(0.2)
            self._running = True
            self._thread = threading.Thread(target=self._run, name="capstone-collector", daemon=True)
            self._thread.start()
        except Exception as exc:
            self._fail(exc)
            self._close()
            raise
        return self

    def snapshot(self):
        """감독자가 읽는 누적 개수/식별/지속 오류. heartbeat 유무를 건강도로 위장하지 않는다."""
        with self._lock:
            return {"run_id": self.run_id, "host": self.host, "port": self.port,
                    "output_dir": str(self.output_dir), "system_id": self.system_id,
                    "component_id": self.component_id, "running": self._running,
                    "error": self._errors[0] if self._errors else None,
                    "errors": list(self._errors), "seen_by_type": dict(self._seen), **self._counts}

    def latest(self, message_name, max_age_s=3, *, since_monotonic_ns=None):
        """신선한 fields 사본이다. 선택 시 PC 단조 ns 경계보다 뒤에 수신한 표본만 반환한다."""
        if not isinstance(max_age_s, (int, float)) or not math.isfinite(max_age_s) or max_age_s < 0:
            raise ValueError("max_age_s는 유한한 비음수 초여야 함")
        if since_monotonic_ns is not None and (type(since_monotonic_ns) is not int or since_monotonic_ns < 0):
            raise ValueError("since_monotonic_ns는 비음수 정수 ns 또는 None이어야 함")
        with self._lock:
            item = self._latest.get(message_name)
            if not self._running or self._failure is not None or item is None:
                return {}
            if since_monotonic_ns is not None and item[0] <= since_monotonic_ns:
                return {}
            age_ns = time.monotonic_ns() - item[0]
            if not 0 <= age_ns <= max_age_s * 1e9:
                return {}
            return deepcopy(item[1])

    def stop(self):
        """소유 스레드만 종료한다. 저장·콜백·종료 실패는 반복 호출에도 계속 보고한다."""
        self._stop.set()
        if self._thread is threading.current_thread():
            raise RuntimeError("on_record 안에서 stop을 호출하지 말고 감독자에게 알릴 것")
        if self._thread is not None:
            self._thread.join(timeout=3)
            if self._thread.is_alive():
                exc = TimeoutError("수집 종료 제한 초과: on_record는 차단 없이 반환해야 함")
                self._fail(exc)
                raise exc
        if self._failure is not None:
            raise RuntimeError(f"수집 실패: {self.snapshot()['error']}") from self._failure
        return self.snapshot()

    def close(self):
        """자원 종료 관례를 쓰는 호출자를 위한 stop의 동등한 이름이다."""
        return self.stop()

    def _fail(self, exc):
        """첫 예외와 후속 자원 종료 오류를 덮어쓰지 않고 누적한다."""
        with self._lock:
            if self._failure is None:
                self._failure = exc
            self._errors.append(f"{type(exc).__name__}: {exc}")
        self._stop.set()

    def _write(self, item, *, notify=True):
        """원문을 먼저 flush하고 소비자에게 복사본을 준다. 콜백 오류도 수집 실패다."""
        item["record_index"] = self._counts["saved_records"]
        self._file.write(encode_json(item) + "\n")
        self._file.flush()
        with self._lock:
            self._counts["saved_records"] += 1
        if notify and self.on_record is not None:
            self.on_record(deepcopy(item))

    def _boundary(self, reason):
        """부팅 시각이 아닌 PC 시각으로 run 경계를 기록한다."""
        return envelope("mavlink_udp", self.run_id, "boundary", reason=reason, scope="run",
                        receive_wall_ns=time.time_ns(), receive_monotonic_ns=time.monotonic_ns(),
                        decoder=self._decoder_info, error=self.snapshot()["error"])

    def _run(self):
        """한 데이터그램을 저장한 뒤 그 안의 모든 디코딩 결과를 순서대로 저장한다."""
        try:
            self._write(self._boundary("run_start"))
            while not self._stop.is_set():
                try:
                    data, peer = self._socket.recvfrom(65535)
                except socket.timeout:
                    continue
                wall_ns, mono_ns = time.time_ns(), time.monotonic_ns()
                with self._lock:
                    number = self._counts["datagrams"]
                    self._counts["datagrams"] += 1
                meta = {"datagram_index": number, "peer": list(peer),
                        "receive_wall_ns": wall_ns, "receive_monotonic_ns": mono_ns}
                self._write(envelope("mavlink_udp", self.run_id, "datagram", raw_hex=data.hex(), **meta))
                self._decode(data, meta)
        except Exception as exc:
            self._fail(exc)
        finally:
            try:
                self._write(self._boundary("run_end"), notify=self._failure is None)
            except Exception as exc:
                self._fail(exc)
            self._close()

    def _decode(self, data, meta):
        """프레임을 데이터그램 사이에서 합치지 않는다. 분할/손상 바이트도 원문에 남는다."""
        decoder = self._dialect.MAVLink(None)
        decoder.robust_parsing = True
        # 예상치 못한 디코더 예외는 숨기지 않는다. 원 데이터그램은 이미 저장했다.
        messages = decoder.parse_buffer(data) or []
        for message in messages:
            name = message.get_type()
            fields = message.to_dict()
            fields.pop("mavpackettype", None)
            bad = name == "BAD_DATA"
            raw = bytes(message.get_msgbuf())
            message_id, system, component, sequence = (None, None, None, None) if bad else _frame_identity(raw)
            item = envelope("mavlink_udp", self.run_id, "message", **meta,
                            message_name=name, fields=fields, raw_hex=raw.hex(),
                            message_id=message_id, source_system=system, source_component=component,
                            packet_sequence=sequence, source_time=source_time(fields),
                            decode_status="bad_data" if bad else ("unknown" if name.startswith("UNKNOWN_") else "decoded"))
            with self._lock:
                self._counts["messages"] += 1
                self._seen[name] += 1
                if bad:
                    self._counts["decode_diagnostics"] += 1
            self._write(item)
            if not bad and not name.startswith("UNKNOWN_"):
                self._update_latest(item)
        remaining = decoder.buf_len()
        if remaining or not data:
            with self._lock:
                self._counts["decode_diagnostics"] += 1
            self._write(envelope("mavlink_udp", self.run_id, "decode_error", **meta,
                                 fields={"reason": "incomplete_or_empty_datagram", "remaining_bytes": remaining}))

    def _update_latest(self, item):
        """자동 식별은 autopilot HEARTBEAT에서만 고정한다. component180을 섞지 않는다."""
        if item["source_component"] != self.component_id:
            return
        with self._lock:
            if self.system_id is None:
                fields = item["fields"]
                if (item["message_name"] != "HEARTBEAT" or fields.get("autopilot") in (None, 8)
                        or not 1 <= item["source_system"] <= 255):
                    return
                self.system_id = item["source_system"]
            if item["source_system"] == self.system_id:
                self._latest[item["message_name"]] = (item["receive_monotonic_ns"], deepcopy(item["fields"]))

    def _close(self):
        """소유 자원만 닫고 summary를 새로 쓴다. 디스크 고장 시 메모리 오류는 남는다."""
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception as exc:
                self._fail(exc)
            self._socket = None
        if self._file is not None:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
            except Exception as exc:
                self._fail(exc)
            try:
                self._file.close()
            except Exception as exc:
                self._fail(exc)
            self._file = None
        with self._lock:
            self._running = False
        if self._owns_output:
            try:
                with (self.output_dir / "summary.json").open("x", encoding="utf-8") as output:
                    output.write(encode_json({"schema": RAW_SCHEMA, **self.snapshot()}) + "\n")
                    output.flush()
                    os.fsync(output.fileno())
            except Exception as exc:
                self._fail(exc)
