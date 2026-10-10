"""역할: 저장 JSONL과 ArduPilot DataFlash BIN을 같은 외피로 순서대로 공급.
입력: 원문 파일 또는 회전 JSONL/독립 BIN 파일 폴더. 원본은 읽기 전용이다.
출력: collector와 같은 envelope 반복자. BIN을 MAVLink라고 부르지 않는다.
호출자: 오프라인 전처리·분석. 실시간 속도 재현이나 기체 연결은 하지 않는다.
"""

from pathlib import Path
import re
import uuid

from .collector import RAW_SCHEMA, decode_json, envelope


RAW_FILE = re.compile(r"mavlink_raw(?:\.(\d+))?\.jsonl\Z")
MAX_LINE_BYTES = 4 * 1024 * 1024


def _order(path):
    """파일 이름의 숫자를 숫자로 정렬한다. 레코드 시각으로 재정렬하지 않는다."""
    return tuple((1, int(part)) if part.isdigit() else (0, part.casefold())
                 for part in re.split(r"(\d+)", path.name))


def _files(path, source_kind):
    """모호한 혼합 폴더와 잘못된 종류는 명시적으로 거부한다."""
    aliases = {"auto": "auto", "jsonl": "raw_jsonl", "raw_jsonl": "raw_jsonl",
               "bin": "ardupilot_bin", "ardupilot_bin": "ardupilot_bin"}
    if source_kind not in aliases:
        raise ValueError("source_kind는 auto/raw_jsonl/ardupilot_bin이어야 함")
    kind = aliases[source_kind]
    path = Path(path).resolve(strict=True)
    if path.is_dir():
        children = [child for child in path.iterdir() if child.is_file()]
        raw = sorted((child for child in children if RAW_FILE.fullmatch(child.name)),
                     key=lambda child: int(RAW_FILE.fullmatch(child.name).group(1) or -1))
        bins = sorted((child for child in children if child.suffix.lower() == ".bin"), key=_order)
        if kind == "auto":
            if raw and bins:
                raise ValueError("JSONL과 BIN이 함께 있음: source_kind를 명시할 것")
            kind = "raw_jsonl" if raw else "ardupilot_bin"
        files = raw if kind == "raw_jsonl" else bins
    else:
        if kind == "auto":
            kind = {".jsonl": "raw_jsonl", ".bin": "ardupilot_bin"}.get(path.suffix.lower())
            if kind is None:
                raise ValueError(f"확인할 수 없는 원문 확장자: {path.name}")
        files = [path]
    if not files:
        raise ValueError(f"지원하는 원문 파일이 없음: {path}")
    return files, kind


def _validate(item):
    """V1/행 JSONL 등을 원문으로 잘못 소비하지 않도록 최소 외피를 확인한다."""
    if not isinstance(item, dict) or item.get("schema") != RAW_SCHEMA:
        raise ValueError(f"{RAW_SCHEMA} 원문이 아님; 이전 스키마를 자동 변환하지 않음")
    if item.get("kind") not in {"boundary", "datagram", "message", "decode_error", "opaque"}:
        raise ValueError("알 수 없는 원문 kind")
    if item.get("source_kind") not in {"mavlink_udp", "ardupilot_bin", "raw_jsonl"}:
        raise ValueError("알 수 없는 source_kind")
    if not isinstance(item.get("run_id"), str) or not item["run_id"]:
        raise ValueError("run_id가 없음")
    if type(item.get("epoch")) is not int or item["epoch"] < 0:
        raise ValueError("epoch는 비음수 정수여야 함")
    if not isinstance(item.get("fields"), dict):
        raise ValueError("fields는 객체여야 함")
    if not isinstance(item.get("raw_hex"), str) or not re.fullmatch(r"(?:[0-9a-fA-F]{2})*", item["raw_hex"]):
        raise ValueError("raw_hex가 바이트열의 hex 형식이 아님")
    if item["kind"] == "message" and not isinstance(item.get("message_name"), str):
        raise ValueError("메시지 이름이 없음")


def _jsonl(path):
    """태그만 복구한다. 저장된 ID·시간·순서·경계를 다시 만들지 않는다."""
    file_id = uuid.uuid5(uuid.NAMESPACE_URL, path.as_uri()).hex
    meta = {"source_file": str(path), "scope": "file"}
    yield envelope("raw_jsonl", file_id, "boundary", reason="file_start", **meta)
    with path.open("rb") as source:
        line_number = 0
        while True:
            raw = source.readline(MAX_LINE_BYTES + 1)
            if not raw:
                break
            line_number += 1
            try:
                if len(raw) > MAX_LINE_BYTES:
                    raise ValueError("원문 JSONL 한 행 크기 제한 초과")
                if not raw.endswith(b"\n"):
                    raise ValueError("불완전한 마지막 행: 닫힌 원문 파일만 재생할 것")
                if not raw.strip():
                    continue
                item = decode_json(raw.decode("utf-8"))
                _validate(item)
            except (ValueError, TypeError, UnicodeError) as exc:
                raise ValueError(f"{path}:{line_number}: {exc}") from exc
            yield item
    yield envelope("raw_jsonl", file_id, "boundary", reason="file_end", **meta)


def _bin(path):
    """DFReader의 실제 필드와 원본 바이트를 함께 보존하고 보간 _timestamp는 쓰지 않는다."""
    from pymavlink import DFReader

    if path.stat().st_size == 0:
        raise ValueError(f"빈 BIN 파일: {path}")
    run_id = uuid.uuid5(uuid.NAMESPACE_URL, path.as_uri()).hex
    meta = {"source_file": str(path)}
    reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
    index = 0
    cursor = 0

    def opaque(start, end):
        """파서가 건너뛴 바이트도 작은 조각으로 그대로 남긴다."""
        nonlocal index
        while start < end:
            stop = min(start + 65536, end)
            index += 1
            yield envelope("ardupilot_bin", run_id, "opaque", **meta, record_index=index,
                           raw_hex=reader.data_map[start:stop].hex(),
                           native={"offset": start, "length": stop - start},
                           fields={"reason": "unparsed_bytes"})
            start = stop

    try:
        yield envelope("ardupilot_bin", run_id, "boundary", **meta, record_index=index,
                       reason="run_start", scope="run", parser="pymavlink.DFReader_binary")
        while True:
            message = reader.recv_msg()
            if message is None:
                yield from opaque(cursor, reader.data_len)
                break
            end = reader.offset
            start = end - message.fmt.len
            if not cursor <= start < end <= reader.data_len:
                raise ValueError(f"DFReader 바이트 경계 불일치: {path}, offset={end}")
            yield from opaque(cursor, start)
            fields = message.to_dict()
            fields.pop("mavpackettype", None)
            native_time = None
            if "TimeUS" in fields:
                native_time = {"field": "TimeUS", "value": fields["TimeUS"], "unit": "us", "clock": "boot"}
            index += 1
            yield envelope("ardupilot_bin", run_id, "message", **meta, record_index=index,
                           message_name=message.get_type(), fields=fields, source_time=native_time,
                           raw_hex=reader.data_map[start:end].hex(),
                           native={"offset": start, "length": end - start, "type_id": message.fmt.type,
                                   "format": message.fmt.format, "columns": list(message.fmt.columns),
                                   "units": message.fmt.units})
            cursor = end
    finally:
        reader.close()
    yield envelope("ardupilot_bin", run_id, "boundary", **meta, record_index=index + 1,
                   reason="run_end", scope="run")


def iter_records(path, *, source_kind="auto", decoder_path=None):
    """파일/폴더를 원래 기록 순서로 공급한다. 조기 중단 시 반복자의 close()를 호출한다.

    decoder_path는 호출부의 공통 인자로만 허용한다. JSONL은 이미 디코딩된 원문이고
    BIN은 DataFlash이므로 MAVLink dialect로 재디코딩하지 않는다.
    """
    files, kind = _files(path, source_kind)
    for source in files:
        if kind == "raw_jsonl":
            yield from _jsonl(source)
        else:
            yield from _bin(source)
