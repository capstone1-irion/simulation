"""CMU eVTOL initial-reference profiles; laboratory cells, not drone packs."""
import math
import csv
import hashlib
import json
import io
from pathlib import Path

FIELDS = ["elapsed_s", "voltage_v", "current_a", "temperature_c"]
SPLITS = {"VAH01": "train", "VAH17": "validation", "VAH27": "test_normal"}
KINDS = {"initial_reference_discharge", "first_mission_discharge"}
INITIAL_SEGMENTS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 3, 4, 5, 6, 7, 0]


# 목적: 셀의 초기 프로토콜에서 기준 방전과 첫 임무 방전 프로파일을 추출한다.
# 입출력: 셀 ID·원본 행 → samples/metadata 목록(s·V·방전 양수 A·°C, 기준 용량 Ah).
# 변경/제약: 설명만 추가, 동작 유지. 순서·임무 길이를 검사하며 단일 셀·무보간·상대 초기 기준이다.
def build_profiles(cell, rows):
    """Use protocol order, not the unreliable global tester cycle numbers."""
    if cell not in SPLITS:
        raise ValueError("Unsupported source cell")
    groups = []
    for index, row in enumerate(rows):
        markers = [float(row[field]) for field in ("cycleNumber", "Ns")]
        if any(not math.isfinite(value) or not value.is_integer() for value in markers):
            raise ValueError("Expected integer protocol marker")
        key = tuple(int(value) for value in markers)
        if not groups or key != groups[-1]["key"]:
            if len(groups) == len(INITIAL_SEGMENTS):
                break
            groups.append({"key": key, "start": index, "rows": []})
        groups[-1]["rows"].append(row)
    if len(groups) != len(INITIAL_SEGMENTS):
        raise ValueError("No complete initial protocol and post-mission witness")
    if ([g["key"][1] for g in groups] != INITIAL_SEGMENTS
            or [g["key"][0] for g in groups] != [0] * 16 + [1]):
        raise ValueError("Unverified initial protocol order")
    for block, duration in zip(groups[12:15], (75.0, 800.0, 105.0)):
        measured = float(block["rows"][-1]["time_s"]) - float(block["rows"][0]["time_s"])
        if not math.isfinite(measured) or abs(measured - duration) > 0.1:
            raise ValueError("Invalid first mission phase duration")
    selected = [("initial_reference_discharge", groups[3:8]), ("first_mission_discharge", groups[12:15])]
    result = []
    baseline = None
    for kind, blocks in selected:
        origin = float(blocks[0]["rows"][0]["time_s"])
        samples = []
        spans = []
        for block in blocks:
            start = len(samples)
            for row in block["rows"]:
                samples.append({"elapsed_s": float(row["time_s"]) - origin,
                                "voltage_v": float(row["Ecell_V"]),
                                "current_a": -float(row["I_mA"]) / 1000.0,
                                "temperature_c": float(row["Temperature__C"])})
            spans.append({"source_row_start": block["start"], "source_row_count": len(block["rows"]),
                          "tester_cycle_number": block["key"][0], "tester_segment": block["key"][1],
                          "profile_row_start": start, "profile_row_count": len(block["rows"]),
                          "source_time_start_s": float(block["rows"][0]["time_s"]),
                          "source_time_end_s": float(block["rows"][-1]["time_s"])})
        if len(samples) < 2:
            raise ValueError("Incomplete initial measurements")
        previous = -1.0
        for sample in samples:
            if any(not math.isfinite(value) for value in sample.values()):
                raise ValueError("Measurements must be finite")
            if sample["elapsed_s"] <= previous:
                raise ValueError("Profile time must strictly increase")
            if sample["current_a"] < 0 or sample["voltage_v"] <= 0 or sample["temperature_c"] < -273.15:
                raise ValueError("Invalid physical measurement")
            previous = sample["elapsed_s"]
        if baseline is None:
            baseline = sum((b["elapsed_s"] - a["elapsed_s"]) * (a["current_a"] + b["current_a"]) / 2
                           for a, b in zip(samples, samples[1:])) / 3600.0
        metadata = {
            "source_dataset": "CMU_eVTOL_v3", "source_cell": cell, "split": SPLITS[cell], "kind": kind,
            "classification": "normal_initial_reference", "soh_percent": 100.0,
            "health_basis": "relative_initial_reference_capacity", "absolute_soh_certified": False,
            "baseline_capacity_ah": baseline, "baseline_capacity_time_basis": "source_time_s_seconds",
            "capacity_multiplier": 1.0, "injected_degradation": False,
            "sample_count": len(samples), "duration_s": samples[-1]["elapsed_s"],
            "observable_fields": FIELDS[1:], "time_field": FIELDS[0],
            "source_current_conversion": "current_a = -I_mA / 1000", "source_step_spans": spans,
            "soc_status": "not_reported_by_source", "electrical_scope": "single_cell_no_pack_scaling",
            "runtime_provenance": "CMU_laboratory_not_PX4_telemetry", "interpolation": "none",
            "mission_phase_order": ["takeoff", "cruise", "landing"] if kind == "first_mission_discharge" else [],
        }
        result.append({"id": cell + "_" + kind, "samples": samples, "metadata": metadata})
    return result


SOURCE_PINS = {
    "VAH01": (3145655, "07ba5c1e3427e9b8dc4c10f7017fcf5aa215f7ff989e7a1a6a0b4b7d04462e90", 110884708, "5b0ac8963f1d161d4b74c731f8d8640a"),
    "VAH17": (3145641, "aa5abd4560d5754d19c1e91e42a9a78bfade3cd4535a11771da1cd2e47821997", 65550764, "30419d4152fd0a1cd00f4725cbdcfc49"),
    "VAH27": (3145672, "74083637595aeb2729a955ef4e51bfbe7a3c30a1761f68314aab5f478921bca3", 39543963, "8512c658d755d456d7c65c68d4f10f49"),
}
SOURCE_FIELDS = ["time_s", "Ecell_V", "I_mA", "EnergyCharge_W_h", "QCharge_mA_h", "EnergyDischarge_W_h", "QDischarge_mA_h", "Temperature__C", "cycleNumber", "Ns"]


# 목적: 핀으로 고정한 원본 CSV를 확인하고 행과 출처 정보를 읽는다.
# 입출력: 로컬 경로·셀 ID → CSV 문자열 행 목록과 provenance(s·V·mA·°C 등 원본 단위).
# 변경/제약: 설명만 추가, 동작 유지. 전체 파일 또는 검증된 초기 바이트 구간만 허용하며 다운로드하지 않는다.
def read_source(path, cell):
    """Accept a pinned complete file or a separately verified initial byte slice."""
    if cell not in SOURCE_PINS:
        raise ValueError("Unsupported source cell")
    data = Path(path).read_bytes()
    size, prefix_sha, total, original_md5 = SOURCE_PINS[cell]
    digest = hashlib.sha256(data).hexdigest()
    complete = len(data) == total and hashlib.md5(data).hexdigest() == original_md5
    initial_slice = len(data) == size and digest == prefix_sha
    if not (complete or initial_slice):
        raise ValueError("Source checksum/size mismatch; partial downloads are not accepted")
    reader = csv.DictReader(io.StringIO(data.decode("utf-8-sig")))
    if reader.fieldnames != SOURCE_FIELDS:
        raise ValueError("Unexpected source CSV schema")
    provenance = {"source_dataset_doi": "10.1184/R1/14226830.v3", "source_file": cell + ".csv",
                  "source_bytes": len(data), "source_sha256": digest, "original_file_bytes": total,
                  "original_repository_md5": original_md5, "whole_file_md5_verified_locally": complete,
                  "source_download_scope": "complete_file" if complete else "verified_initial_http_ranges",
                  "source_units": {"time_s": "s", "Ecell_V": "V", "I_mA": "mA", "Temperature__C": "degC"},
                  "licence": "CC BY 4.0"}
    return list(reader), provenance


# 목적: 메타데이터의 원본 출처·크기·해시 표기·단위가 고정 계약과 맞는지 검사한다.
# 입출력: metadata·필수 여부 → 성공 시 None, 불일치 시 ValueError(크기 byte, 원본 s·V·mA·°C).
# 변경/제약: 설명만 추가, 동작 유지. 원본 파일을 다시 읽지 않으며 required=False이면 출처 누락을 허용한다.
def _validate_source_integrity(meta, required=False):
    if "source_integrity" not in meta:
        if required:
            raise ValueError("Published profiles require source integrity provenance")
        return
    integrity = meta["source_integrity"]
    if not isinstance(integrity, dict):
        raise ValueError("Invalid source integrity provenance")
    cell = meta["source_cell"]
    size, prefix_sha, total, original_md5 = SOURCE_PINS[cell]
    expected = {"source_dataset_doi": "10.1184/R1/14226830.v3", "source_file": cell + ".csv",
                "original_file_bytes": total, "original_repository_md5": original_md5,
                "source_units": {"time_s": "s", "Ecell_V": "V", "I_mA": "mA", "Temperature__C": "degC"},
                "licence": "CC BY 4.0"}
    if any(integrity.get(key) != value for key, value in expected.items()):
        raise ValueError("Source provenance conflicts with literal source pins/units")
    digest = integrity.get("source_sha256")
    if (not isinstance(digest, str) or len(digest) != 64
            or any(c not in "0123456789abcdef" for c in digest) or digest == "0" * 64):
        raise ValueError("Invalid source SHA-256")
    scope = integrity.get("source_download_scope")
    if scope == "verified_initial_http_ranges":
        if (integrity.get("source_bytes") != size or digest != prefix_sha
                or integrity.get("whole_file_md5_verified_locally") is not False):
            raise ValueError("Initial source prefix integrity/scope conflict")
    elif scope == "complete_file":
        if (integrity.get("source_bytes") != total
                or integrity.get("whole_file_md5_verified_locally") is not True):
            raise ValueError("Complete source integrity/scope conflict")
    else:
        raise ValueError("Unverified source download scope")


# 목적: 선택한 원본 구간과 프로파일 행의 연속 일대일 대응·프로토콜 순서를 검사한다.
# 입출력: samples/metadata 프로파일 → 성공 시 None, 불일치 시 ValueError(행 인덱스·개수, 시간 s).
# 변경/제약: 설명만 추가, 동작 유지. 모든 표본을 덮어야 하며 첫 임무의 각 구간 길이도 검사한다.
def _validate_selection_spans(profile):
    meta, rows = profile["metadata"], profile["samples"]
    mission = meta["kind"] == "first_mission_discharge"
    segments = [4, 5, 6] if mission else [3, 4, 5, 6, 7]
    spans = meta.get("source_step_spans")
    if not isinstance(spans, list) or len(spans) != len(segments):
        raise ValueError("Missing/incomplete selected source spans")
    profile_end, source_end, previous_time_end = 0, None, -1.0
    origin = None
    for index, (span, segment) in enumerate(zip(spans, segments)):
        integer_fields = ("profile_row_start", "profile_row_count", "source_row_start", "source_row_count",
                          "tester_cycle_number", "tester_segment")
        if not isinstance(span, dict) or any(type(span.get(key)) is not int for key in integer_fields):
            raise ValueError("Selected span indices/counts must be integers")
        start, count = span["profile_row_start"], span["profile_row_count"]
        if (start != profile_end or count <= 0 or start + count > len(rows)
                or span["source_row_count"] != count or span["source_row_start"] < 0
                or (source_end is not None and span["source_row_start"] != source_end)):
            raise ValueError("Selected source/profile spans must provide contiguous one-to-one coverage")
        if span["tester_cycle_number"] != 0 or span["tester_segment"] != segment:
            raise ValueError("Selected source protocol segment order mismatch")
        times = [span.get(key) for key in ("source_time_start_s", "source_time_end_s")]
        if any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in times):
            raise ValueError("Selected source span times must be finite")
        time_start, time_end = float(span["source_time_start_s"]), float(span["source_time_end_s"])
        if time_start <= previous_time_end or time_end < time_start:
            raise ValueError("Selected source span times must increase")
        if origin is None:
            origin = time_start
        if (not math.isclose(time_start - origin, rows[start]["elapsed_s"], rel_tol=0, abs_tol=1e-7)
                or not math.isclose(time_end - origin, rows[start + count - 1]["elapsed_s"], rel_tol=0, abs_tol=1e-7)):
            raise ValueError("Selected source/profile span time alignment mismatch")
        if mission and abs(time_end - time_start - (75.0, 800.0, 105.0)[index]) > 0.1:
            raise ValueError("Invalid selected mission phase duration")
        profile_end = start + count
        source_end = span["source_row_start"] + count
        previous_time_end = time_end
    if profile_end != len(rows):
        raise ValueError("Selected spans do not cover every profile sample")


# 목적: 단일 프로파일의 식별자·정상 초기 기준·관측값·선택 구간 계약을 검사한다.
# 입출력: 프로파일 dict → 성공 시 None, 계약 위반 시 ValueError(표본 s·V·A·°C, 기준 용량 Ah).
# 변경/제약: 설명만 추가, 동작 유지. 기준 방전은 전류 적분과 비교하지만 파일 해시·번들 전체는 여기서 검사하지 않는다.
def validate_profile(profile):
    meta = profile["metadata"]
    cell = meta["source_cell"]
    if cell not in SPLITS or meta["kind"] not in KINDS or profile["id"] != cell + "_" + meta["kind"]:
        raise ValueError("Invalid source/profile identifier")
    if meta["split"] != SPLITS[cell]:
        raise ValueError("Source cell split conflict")
    _validate_source_integrity(meta)
    if (meta["source_dataset"] != "CMU_eVTOL_v3" or meta["classification"] != "normal_initial_reference"
            or meta["soh_percent"] != 100.0 or meta["health_basis"] != "relative_initial_reference_capacity"
            or meta["absolute_soh_certified"] is not False or meta["capacity_multiplier"] != 1.0
            or meta["injected_degradation"] is not False):
        raise ValueError("Invalid initial normal health metadata")
    if meta["observable_fields"] != FIELDS[1:] or meta["time_field"] != FIELDS[0]:
        raise ValueError("Observable feature contract changed")
    expected = {"baseline_capacity_time_basis": "source_time_s_seconds",
                "source_current_conversion": "current_a = -I_mA / 1000", "interpolation": "none",
                "soc_status": "not_reported_by_source", "electrical_scope": "single_cell_no_pack_scaling",
                "runtime_provenance": "CMU_laboratory_not_PX4_telemetry",
                "mission_phase_order": ["takeoff", "cruise", "landing"] if meta["kind"] == "first_mission_discharge" else []}
    if any(meta.get(key) != value for key, value in expected.items()):
        raise ValueError("Unit/no-interpolation/measurement scope contract changed")
    if not math.isfinite(meta["baseline_capacity_ah"]) or meta["baseline_capacity_ah"] <= 0:
        raise ValueError("Invalid initial reference capacity")
    rows = profile["samples"]
    if len(rows) < 2 or len(rows) != meta["sample_count"]:
        raise ValueError("Invalid sample count")
    previous = -1.0
    for row in rows:
        if set(row) != set(FIELDS):
            raise ValueError("Unexpected observable or hidden feature")
        if any(not math.isfinite(float(value)) for value in row.values()):
            raise ValueError("Measurements must be finite")
        if row["elapsed_s"] <= previous:
            raise ValueError("Profile time must strictly increase")
        if row["current_a"] < 0 or row["voltage_v"] <= 0 or row["temperature_c"] < -273.15:
            raise ValueError("Invalid physical measurement")
        previous = row["elapsed_s"]
    if rows[0]["elapsed_s"] != 0 or rows[-1]["elapsed_s"] != meta["duration_s"]:
        raise ValueError("Profile duration mismatch")
    _validate_selection_spans(profile)
    if meta["kind"] == "initial_reference_discharge":
        measured_ah = sum((b["elapsed_s"] - a["elapsed_s"]) * (a["current_a"] + b["current_a"]) / 2
                          for a, b in zip(rows, rows[1:])) / 3600.0
        if not math.isclose(meta["baseline_capacity_ah"], measured_ah, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError("Initial reference capacity does not match selected RPT current/time integral")


# 목적: 지원 셀마다 기준 방전·첫 임무 방전 ID가 빠짐없이 한 번씩 있는지 검사한다.
# 입출력: 프로파일 ID 목록 → 성공 시 None, 누락·중복·추가 시 ValueError(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. SPLITS·KINDS의 조합만 허용하고 순서는 요구하지 않는다.
def _validate_bundle_ids(ids):
    expected = {cell + "_" + kind for cell in SPLITS for kind in KINDS}
    if len(ids) != len(expected) or set(ids) != expected:
        raise ValueError("Bundle requires exactly both profile kinds for all three source cells")


# 목적: 같은 셀의 기준 방전과 첫 임무 방전이 동일 출처·초기 용량을 공유하는지 확인한다.
# 입출력: 프로파일 목록 → 성공 시 None, 짝 간 불일치 시 ValueError(용량 Ah, 원본 시간 s·행 인덱스).
# 변경/제약: 설명만 추가, 동작 유지. 두 ID와 출처 메타데이터가 존재해야 하며 임무는 기준 방전 뒤에 와야 한다.
def _validate_cell_pairs(profiles):
    by_id = {p["id"]: p["metadata"] for p in profiles}
    for cell in SPLITS:
        reference = by_id[cell + "_initial_reference_discharge"]
        mission = by_id[cell + "_first_mission_discharge"]
        if mission["baseline_capacity_ah"] != reference["baseline_capacity_ah"]:
            raise ValueError("Same-cell mission must share the initial RPT capacity baseline")
        if mission["source_integrity"] != reference["source_integrity"]:
            raise ValueError("Same-cell profiles must share source integrity provenance")
        last = reference["source_step_spans"][-1]
        first = mission["source_step_spans"][0]
        if (first["source_row_start"] < last["source_row_start"] + last["source_row_count"]
                or first["source_time_start_s"] <= last["source_time_end_s"]):
            raise ValueError("Mission source selection must follow initial RPT rows/time")


# 목적: 검증·직렬화를 마친 프로파일들을 CSV·메타데이터·해시 manifest로 저장한다.
# 입출력: 전체 프로파일 목록·새 출력 경로 → 파일 번들과 manifest dict(표본 s·V·A·°C, 용량 Ah).
# 변경/제약: 설명만 추가, 동작 유지. 기존 출력 디렉터리를 덮어쓰지 않으며 중단 시 원자적 저장은 보장하지 않는다.
def write_bundle(profiles, output):
    """Preflight contracts/serialization before mkdir; not crash-atomic I/O."""
    _validate_bundle_ids([p["id"] for p in profiles])
    for p in profiles:
        validate_profile(p)
        _validate_source_integrity(p["metadata"], required=True)
    _validate_cell_pairs(profiles)
    manifest = {"schema": "arion.battery.source_normal_profiles.v1", "source_dataset": "CMU_eVTOL_v3",
                "features": FIELDS[1:], "time_field": FIELDS[0], "normal_only": True,
                "abnormal_policy": "independent_test_only_not_generated", "profile_count": len(profiles),
                "splits": {split: [] for split in SPLITS.values()}, "profiles": []}
    prepared = []
    for p in profiles:
        identifier = p["id"]
        metadata_bytes = (json.dumps(p["metadata"], indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(p["samples"])
        csv_bytes = stream.getvalue().encode("utf-8")
        csv_name, meta_name = identifier + ".csv", identifier + ".meta.json"
        prepared.extend(((csv_name, csv_bytes), (meta_name, metadata_bytes)))
        manifest["profiles"].append({"id": identifier, "csv": csv_name, "metadata": meta_name,
                                     "csv_sha256": hashlib.sha256(csv_bytes).hexdigest(),
                                     "metadata_sha256": hashlib.sha256(metadata_bytes).hexdigest()})
        manifest["splits"][p["metadata"]["split"]].append(identifier)
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, data in prepared + [("manifest.json", manifest_bytes)]:
        with (output / name).open("xb") as stream:
            stream.write(data)
    return manifest


# 목적: 번들에서 요청한 프로파일의 경로·파일 해시·단일 프로파일 계약을 확인해 읽는다.
# 입출력: 번들 경로·ID → float 표본과 metadata를 담은 프로파일 dict(s·V·A·°C, 기준 용량 Ah).
# 변경/제약: 설명만 추가, 동작 유지. 번들 밖 경로와 중복 ID는 거부하며 전체 번들 검증은 verify_bundle의 역할이다.
def load_profile(bundle, identifier):
    bundle = Path(bundle)
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    entries = [e for e in manifest["profiles"] if e["id"] == identifier]
    if len(entries) != 1:
        raise ValueError("Profile identity missing/duplicated")
    entry = entries[0]
    for key, suffix in (("csv", ".csv"), ("metadata", ".meta.json")):
        name = entry[key]
        path = bundle / name
        if name != identifier + suffix or path.resolve().parent != bundle.resolve():
            raise ValueError("Profile path escapes bundle")
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry[key + "_sha256"]:
            raise ValueError("Profile checksum mismatch")
    with (bundle / entry["csv"]).open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != FIELDS:
            raise ValueError("Unexpected observable CSV features")
        rows = [{key: float(value) for key, value in row.items()} for row in reader]
    profile = {"id": identifier, "samples": rows,
               "metadata": json.loads((bundle / entry["metadata"]).read_text(encoding="utf-8"))}
    validate_profile(profile)
    return profile


# 목적: manifest·각 파일·셀별 짝·분할 배정까지 로컬 번들 전체의 계약을 검사한다.
# 입출력: 번들 경로 → 검증 범위와 프로파일·표본·분할별 개수 dict(개수는 물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 로컬 무결성 확인일 뿐 PX4 실행이나 절대 SOH를 인증하지 않는다.
def verify_bundle(bundle):
    bundle = Path(bundle)
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    if (manifest["schema"] != "arion.battery.source_normal_profiles.v1"
            or manifest["source_dataset"] != "CMU_eVTOL_v3" or manifest["features"] != FIELDS[1:]
            or manifest["time_field"] != FIELDS[0] or manifest["normal_only"] is not True
            or manifest["abnormal_policy"] != "independent_test_only_not_generated"):
        raise ValueError("Invalid source normal bundle contract")
    ids = [entry["id"] for entry in manifest["profiles"]]
    _validate_bundle_ids(ids)
    assigned = [identifier for group in manifest["splits"].values() for identifier in group]
    if not ids or len(ids) != len(set(ids)) or manifest["profile_count"] != len(ids) or sorted(ids) != sorted(assigned):
        raise ValueError("Invalid count/split assignment")
    if set(manifest["splits"]) != set(SPLITS.values()):
        raise ValueError("Unexpected source splits")
    count = 0
    profiles = []
    for identifier in ids:
        p = load_profile(bundle, identifier)
        _validate_source_integrity(p["metadata"], required=True)
        if identifier not in manifest["splits"][p["metadata"]["split"]]:
            raise ValueError("Manifest/source split conflict")
        count += len(p["samples"])
        profiles.append(p)
    _validate_cell_pairs(profiles)
    return {"passed": True, "scope": "local_profile_integrity_not_PX4_telemetry", "profile_count": len(ids),
            "sample_count": count, "split_counts": {k: len(v) for k, v in manifest["splits"].items()},
            "absolute_soh_certified": False, "px4_runtime_verified": False}


class ProfileReplay:
    """Causal zero-order hold; never manufacture unseen future measurements."""
    # 목적: 유효한 프로파일의 표본을 복사하고 인과적 재생에 사용할 시간 인덱스를 준비한다.
    # 입출력: 프로파일 dict → 초기화된 재생 객체(표본 s·V·A·°C, 시간축 s; 반환값 없음).
    # 변경/제약: 설명만 추가, 동작 유지. 검증 실패는 전파하며 원본 표본 dict를 직접 보관하지 않는다.
    def __init__(self, profile):
        from bisect import bisect_right
        validate_profile(profile)
        self._bisect = bisect_right
        self._rows = [dict(row) for row in profile["samples"]]
        self._times = [row["elapsed_s"] for row in self._rows]
        self.reset()

    # 목적: 마지막 요청 시각을 초기화해 처음부터 또는 이전 시각부터 다시 재생할 수 있게 한다.
    # 입출력: 추가 인자 없음 → None, 내부 마지막 시각을 -1 s로 변경한다.
    # 변경/제약: 설명만 추가, 동작 유지. 표본·시간축은 바꾸지 않으며 되감기 전에 명시적으로 호출해야 한다.
    def reset(self):
        self._last = -1.0

    # 목적: 요청 시각 이하의 가장 최근 실측 표본을 영차 유지 방식으로 반환한다.
    # 입출력: 경과 시간 s → 원래 표본 시각(s)·V·A·°C를 담은 dict 복사본, 마지막 시각 초과 시 None.
    # 변경/제약: 설명만 추가, 동작 유지. 유한·비음수·비감소 시각만 허용하며 보간이나 미래 표본 사용은 하지 않는다.
    def sample(self, elapsed_s):
        elapsed_s = float(elapsed_s)
        if not math.isfinite(elapsed_s) or elapsed_s < 0 or elapsed_s < self._last:
            raise ValueError("Replay time must be finite, nonnegative and monotonic; reset before rewind")
        self._last = elapsed_s
        if elapsed_s > self._times[-1]:
            return None
        index = self._bisect(self._times, elapsed_s) - 1
        return dict(self._rows[index])


# 목적: generate/verify CLI를 통해 원본 기반 번들 생성 또는 기존 번들 검증을 실행한다.
# 입출력: argv(기본값은 프로세스 인자) → JSON stdout·실패 설명 stderr, 성공 0/처리한 오류 1(종료 코드).
# 변경/제약: 설명만 추가, 동작 유지. 로컬 자료만 다루며 argparse 종료와 명시한 예외 밖의 오류는 별도로 전파된다.
def main(argv=None):
    import argparse
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("generate")
    generate.add_argument("--sources", type=Path, required=True)
    generate.add_argument("--output", type=Path, required=True)
    verify = sub.add_parser("verify")
    verify.add_argument("--bundle", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "generate":
            profiles = []
            for cell in SPLITS:
                candidates = [args.sources / (cell + ".csv"), args.sources / (cell + ".csv.initial-prefix.csv")]
                source = next((p for p in candidates if p.is_file()), candidates[0])
                rows, provenance = read_source(source, cell)
                for p in build_profiles(cell, rows):
                    p["metadata"]["source_integrity"] = provenance
                    profiles.append(p)
                del rows
            write_bundle(profiles, args.output)
            result = verify_bundle(args.output)
        else:
            result = verify_bundle(args.bundle)
    except (ValueError, OSError, KeyError, TypeError) as error:
        print("CMU profile operation failed: " + str(error), file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
