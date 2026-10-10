"""새 run의 명령 기반 배터리 전압·전류·SOC를 계산한다 (표준 라이브러리만 사용).

입력: 시작 전에 고른 프로파일 dict, PX4 원래 모터 명령 4개(rad/s), 시뮬레이션 초.
출력: 팩 V/A, SOC fraction, run 시작 이후 소비 mAh, 시뮬레이션 초를 담은 dict.
주요 호출자: battery_link와 최종 단위시험. 네트워크·MAVLink·기체 제어는 하지 않는다.
명령→16 V 벤치 부하는 근사이며 RPM/전류 측정이나 전압→추진력 피드백이 아니다.
"""

from bisect import bisect_right
import json
import math
from pathlib import Path
import re


PROFILE_DIR = Path(__file__).resolve().parents[1] / "profiles" / "battery"
LOAD_PATH = PROFILE_DIR.parent / "load.json"
_PROFILE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}")


def _finite(value, name):
    """결측·문자열·bool·비유한 수를 거부하여 정상 0으로 위장하지 않는다."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} is too large") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _read_json(path):
    """명시한 JSON 객체만 읽는다. 파일/문법 오류를 기본값으로 대체하지 않는다."""
    with Path(path).open(encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _curve(data, x_key, y_key, start):
    """0..1 축의 순서·끝점·양수 출력을 검사하고 변경 불가 tuple로 보관한다."""
    xs, ys = data.get(x_key), data.get(y_key)
    if (not isinstance(xs, (list, tuple)) or not isinstance(ys, (list, tuple))
            or len(xs) < 2 or len(xs) != len(ys)):
        raise ValueError(f"{x_key}/{y_key} need equal-length arrays with at least two points")
    xs = tuple(_finite(x, x_key) for x in xs)
    ys = tuple(_finite(y, y_key) for y in ys)
    if xs[0] < 0 or xs[-1] != 1 or any(b <= a for a, b in zip(xs, xs[1:])):
        raise ValueError(f"{x_key} must increase strictly within 0..1 and end at 1")
    if start is not None and xs[0] != start:
        raise ValueError(f"{x_key} must start at {start}")
    if any(y <= 0 for y in ys):
        raise ValueError(f"{y_key} values must be positive")
    return xs, ys


def _interpolate(value, xs, ys):
    """검사된 구간 안에서 인접 두 점을 선형 보간한다. 외삽·단조화는 하지 않는다."""
    if value <= xs[0]:
        return ys[0]
    if value >= xs[-1]:
        return ys[-1]
    high = bisect_right(xs, value)
    low = high - 1
    fraction = (value - xs[low]) / (xs[high] - xs[low])
    return ys[low] + fraction * (ys[high] - ys[low])


def _profile_values(profile):
    """프로파일의 물리 입력을 검사해 ID, mAh, Ω, 직렬 수, SOC/V 곡선을 돌려준다."""
    if not isinstance(profile, dict):
        raise ValueError("profile must be a dict from load_profile or an explicit configuration")
    if "schema" in profile and profile["schema"] != "capstone.battery_profile.v1":
        raise ValueError("unsupported battery profile schema")
    profile_id = profile.get("id")
    if not isinstance(profile_id, str) or _PROFILE_ID.fullmatch(profile_id) is None:
        raise ValueError("profile id must be 1..64 ASCII letters/digits/_.- and start with a letter/digit")
    capacity = _finite(profile.get("capacity_ah"), "capacity_ah")
    resistance = _finite(profile.get("pack_resistance_ohm"), "pack_resistance_ohm")
    if capacity <= 0 or resistance <= 0:
        raise ValueError("capacity_ah and pack_resistance_ohm must be positive")
    capacity_mah = _finite(capacity * 1000.0, "capacity_mah")
    for key in ("series_cells", "parallel_cells"):
        if key == "parallel_cells" and key not in profile:
            continue  # 팩 Ah/Ω에 이미 반영된 메타데이터이며 계산에는 다시 곱하지 않는다.
        count = profile.get(key)
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError(f"{key} must be a positive integer")
    cells = profile["series_cells"]
    _finite(cells, "series_cells")
    if "initial_soc" in profile and not 0 <= _finite(profile["initial_soc"], "initial_soc") <= 1:
        raise ValueError("profile initial_soc must be within 0..1")
    xs, ys = _curve(profile, "ocv_soc", "ocv_cell_v", 0.0)
    _finite(cells * max(ys), "pack OCV")
    return profile_id, capacity_mah, resistance, cells, xs, ys


def load_profile(profile_id_or_path: str | Path, profiles_dir=None) -> dict:
    """정확한 ID 또는 JSON 경로를 읽어 검사한 plain dict를 반환한다.

    profiles_dir=None이면 이 저장소 profiles/battery를 쓴다. 문자열 경로는
    '/' 포함 또는 '.json' 끝, Path 객체는 항상 경로다. 다른 문자열은 ID다.
    ID 선택 시 파일 내부 ID도 일치해야 한다. 누락 파일은 FileNotFoundError,
    내용/ID 오류는 ValueError이며, 유사 ID 검색이나 초기 프로파일 fallback은 없다.
    """
    requested_id = None
    if isinstance(profile_id_or_path, Path):
        path = profile_id_or_path
    elif isinstance(profile_id_or_path, str):
        if "/" in profile_id_or_path or profile_id_or_path.endswith(".json"):
            path = Path(profile_id_or_path)
        else:
            if _PROFILE_ID.fullmatch(profile_id_or_path) is None:
                raise ValueError("invalid profile id; whitespace and identifiers are not normalized")
            requested_id = profile_id_or_path
            directory = PROFILE_DIR if profiles_dir is None else Path(profiles_dir)
            path = directory / f"{requested_id}.json"
    else:
        raise ValueError("profile_id_or_path must be a string or Path")
    profile = _read_json(path)
    _profile_values(profile)
    if requested_id is not None and profile["id"] != requested_id:
        raise ValueError("profile file id does not match the requested id")
    return profile


class Battery:
    """한 객체가 한 run이다. 선택 변경·reset API는 없고 오류 뒤에는 새 run이 필요하다."""

    def __init__(self, profile: dict, initial_soc: float = 0.95, load_config=None):
        """시작 상태를 한 번 정한다. 입력 dict/배열 변경은 이미 만든 객체에 반영하지 않는다."""
        (self._profile_id, self._capacity_mah, self._resistance, self._cells,
         self._ocv_soc, self._ocv_cell_v) = _profile_values(profile)
        self._initial_soc = _finite(initial_soc, "initial_soc")
        if not 0 <= self._initial_soc <= 1:
            raise ValueError("initial_soc must be within 0..1")

        config = _read_json(LOAD_PATH) if load_config is None else load_config
        if not isinstance(config, dict):
            raise ValueError("load_config must be a dict or None")
        if "schema" in config and config["schema"] != "capstone.command_load.v1":
            raise ValueError("unsupported load configuration schema")
        self._command_max = _finite(config.get("command_max_rad_s"), "command_max_rad_s")
        self._auxiliary = _finite(config.get("auxiliary_current_a"), "auxiliary_current_a")
        self._max_gap_s = _finite(config.get("max_gap_s"), "max_gap_s")
        bench_voltage = _finite(config.get("bench_voltage_v"), "bench_voltage_v")
        if self._command_max <= 0 or self._max_gap_s <= 0 or bench_voltage <= 0 or self._auxiliary < 0:
            raise ValueError("load limits/bench voltage must be positive; auxiliary current must be nonnegative")
        self._throttle, self._motor_amps = _curve(config, "throttle", "motor_current_a", None)
        if self._throttle[0] <= 0:
            raise ValueError("first bench throttle must be positive for the low-throttle approximation")
        _finite(self._auxiliary + 4 * max(self._motor_amps), "maximum pack current")
        self._available_mah = self._initial_soc * self._capacity_mah
        self._consumed_mah = 0.0
        self._last_time_s = None
        self._last_current_a = None
        self._failure = None

    @property
    def profile_id(self):
        """정답 메타데이터용 선택 ID를 읽는다. setter나 선택 변경 기능은 없다."""
        return self._profile_id

    @property
    def series_cells(self):
        """가상 팩 직렬 수를 읽는다. 개별 셀 전압을 측정했다는 뜻은 아니다."""
        return self._cells

    def _current(self, commands):
        """원래 명령 4개를 정규화하여 벤치 근사 전류와 가정한 보조부하를 합산한다."""
        if not isinstance(commands, (list, tuple)) or len(commands) != 4:
            raise ValueError("command_rad_s must contain exactly four fresh commands")
        current = self._auxiliary
        for command in commands:
            command = _finite(command, "command_rad_s")
            if command < 0:
                raise ValueError("command_rad_s must be nonnegative")
            throttle = min(command, self._command_max) / self._command_max
            if throttle < self._throttle[0]:
                # 30% 미만은 측정값이 아니라 기존의 0까지 이어지는 이차식 가정이다.
                current += self._motor_amps[0] * (throttle / self._throttle[0]) ** 2
            else:
                current += _interpolate(throttle, self._throttle, self._motor_amps)
        return _finite(current, "pack current")

    def update(self, command_rad_s, sim_time_s: float) -> dict:
        """신선한 원래 명령과 시뮬레이션 초로 plain dict 샘플을 만든다.

        첫 표본은 적분하지 않는다. 이후 직전 전류를 짧은 구간에만 유지한다.
        같은 시각은 pause/중복이므로 적분하지 않지만 순간 부하는 바꿀 수 있다.
        역행·긴 공백·잘못된 입력은 ValueError로 run을 영구 무효화한다.
        호출자가 실패를 무시해 미관측 구간을 정상 적분하는 일을 막기 위해서다.
        """
        if self._failure is not None:
            raise ValueError(f"battery run invalid ({self._failure}); create a Battery for a new run")
        try:
            now = _finite(sim_time_s, "sim_time_s")
            if now < 0:
                raise ValueError("sim_time_s must be nonnegative")
            elapsed = 0.0 if self._last_time_s is None else now - self._last_time_s
            if elapsed < 0:
                raise ValueError("simulation clock moved backward; start a new run")
            if elapsed > self._max_gap_s:
                raise ValueError(f"simulation input gap {elapsed} s exceeds {self._max_gap_s} s; start a new run")
            current = self._current(command_rad_s)
            consumed = self._consumed_mah
            if elapsed > 0:
                # 왼쪽 구간 유지: 새 명령을 과거 구간 전체에 소급 적용하지 않는다.
                consumed = _finite(consumed + self._last_current_a * (elapsed / 3.6), "consumed_mah")
            consumed = min(consumed, self._available_mah)
            soc = 0.0 if consumed >= self._available_mah else max(
                0.0, min(1.0, self._initial_soc - consumed / self._capacity_mah))
            cell_v = _interpolate(soc, self._ocv_soc, self._ocv_cell_v)
            voltage = _finite(self._cells * cell_v - current * self._resistance, "voltage_v")
            if voltage <= 0:
                raise ValueError("modeled voltage is nonpositive; load/profile is outside this model")
        except ValueError as error:
            self._failure = str(error)
            raise

        # 검사/계산이 끝난 뒤에만 상태를 반영한다. 반환 dict 수정도 상태에 영향이 없다.
        self._consumed_mah = consumed
        self._last_time_s = now
        self._last_current_a = current
        return {"voltage_v": voltage, "current_a": current, "soc": soc,
                "consumed_mah": consumed, "sim_time_s": now}
