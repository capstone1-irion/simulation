"""배터리 계산·프로파일 보존 계약을 최종 통합 검증에서 확인한다.

입력은 명시적으로 만든 합성 단위시험 값과 보존된 CMU 파생 JSON이다.
출력은 unittest 결과이며 호출자는 최종 검증 실행기다. 작성 중 실행하지 않는다.
원본 CMU CSV, V1 제품 import, Gazebo, MAVLink, 네트워크는 필요하지 않다.
"""

import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from capstone.battery import Battery, load_profile


ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT / "profiles" / "battery"
# 아래 값은 V1 catalog에서 읽은 값이다. 합성 시험 fixture와 구분한다.
EXPECTED = {
    "cmu_vah01_rpt000": (6.178676149834232, 0.04615530988203852, 100.0),
    "cmu_vah01_rpt002": (5.871438083050888, 0.045534174816292114, 95.02744504918603),
    "cmu_vah01_rpt006": (5.586921940178748, 0.0508360108368731, 90.42263754718137),
    "cmu_vah01_rpt013": (5.240295514286093, 0.05836240894542594, 84.81259394743783),
}
CATALOG_HASH = "05245eabc7943fa0d00352d11a7b2cfabfd59752b1892d8d16ed3f93ff3f5e5d"
RUNTIME_FIELDS = (
    "id", "label_ko", "capacity_ah", "pack_resistance_ohm", "initial_soc",
    "ocv_soc", "ocv_cell_v", "soh_percent",
)
BENCH_THROTTLE = [0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65,
                  0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00]
BENCH_AMPS = [1.44, 1.87, 2.29, 2.86, 3.60, 4.53, 5.61, 6.78,
              7.92, 9.20, 10.59, 12.11, 13.81, 15.68, 16.37]


def synthetic_profile():
    """식 계산용 단순 곡선을 만든다. CMU 실측 프로파일로 발행하지 않는다."""
    return {
        "schema": "capstone.battery_profile.v1",
        "id": "synthetic_test_only",
        "capacity_ah": 2.0,
        "pack_resistance_ohm": 0.05,
        "series_cells": 4,
        "parallel_cells": 1,
        "ocv_soc": [0.0, 0.5, 1.0],
        "ocv_cell_v": [3.0, 3.7, 4.2],
    }


def load_config():
    """시험마다 독립된 부하 설정 사본을 읽어 변경의 누출을 막는다."""
    return json.loads((ROOT / "profiles" / "load.json").read_text(encoding="utf-8"))


def canonical_hash(value):
    """이전 시 기록한 동일 JSON 규칙으로 배열·모든 파생 수치를 비교한다."""
    data = json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


class ProfileTests(unittest.TestCase):
    def test_all_four_profiles_and_full_curves_are_available(self):
        self.assertEqual({p.stem for p in PROFILES.glob("cmu_*.json")}, set(EXPECTED))
        for profile_id, values in EXPECTED.items():
            with self.subTest(profile_id=profile_id):
                profile = load_profile(profile_id)
                self.assertIsInstance(profile, dict)
                self.assertEqual(profile["id"], profile_id)
                self.assertEqual(tuple(profile[k] for k in (
                    "capacity_ah", "pack_resistance_ohm", "soh_percent")), values)
                self.assertEqual(profile["series_cells"], 4)
                self.assertEqual(profile["parallel_cells"], 2)
                self.assertEqual(profile["initial_soc"], 0.95)
                self.assertEqual(len(profile["ocv_soc"]), 101)
                self.assertEqual(len(profile["ocv_cell_v"]), 101)
                sample = Battery(profile).update([0.0] * 4, 0.0)
                self.assertEqual(sample["soc"], 0.95)
                self.assertTrue(math.isfinite(sample["voltage_v"]))

    def test_runtime_plus_provenance_reconstruct_exact_source_values(self):
        receipt = json.loads((PROFILES / "provenance.json").read_text(encoding="utf-8"))
        rebuilt = copy.deepcopy(receipt["retained_v1_catalog_metadata"])
        rebuilt["profiles"] = []
        for profile_id in receipt["migration"]["preserved_profile_ids"]:
            profile = load_profile(profile_id)
            original = {key: profile[key] for key in RUNTIME_FIELDS}
            self.assertEqual(canonical_hash(original), receipt["migration"][
                "source_runtime_payload_sha256"][profile_id])
            original["source"] = receipt["retained_v1_profile_sources"][profile_id]
            rebuilt["profiles"].append(original)
            self.assertEqual(original["source"]["license"], "CC BY 4.0")
        self.assertEqual(canonical_hash(rebuilt), CATALOG_HASH)
        self.assertEqual(canonical_hash(receipt["retained_v1_provenance"]),
                         "947d065ecafacd67e1b52ee6fee2e86f8dab3ef92dd8133ac98d437049bdb9cb")
        self.assertFalse(receipt["migration"]["raw_science_rederived"])
        # RPT002 저항이 초기보다 낮은 원래 값을 임의 단조화하지 않는다.
        self.assertLess(EXPECTED["cmu_vah01_rpt002"][1], EXPECTED["cmu_vah01_rpt000"][1])

    def test_migrated_runtime_file_hashes(self):
        receipt = json.loads((PROFILES / "provenance.json").read_text(encoding="utf-8"))
        for name, expected_hash in receipt["migration"]["runtime_file_sha256"].items():
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((PROFILES / name).read_bytes()).hexdigest(),
                                 expected_hash)

    def test_explicit_path_and_custom_profile_directory(self):
        path = PROFILES / "cmu_vah01_rpt002.json"
        self.assertEqual(load_profile(path), load_profile("cmu_vah01_rpt002", PROFILES))
        with tempfile.TemporaryDirectory() as folder:
            profile = synthetic_profile()
            path = Path(folder) / "synthetic_test_only.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            self.assertEqual(load_profile("synthetic_test_only", folder), profile)
            profile["id"] = "a_different_id"
            path.write_text(json.dumps(profile), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_profile("synthetic_test_only", folder)

    def test_missing_and_malformed_profiles_do_not_fall_back(self):
        with self.assertRaises(FileNotFoundError):
            load_profile("does_not_exist")
        for name in ("", " cmu_vah01_rpt000", "cmu_vah01_rpt000 ", "한글ID"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                load_profile(name)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bad.json"
            for text in ("{bad JSON", "[]", "{}"):
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaises(ValueError):
                    load_profile(path)

    def test_invalid_physical_parameters_and_curves_are_rejected(self):
        cases = [
            ("capacity_ah", 0), ("capacity_ah", -1), ("capacity_ah", 1e308),
            ("capacity_ah", math.nan), ("pack_resistance_ohm", 0),
            ("pack_resistance_ohm", -1), ("pack_resistance_ohm", math.inf),
            ("series_cells", 0), ("series_cells", 4.5), ("series_cells", True),
            ("parallel_cells", -1), ("id", "invalid id"),
            ("initial_soc", 1.1), ("schema", "unknown"),
            ("ocv_soc", [0, 0, 1]), ("ocv_soc", [0, 0.8, 0.5]),
            ("ocv_soc", [0.1, 0.5, 1]), ("ocv_soc", [0, 0.5, 0.9]),
            ("ocv_soc", [0, math.nan, 1]), ("ocv_soc", [0, 1]),
            ("ocv_cell_v", [3, 0, 4]), ("ocv_cell_v", [3, math.inf, 4]),
            ("ocv_cell_v", None),
        ]
        for key, value in cases:
            profile = synthetic_profile()
            profile[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                Battery(profile)
        profile = synthetic_profile()
        del profile["capacity_ah"]
        with self.assertRaises(ValueError):
            Battery(profile)


class LoadTests(unittest.TestCase):
    def test_source_bench_table_and_command_limit_are_preserved(self):
        config = load_config()
        self.assertEqual(config["throttle"], BENCH_THROTTLE)
        self.assertEqual(config["motor_current_a"], BENCH_AMPS)
        self.assertEqual(config["bench_voltage_v"], 16.0)
        self.assertEqual(config["command_max_rad_s"], 1000.0)
        self.assertEqual(config["auxiliary_current_a"], 0.5)
        self.assertEqual(config["max_gap_s"], 0.5)

    def test_all_bench_knots_and_above_limit_saturation(self):
        battery = Battery(synthetic_profile())
        for throttle, amps in zip(BENCH_THROTTLE, BENCH_AMPS):
            with self.subTest(throttle=throttle):
                sample = battery.update([throttle * 1000] * 4, 0.0)
                self.assertAlmostEqual(sample["current_a"], 4 * amps + 0.5)
        sample = battery.update([2000.0] * 4, 0.0)
        self.assertAlmostEqual(sample["current_a"], 4 * BENCH_AMPS[-1] + 0.5)

    def test_linear_interpolation_quadratic_low_range_and_individual_sum(self):
        battery = Battery(synthetic_profile())
        sample = battery.update([0.0, 150.0, 325.0, 500.0], 0.0)
        expected = 0.5 + 1.44 * (0.15 / 0.30) ** 2 + (1.44 + 1.87) / 2 + 3.60
        self.assertAlmostEqual(sample["current_a"], expected)

    def test_fresh_zero_commands_mean_auxiliary_load_not_missing_data(self):
        sample = Battery(synthetic_profile()).update([0.0] * 4, 0.0)
        self.assertEqual(sample["current_a"], 0.5)

    def test_ocv_knots_endpoints_and_midpoints(self):
        config = load_config()
        config["auxiliary_current_a"] = 0.0
        for soc, cell_v in ((0, 3.0), (0.25, (3.0 + 3.7) / 2),
                            (0.5, 3.7), (0.75, (3.7 + 4.2) / 2), (1, 4.2)):
            with self.subTest(soc=soc):
                battery = Battery(synthetic_profile(), soc, config)
                self.assertAlmostEqual(battery.update([0.0] * 4, 0.0)["voltage_v"], 4 * cell_v)

    def test_terminal_voltage_uses_pack_resistance(self):
        sample = Battery(synthetic_profile(), 0.5).update([500.0] * 4, 0.0)
        self.assertAlmostEqual(sample["voltage_v"], 4 * 3.7 - (4 * 3.60 + 0.5) * 0.05)

    def test_invalid_load_configuration_does_not_use_hidden_defaults(self):
        for key, value in (
            ("command_max_rad_s", 0), ("command_max_rad_s", math.inf),
            ("auxiliary_current_a", -1), ("auxiliary_current_a", math.nan),
            ("max_gap_s", 0), ("max_gap_s", -1), ("bench_voltage_v", 0),
            ("throttle", [0.0, 1.0]), ("throttle", [0.3, 0.3]),
            ("motor_current_a", [1.0]), ("schema", "unknown"),
        ):
            config = load_config()
            config[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                Battery(synthetic_profile(), load_config=config)
        with self.assertRaises(ValueError):
            Battery(synthetic_profile(), load_config={})


class StateTests(unittest.TestCase):
    def test_first_sample_establishes_origin_without_backfill(self):
        sample = Battery(synthetic_profile()).update([500.0] * 4, 1000.0)
        self.assertEqual(set(sample), {"voltage_v", "current_a", "soc", "consumed_mah", "sim_time_s"})
        self.assertEqual(sample["consumed_mah"], 0.0)
        self.assertEqual(sample["soc"], 0.95)
        self.assertEqual(sample["sim_time_s"], 1000.0)

    def test_integration_uses_previous_demand_only_over_bounded_interval(self):
        battery = Battery(synthetic_profile(), 0.8)
        first = battery.update([500.0] * 4, 10.0)
        sample = battery.update([0.0] * 4, 10.5)
        expected_mah = first["current_a"] * 0.5 / 3.6
        self.assertAlmostEqual(sample["consumed_mah"], expected_mah)
        self.assertAlmostEqual(sample["soc"], 0.8 - expected_mah / 2000.0)
        self.assertEqual(sample["current_a"], 0.5)
        later = battery.update([0.0] * 4, 10.75)
        self.assertAlmostEqual(later["consumed_mah"], expected_mah + 0.5 * 0.25 / 3.6)
        self.assertLess(later["soc"], sample["soc"])

    def test_equal_time_pause_does_not_integrate_but_updates_demand(self):
        battery = Battery(synthetic_profile())
        first = battery.update([0.0] * 4, 1.0)
        paused = battery.update([500.0] * 4, 1.0)
        self.assertEqual(paused["consumed_mah"], first["consumed_mah"])
        self.assertEqual(paused["soc"], first["soc"])
        self.assertGreater(paused["current_a"], first["current_a"])
        resumed = battery.update([500.0] * 4, 1.25)
        self.assertAlmostEqual(resumed["consumed_mah"], paused["current_a"] * 0.25 / 3.6)

    def test_significant_gap_invalidates_run_without_normal_sample(self):
        battery = Battery(synthetic_profile())
        battery.update([500.0] * 4, 0.0)
        with self.assertRaisesRegex(ValueError, "gap"):
            battery.update([500.0] * 4, 0.500001)
        with self.assertRaisesRegex(ValueError, "new run"):
            battery.update([0.0] * 4, 0.5)

    def test_backward_clock_is_error_not_automatic_reset(self):
        battery = Battery(synthetic_profile())
        battery.update([500.0] * 4, 10.0)
        with self.assertRaisesRegex(ValueError, "backward"):
            battery.update([500.0] * 4, 9.0)
        with self.assertRaisesRegex(ValueError, "new run"):
            battery.update([500.0] * 4, 10.0)

    def test_missing_negative_and_nonfinite_commands_are_terminal_errors(self):
        invalid = [None, [], [0.0] * 3, [0.0] * 5, "0000",
                   [-1.0, 0, 0, 0], [math.nan, 0, 0, 0], [math.inf, 0, 0, 0],
                   [None, 0, 0, 0], [True, 0, 0, 0], ["500", 0, 0, 0]]
        for commands in invalid:
            battery = Battery(synthetic_profile())
            battery.update([500.0] * 4, 0.0)
            with self.subTest(commands=commands):
                with self.assertRaises(ValueError):
                    battery.update(commands, 0.1)
                with self.assertRaisesRegex(ValueError, "new run"):
                    battery.update([500.0] * 4, 0.2)

    def test_invalid_simulation_times_and_initial_soc_are_rejected(self):
        for value in (None, -1, math.nan, math.inf, True, "0"):
            with self.subTest(time=value), self.assertRaises(ValueError):
                Battery(synthetic_profile()).update([0.0] * 4, value)
        for value in (None, -0.1, 1.1, math.nan, math.inf, True):
            with self.subTest(soc=value), self.assertRaises(ValueError):
                Battery(synthetic_profile(), value)

    def test_consumption_is_bounded_by_initial_available_charge(self):
        profile = synthetic_profile()
        profile["capacity_ah"] = 0.0001
        battery = Battery(profile, initial_soc=0.1)
        battery.update([1000.0] * 4, 0.0)
        empty = battery.update([1000.0] * 4, 0.5)
        self.assertEqual(empty["soc"], 0.0)
        self.assertAlmostEqual(empty["consumed_mah"], 0.1 * 0.0001 * 1000)
        self.assertEqual(battery.update([1000.0] * 4, 1.0)["consumed_mah"], empty["consumed_mah"])

    def test_input_mutation_and_returned_sample_mutation_cannot_reset_model(self):
        profile, config = synthetic_profile(), load_config()
        battery = Battery(profile, load_config=config)
        initial = battery.update([500.0] * 4, 0.0)
        expected_current, expected_voltage = initial["current_a"], initial["voltage_v"]
        profile["capacity_ah"] = 1000.0
        profile["ocv_cell_v"][1] = 100.0
        config["motor_current_a"][:] = [100.0] * len(BENCH_AMPS)
        config["auxiliary_current_a"] = 100.0
        initial["consumed_mah"] = 99999.0
        repeated = battery.update([500.0] * 4, 0.0)
        self.assertEqual(repeated["current_a"], expected_current)
        self.assertEqual(repeated["voltage_v"], expected_voltage)
        later = battery.update([500.0] * 4, 0.5)
        self.assertAlmostEqual(later["soc"], 0.95 - (expected_current * 0.5 / 3.6) / 2000)
        self.assertFalse(hasattr(battery, "reset"))
        self.assertFalse(hasattr(battery, "select_profile"))
        with self.assertRaises(AttributeError):
            battery.profile_id = "another_profile"

    def test_nonpositive_voltage_is_an_error_not_a_wire_floor(self):
        profile = synthetic_profile()
        profile["pack_resistance_ohm"] = 100.0
        battery = Battery(profile)
        with self.assertRaisesRegex(ValueError, "voltage"):
            battery.update([1000.0] * 4, 0.0)
        with self.assertRaisesRegex(ValueError, "new run"):
            battery.update([0.0] * 4, 0.0)


if __name__ == "__main__":
    unittest.main()
