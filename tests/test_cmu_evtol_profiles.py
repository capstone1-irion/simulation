"""Synthetic protocol fixtures; NOT downloaded CMU measurements."""
import importlib.util
import os
from pathlib import Path
import unittest

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools/battery_profiles/cmu_evtol.py"


def module():
    spec = importlib.util.spec_from_file_location("cmu_evtol_tested", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("Profile module cannot be loaded")
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def initial_rows():
    rows = []
    origin = 0.0
    protocol = [0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 3, 4, 5, 6, 7, 0]
    for group, segment in enumerate(protocol):
        duration = {12: 75.0, 13: 800.0, 14: 105.0}.get(group, 30.0)
        discharge = group in (3, 4, 5, 6, 7, 12, 13, 14)
        for n in range(3):
            row = {key: 0.0 for key in ("EnergyCharge_W_h", "QCharge_mA_h", "EnergyDischarge_W_h", "QDischarge_mA_h")}
            row.update(time_s=origin + n * duration / 2, Ecell_V=4.2 - group * 0.01,
                       I_mA=-600.0 if discharge else 1000.0, Temperature__C=25.0,
                       cycleNumber=1 if group == 16 else 0, Ns=segment)
            rows.append(row)
        origin += duration + 0.002
    return rows


def synthetic_publication(m, complete=False):
    """TEST-only byte pins; schema labels are not CMU measurement provenance."""
    import csv
    import hashlib
    import io
    import tempfile
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=m.SOURCE_FIELDS)
    writer.writeheader()
    writer.writerows(initial_rows())
    data = stream.getvalue().encode("utf-8")
    digest = hashlib.sha256(data).hexdigest()
    md5 = hashlib.md5(data).hexdigest()
    # Only this imported test module is changed, never production source pins.
    m.SOURCE_PINS = {cell: (len(data) + (1 if complete else 0), digest,
                            len(data) + (0 if complete else 1), md5)
                     for cell in m.SPLITS}
    profiles = []
    with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
        for cell in m.SPLITS:
            path = Path(d) / (cell + ".csv")
            path.write_bytes(data)
            rows, provenance = m.read_source(path, cell)
            provenance["test_fixture"] = "synthetic_protocol_NOT_CMU_measurements"
            for p in m.build_profiles(cell, rows):
                p["metadata"]["test_fixture"] = "synthetic_protocol_NOT_CMU_measurements"
                p["metadata"]["source_integrity"] = dict(provenance)
                profiles.append(p)
    return profiles


def assert_bundle_metadata_rejected(test, keys, value, index=0):
    """Change only one field in a fresh valid synthetic bundle and refresh hash."""
    import copy
    import hashlib
    import json
    import tempfile
    m = module()
    profiles = synthetic_publication(m)
    with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
        output = Path(d) / "healthy"
        manifest = m.write_bundle(profiles, output)
        test.assertTrue(m.verify_bundle(output)["passed"])
        changed = copy.deepcopy(profiles)
        target = changed[index]["metadata"]
        for key in keys[:-1]:
            target = target[key]
        target[keys[-1]] = value
        rejected_output = Path(d) / "rejected"
        with test.assertRaises(ValueError):
            m.write_bundle(changed, rejected_output)
        test.assertFalse(rejected_output.exists())
        metadata_path = output / manifest["profiles"][index]["metadata"]
        metadata_path.write_text(json.dumps(changed[index]["metadata"], allow_nan=False))
        manifest["profiles"][index]["metadata_sha256"] = hashlib.sha256(metadata_path.read_bytes()).hexdigest()
        (output / "manifest.json").write_text(json.dumps(manifest))
        with test.assertRaises(ValueError):
            m.verify_bundle(output)


class InitialSelectionTests(unittest.TestCase):
    def test_only_initial_reference_and_first_complete_mission_are_selected(self):
        self.assertTrue(MODULE_PATH.is_file(), "CMU initial profile implementation is missing")
        profiles = module().build_profiles("VAH01", initial_rows())
        self.assertEqual([p["metadata"]["kind"] for p in profiles], ["initial_reference_discharge", "first_mission_discharge"])
        self.assertEqual([len(p["samples"]) for p in profiles], [15, 9])
        self.assertTrue(all(p["metadata"]["split"] == "train" for p in profiles))
        self.assertTrue(all(p["metadata"]["soh_percent"] == 100.0 for p in profiles))
        self.assertTrue(all(p["metadata"]["absolute_soh_certified"] is False for p in profiles))
        self.assertEqual(profiles[0]["samples"][0]["current_a"], 0.6)
        self.assertEqual(profiles[0]["samples"][0]["elapsed_s"], 0.0)
        self.assertEqual(profiles[1]["metadata"]["mission_phase_order"], ["takeoff", "cruise", "landing"])
        self.assertEqual(profiles[0]["metadata"]["baseline_capacity_ah"], profiles[1]["metadata"]["baseline_capacity_ah"])
        self.assertEqual(set(profiles[0]["samples"][0]), {"elapsed_s", "voltage_v", "current_a", "temperature_c"})


    def test_publication_requires_exact_six_profile_members(self):
        import copy
        import tempfile
        m = module()
        complete = synthetic_publication(m)
        collections = {
            "one_reference": complete[:1],
            "missing_mission": complete[:-1],
            "duplicate": complete + [complete[0]],
            "unexpected": complete + [dict(complete[0], id="VAH09_initial_reference_discharge")],
        }
        for mutation, profiles in collections.items():
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
                output = Path(d) / "bundle"
                with self.assertRaises(ValueError):
                    m.write_bundle(copy.deepcopy(profiles), output)
                self.assertFalse(output.exists())

    def test_verifier_requires_exact_members_and_split_completeness(self):
        import copy
        import json
        import tempfile
        m = module()
        complete = synthetic_publication(m)
        for mutation in ("one_reference", "missing_mission", "duplicate", "unexpected", "wrong_split"):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
                output = Path(d) / "bundle"
                manifest = m.write_bundle(copy.deepcopy(complete), output)
                if mutation == "one_reference":
                    manifest["profiles"] = manifest["profiles"][:1]
                elif mutation == "missing_mission":
                    manifest["profiles"] = manifest["profiles"][:-1]
                elif mutation == "duplicate":
                    manifest["profiles"].append(manifest["profiles"][0])
                elif mutation == "unexpected":
                    manifest["profiles"][0]["id"] = "VAH09_initial_reference_discharge"
                manifest["profile_count"] = len(manifest["profiles"])
                manifest["splits"] = {split: [e["id"] for e in manifest["profiles"] if e["id"].startswith(cell + "_")]
                                      for cell, split in m.SPLITS.items()}
                if mutation == "wrong_split":
                    manifest["splits"]["validation"].append(manifest["splits"]["train"].pop())
                (output / "manifest.json").write_text(json.dumps(manifest))
                with self.assertRaises(ValueError):
                    m.verify_bundle(output)

    def test_unverified_protocol_is_rejected(self):
        rows = initial_rows()
        rows[9]["Ns"] = 99
        with self.assertRaisesRegex(ValueError, "initial protocol"):
            module().build_profiles("VAH01", rows)

    def test_missing_post_mission_witness_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "complete initial"):
            module().build_profiles("VAH01", initial_rows()[:45])

    def test_first_mission_duration_is_not_silently_truncated(self):
        rows = initial_rows()
        rows[44]["time_s"] -= 10
        with self.assertRaisesRegex(ValueError, "mission phase duration"):
            module().build_profiles("VAH01", rows)

    def test_backward_selected_time_is_rejected(self):
        rows = initial_rows()
        rows[10]["time_s"] = rows[9]["time_s"]
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            module().build_profiles("VAH01", rows)

    def test_source_charge_is_not_turned_into_drone_discharge(self):
        rows = initial_rows()
        rows[10]["I_mA"] = 600
        with self.assertRaisesRegex(ValueError, "physical measurement"):
            module().build_profiles("VAH01", rows)

    def test_nonfinite_temperature_is_rejected(self):
        rows = initial_rows()
        rows[10]["Temperature__C"] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            module().build_profiles("VAH01", rows)

    def test_all_source_cell_split_assignments(self):
        m = module()
        self.assertEqual({cell: m.build_profiles(cell, initial_rows())[0]["metadata"]["split"] for cell in m.SPLITS},
                         {"VAH01": "train", "VAH17": "validation", "VAH27": "test_normal"})
        with self.assertRaises(ValueError):
            m.build_profiles("VAH09", initial_rows())

    def test_verified_source_reader_pins_exact_bytes(self):
        import tempfile
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            source = Path(d) / "VAH01.csv.initial-prefix.csv"
            source.write_text("unverified bytes")
            m = module()
            self.assertTrue(hasattr(m, "read_source"), "Source integrity reader is missing")
            with self.assertRaisesRegex(ValueError, "checksum"):
                m.read_source(source, "VAH01")

    def test_export_reload_integrity_and_no_overwrite(self):
        import tempfile
        m = module()
        self.assertTrue(hasattr(m, "write_bundle"), "CMU profile bundle writer is missing")
        ps = synthetic_publication(m)
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            out = Path(d) / "bundle"
            m.write_bundle(ps, out)
            result = m.verify_bundle(out)
            self.assertTrue(result["passed"])
            self.assertEqual(result["profile_count"], 6)
            self.assertEqual(result["sample_count"], 72)
            self.assertEqual(m.load_profile(out, ps[0]["id"]), ps[0])
            with self.assertRaises(FileExistsError):
                m.write_bundle(ps, out)
            (out / (ps[0]["id"] + ".csv")).write_text("corruption")
            with self.assertRaisesRegex(ValueError, "checksum"):
                m.verify_bundle(out)

    def test_unsafe_profile_id_is_rejected_before_output_creation(self):
        import tempfile
        m = module()
        self.assertTrue(hasattr(m, "write_bundle"), "CMU profile bundle writer is missing")
        ps = m.build_profiles("VAH01", initial_rows())
        ps[0]["id"] = "../escaped"
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            out = Path(d) / "bundle"
            with self.assertRaises(ValueError):
                m.write_bundle(ps, out)
            self.assertFalse(out.exists())

    def test_hidden_observable_and_cell_split_tampering_are_rejected(self):
        m = module()
        self.assertTrue(hasattr(m, "validate_profile"), "Profile contract validator is missing")
        p = m.build_profiles("VAH01", initial_rows())[0]
        p["samples"][0]["soh_percent"] = 100.0
        with self.assertRaisesRegex(ValueError, "feature"):
            m.validate_profile(p)
        p = m.build_profiles("VAH01", initial_rows())[0]
        p["metadata"]["split"] = "test_normal"
        with self.assertRaisesRegex(ValueError, "split"):
            m.validate_profile(p)

    def test_causal_replay_end_and_reset(self):
        m = module()
        self.assertTrue(hasattr(m, "ProfileReplay"), "CMU causal replay is missing")
        p = m.build_profiles("VAH01", initial_rows())[1]
        replay = m.ProfileReplay(p)
        first, second = p["samples"][:2]
        self.assertEqual(replay.sample(first["elapsed_s"]), first)
        self.assertEqual(replay.sample((first["elapsed_s"] + second["elapsed_s"]) / 2), first)
        self.assertEqual(replay.sample(second["elapsed_s"]), second)
        with self.assertRaises(ValueError):
            replay.sample(0)
        self.assertIsNone(replay.sample(p["metadata"]["duration_s"] + 1))
        replay.reset()
        self.assertEqual(replay.sample(0), first)
        with self.assertRaises(ValueError):
            replay.sample(float("nan"))

    def test_cli_generate_verify_and_bad_source_fail_closed(self):
        import subprocess
        import tempfile
        m = module()
        self.assertTrue(hasattr(m, "main"), "CMU command-line entry is missing")
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            root = Path(d)
            result = subprocess.run([__import__("sys").executable, "-B", str(MODULE_PATH), "generate", "--sources", d, "--output", str(root / "out")], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((root / "out").exists())

    def test_optimized_python_does_not_remove_source_integrity_check(self):
        import subprocess
        import tempfile
        m = module()
        self.assertTrue(hasattr(m, "main"), "CMU command-line entry is missing")
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            root = Path(d)
            (root / "VAH01.csv.initial-prefix.csv").write_text("invalid source")
            r = subprocess.run([__import__("sys").executable, "-B", "-O", str(MODULE_PATH), "generate", "--sources", d, "--output", str(root / "out")], capture_output=True, text=True, timeout=10)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("checksum", r.stderr)
            self.assertFalse((root / "out").exists())

    def test_fractional_tester_protocol_marker_is_not_coerced(self):
        rows = initial_rows()
        rows[9]["Ns"] = 3.25
        with self.assertRaisesRegex(ValueError, "integer protocol"):
            module().build_profiles("VAH01", rows)

class ProvenanceContractTests(unittest.TestCase):
    def test_publication_and_verification_require_source_provenance(self):
        import copy
        import hashlib
        import json
        import tempfile
        m = module()
        profiles = synthetic_publication(m)
        without = copy.deepcopy(profiles)
        del without[0]["metadata"]["source_integrity"]
        # Building and individual causal replay remain usable before provenance attachment.
        m.validate_profile(without[0])
        self.assertEqual(m.ProfileReplay(without[0]).sample(0), without[0]["samples"][0])
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            output = Path(d) / "healthy"
            manifest = m.write_bundle(profiles, output)
            rejected = Path(d) / "rejected"
            with self.assertRaises(ValueError):
                m.write_bundle(without, rejected)
            self.assertFalse(rejected.exists())
            path = output / manifest["profiles"][0]["metadata"]
            path.write_text(json.dumps(without[0]["metadata"]))
            manifest["profiles"][0]["metadata_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            (output / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                m.verify_bundle(output)

    def test_each_source_provenance_contradiction_is_rejected_independently(self):
        mutations = {
            "whole_file_md5_verified_locally": True,
            "source_sha256": "0" * 64,
            "source_download_scope": "complete_file",
            "source_file": "VAH17.csv",
            "source_bytes": 1,
            "original_file_bytes": 1,
            "original_repository_md5": "not-the-literal-pin",
            "source_dataset_doi": "10.1184/R1/14226830.v2",
            "licence": "unknown",
        }
        for field, value in mutations.items():
            with self.subTest(field=field):
                assert_bundle_metadata_rejected(self, ("source_integrity", field), value)
        for field in ("time_s", "Ecell_V", "I_mA", "Temperature__C"):
            with self.subTest(unit=field):
                assert_bundle_metadata_rejected(self, ("source_integrity", "source_units", field), "wrong-unit")

    def test_complete_source_scope_remains_supported(self):
        import tempfile
        m = module()
        profiles = synthetic_publication(m, complete=True)

        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            output = Path(d) / "bundle"
            m.write_bundle(profiles, output)
            self.assertTrue(m.verify_bundle(output)["passed"])


class SelectionContractTests(unittest.TestCase):
    def test_each_unit_interpolation_and_scope_contract_is_checked_independently(self):
        mutations = {"interpolation": "linear_upsampled", "baseline_capacity_time_basis": "sample_index",
                     "source_current_conversion": "abs(I_mA)", "electrical_scope": "scaled_pack",
                     "soc_status": "invented_soc", "runtime_provenance": "PX4_telemetry",
                     "mission_phase_order": ["landing", "cruise", "takeoff"]}
        for field, value in mutations.items():
            with self.subTest(field=field):
                assert_bundle_metadata_rejected(self, (field,), value)

    def test_required_span_counts_and_protocol_order_are_checked_independently(self):
        for index in (0, 1):
            spans = module().build_profiles("VAH01", initial_rows())[index]["metadata"]["source_step_spans"]
            for label, changed in (("empty", []), ("missing", spans[:-1]), ("duplicate", spans + [spans[-1]])):
                with self.subTest(kind=index, mutation=label):
                    assert_bundle_metadata_rejected(self, ("source_step_spans",), changed, index)
            for field, value in (("profile_row_start", 1), ("profile_row_count", 2),
                                 ("source_row_count", 2), ("source_row_start", -1),
                                 ("tester_cycle_number", 1), ("tester_segment", 99),
                                 ("source_row_count", 3.5)):
                with self.subTest(kind=index, span=0, field=field, value=value):
                    assert_bundle_metadata_rejected(self, ("source_step_spans", 0, field), value, index)

    def test_span_contiguous_coverage_and_time_alignment_are_checked_independently(self):
        for index in (0, 1):
            spans = module().build_profiles("VAH01", initial_rows())[index]["metadata"]["source_step_spans"]
            for field in ("profile_row_start", "profile_row_count", "source_row_start", "source_row_count",
                          "source_time_start_s", "source_time_end_s", "tester_segment"):
                with self.subTest(kind=index, span=1, field=field):
                    assert_bundle_metadata_rejected(self, ("source_step_spans", 1, field), spans[1][field] + 1, index)
            with self.subTest(kind=index, terminal="count"):
                assert_bundle_metadata_rejected(self, ("source_step_spans", len(spans) - 1, "profile_row_count"), 2, index)

    def test_mission_phase_label_order_is_checked(self):
        assert_bundle_metadata_rejected(self, ("mission_phase_order",), ["landing", "cruise", "takeoff"], 1)


class BaselineContractTests(unittest.TestCase):
    def test_each_initial_reference_capacity_is_recomputed_from_selected_rows(self):
        for index in (0, 2, 4):
            with self.subTest(profile=index):
                assert_bundle_metadata_rejected(self, ("baseline_capacity_ah",), 999.0, index)

    def test_each_mission_must_share_the_same_cell_reference_baseline(self):
        for index in (1, 3, 5):
            with self.subTest(profile=index):
                assert_bundle_metadata_rejected(self, ("baseline_capacity_ah",), 999.0, index)

    def test_same_cell_complete_source_digests_must_agree(self):
        import copy
        import hashlib
        import json
        import tempfile
        m = module()
        profiles = synthetic_publication(m, complete=True)
        changed = copy.deepcopy(profiles)
        changed[1]["metadata"]["source_integrity"]["source_sha256"] = "a" * 64
        m.validate_profile(changed[1])  # Valid scope/shape alone does not authenticate a full-file SHA.
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            output = Path(d) / "healthy"
            manifest = m.write_bundle(profiles, output)
            rejected = Path(d) / "rejected"
            with self.assertRaises(ValueError):
                m.write_bundle(changed, rejected)
            self.assertFalse(rejected.exists())
            path = output / manifest["profiles"][1]["metadata"]
            path.write_text(json.dumps(changed[1]["metadata"]))
            manifest["profiles"][1]["metadata_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            (output / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):
                m.verify_bundle(output)

    def test_mission_source_selection_must_follow_initial_reference(self):
        import copy
        m = module()
        reference, mission = m.build_profiles("VAH01", initial_rows())
        for axis in ("row", "time"):
            spans = copy.deepcopy(mission["metadata"]["source_step_spans"])
            if axis == "row":
                shift = spans[0]["source_row_start"] - reference["metadata"]["source_step_spans"][-1]["source_row_start"]
                for span in spans:
                    span["source_row_start"] -= shift
            else:
                shift = spans[0]["source_time_start_s"] - reference["metadata"]["source_step_spans"][-1]["source_time_end_s"]
                for span in spans:
                    span["source_time_start_s"] -= shift
                    span["source_time_end_s"] -= shift
            with self.subTest(axis=axis):
                assert_bundle_metadata_rejected(self, ("source_step_spans",), spans, 1)


class PublicationPreflightTests(unittest.TestCase):
    def test_all_metadata_is_serialized_strictly_before_any_output_creation(self):
        import copy
        import tempfile
        m = module()
        profiles = synthetic_publication(m)
        payloads = (("nan", float("nan")), ("infinity", float("inf")),
                    ("object", object()), ("set", {1, 2}), ("bytes", b"not-json"))
        for location in ("metadata_extra", "provenance_extra"):
            for label, payload in payloads:
                with self.subTest(location=location, payload=label), tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
                    changed = copy.deepcopy(profiles)
                    if location == "metadata_extra":
                        changed[-1]["metadata"]["extra"] = {"nested": [payload]}
                    else:
                        # Equal paired provenance reaches serialization, not the cell-pair check.
                        for p in changed[-2:]:
                            p["metadata"]["source_integrity"]["extra"] = {"nested": [payload]}
                    output = Path(d) / "new-parent" / "bundle"
                    with self.assertRaises((ValueError, TypeError)):
                        m.write_bundle(changed, output)
                    self.assertFalse(output.exists(), "Metadata failure left a partial bundle")
                    self.assertFalse(output.parent.exists(), "Metadata preflight created directories")

    def test_nonfinite_source_digest_is_rejected_before_output_creation(self):
        import tempfile
        m = module()
        profiles = synthetic_publication(m)
        profiles[0]["metadata"]["source_integrity"]["source_sha256"] = float("nan")
        with tempfile.TemporaryDirectory(dir=os.environ.get("ARION_BATTERY_TEST_ROOT")) as d:
            output = Path(d) / "bundle"
            with self.assertRaises(ValueError):
                m.write_bundle(profiles, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
