"""작은 합성 프로토콜로 추출기를 시험하며 실측 자료인 것처럼 사용하지 않는다."""
import importlib.util
import math
from pathlib import Path
import unittest
import csv
import hashlib
import json
import os
import tempfile

MODULE = Path(__file__).resolve().parents[1] / 'tools/battery_profiles/cmu_aging.py'


def subject():
    """추출기 모듈을 읽는다. 단위 없음, 전역 실행·파일 쓰기 없이 로드한다."""
    if not MODULE.is_file():
        raise AssertionError('CMU aging extractor is missing')
    spec = importlib.util.spec_from_file_location('cmu_aging_under_test', MODULE)
    if spec is None or spec.loader is None:
        raise AssertionError('Cannot load extractor module')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fixture_rpt(offset=0.0, last_duration=7200):
    """초·V·mA 단위의 합성 RPT를 반환한다. 원본·외부 상태를 바꾸지 않는다."""
    rows = []
    time = offset
    q = 0.0
    specs = [(0, 3600, 1500), (1, 1800, 200), (2, 880, 0),
             (3, 3600, -600), (4, 30, -60), (5, 7200, -600),
             (6, 30, -60), (7, last_duration, -600), (8, 890, 0)]
    for ns, duration, milliamps in specs:
        ticks = list(range(0, duration+1, 10 if ns in (0, 1, 2, 8) else 1))
        for tick in ticks:
            dt = 0 if not rows else time+tick-rows[-1]['time_s']
            if ns in (3, 4, 5, 6, 7):
                q += (-milliamps)*dt/3600
            if ns <= 2:
                voltage = 4.19
            elif ns == 8:
                voltage = 2.8
            else:
                capacity = (3600+7200+last_duration)*600/3600 + 60*60/3600
                voltage = 4.19-1.69*q/capacity
            rows.append({'time_s': time+tick, 'Ecell_V': voltage, 'I_mA': milliamps,
                         'Temperature__C': 25.0, 'QDischarge_mA_h': q,
                         'cycleNumber': 0, 'Ns': ns})
        time += duration+0.002
    return rows


class RptSelectionTests(unittest.TestCase):
    """연속 프로토콜과 용량 추출을 검증한다. 파일·네트워크 상태는 바꾸지 않는다."""

    def test_rejects_incomplete_or_physically_wrong_protocol(self):
        """누락·충전·이상 시간·끝전압을 거부한다. 초/V/mA 입력, 상태 변경 없음."""
        module = subject()
        original = fixture_rpt()
        mutations = [('missing end', [r for r in original if r['Ns'] != 8])]
        for name in ('charge', 'gap', 'cutoff', 'nonfinite', 'duration', 'counter reset'):
            rows = [dict(r) for r in original]
            selected = [r for r in rows if r['Ns'] == 7]
            if name == 'charge':
                selected[20]['I_mA'] = 600
            elif name == 'gap':
                selected[20]['time_s'] += 100
            elif name == 'cutoff':
                selected[-1]['Ecell_V'] = 3.2
            elif name == 'nonfinite':
                selected[20]['Temperature__C'] = math.nan
            elif name == 'duration':
                rows = [r for r in rows if not (r['Ns'] == 4 and r['time_s'] > next(x['time_s'] for x in rows if x['Ns'] == 4)+10)]
            else:
                selected[20]['QDischarge_mA_h'] = 0
            mutations.append((name, rows))
        for name, rows in mutations:
            with self.subTest(name=name), self.assertRaises(ValueError):
                module.extract_rpts(iter(rows))

    def test_finds_distinct_rpts_with_reset_cycle_numbers(self):
        """같은 tester counter의 별도 RPT를 구별한다. 단위 초, 상태 효과 없음."""
        first = fixture_rpt()
        second = fixture_rpt(first[-1]['time_s']+1, last_duration=6000)
        actual = subject().extract_rpts(iter(first+second))
        self.assertEqual(len(actual), 2)
        self.assertEqual([r['rpt_index'] for r in actual], [0, 1])
        self.assertEqual(actual[1]['source_row_start'], len(first))
        self.assertGreater(actual[0]['cell_capacity_ah'], actual[1]['cell_capacity_ah'])
        self.assertEqual([r['tester_cycle_number'] for r in actual], [0, 0])
        self.assertAlmostEqual(actual[0]['soh_percent'], 100)


class ResistanceTests(unittest.TestCase):
    """두 DC 완화 step의 등가 저항 회귀를 검증한다. 외부 상태 변경 없음."""

    def test_uses_measured_current_step_and_reports_fit_residual(self):
        """V/A 기울기와 V 잔차를 추정한다. 합성 자료이며 입력을 변경하지 않는다."""
        module = subject()
        self.assertTrue(hasattr(module, 'estimate_resistance'), 'DC step extraction is missing')
        blocks = [{'rows': []} for _ in range(9)]
        for number, resistance in ((4, 0.025), (6, 0.035)):
            before = {'time_s': 0., 'I_mA': -600., 'Ecell_V': 3.8,
                      'QDischarge_mA_h': 600., 'source_row': number*100}
            blocks[number-1]['rows'] = [before]
            blocks[number]['rows'] = [dict(before, time_s=t, I_mA=-60.,
                Ecell_V=3.8+(0.6-0.06)*resistance, source_row=number*100+k+1)
                for k, t in enumerate((0.002, 1., 30.))]
        fitted = module.estimate_resistance({'_blocks': blocks, 'cell_capacity_ah': 3.})
        self.assertAlmostEqual(fitted['cell_resistance_ohm'], 0.03)
        self.assertAlmostEqual(fitted['fit_rmse_v'], (0.6-0.06)*0.005)
        self.assertEqual(fitted['method'], 'effective_DC_1s_step_estimate')
        self.assertEqual(len(fitted['steps']), 2)
        blocks[4]['rows'][1]['I_mA'] = -600.
        with self.assertRaises(ValueError):
            module.estimate_resistance({'_blocks': blocks, 'cell_capacity_ah': 3.})


class CurveTests(unittest.TestCase):
    """OCV 등가곡선의 전하축·전압 보정을 검증한다. 원본 변경 없음."""

    def test_builds_soc_curve_from_each_reference_not_scalar_replay(self):
        """Ah 기반 SOC와 I·R 보정 V를 확인한다. 합성 입력·상태 효과 없음."""
        module = subject()
        self.assertTrue(hasattr(module, 'build_curve'), 'Equivalent OCV extraction is missing')
        rpt = module.extract_rpts(iter(fixture_rpt()))[0]
        q0 = rpt['_blocks'][3]['rows'][0]['QDischarge_mA_h']
        resistance = 0.03
        for block in rpt['_blocks'][3:8]:
            for row in block['rows']:
                soc = 1-(row['QDischarge_mA_h']-q0)/1000/rpt['cell_capacity_ah']
                row['Ecell_V'] = 2.5+1.69*soc+row['I_mA']/1000*resistance
        curve = module.build_curve(rpt, resistance)
        self.assertEqual(curve['ocv_soc'][0], 0)
        self.assertEqual(curve['ocv_soc'][-1], 1)
        self.assertEqual(len(curve['ocv_soc']), 101)
        for soc, voltage in zip(curve['ocv_soc'], curve['ocv_cell_v']):
            self.assertAlmostEqual(voltage, 2.5+1.69*soc)
        self.assertAlmostEqual(curve['diagnostics']['reconstruction_rmse_v'], 0)
        self.assertEqual(curve['diagnostics']['method'], 'C5_discharge_plus_I_R_OCV_equivalent')
        self.assertGreater(curve['diagnostics']['excluded_relaxation_and_settling_rows'], 0)


class StageSelectionTests(unittest.TestCase):
    """실측에 가장 가까운 네 단계 선택을 검증한다. 외부 상태 변경 없음."""

    def test_requires_four_distinct_measured_nearest_stages(self):
        """SOH %에서 가까운 실제 RPT를 고른다. 목표 밖 합성·복제는 거부한다."""
        module = subject()
        self.assertTrue(hasattr(module, 'select_stages'), 'Measured stage selection is missing')
        rpts = [{'rpt_index': i, 'soh_percent': s} for i, s in enumerate((100., 97., 95.03, 90.42, 84.81, 81.48))]
        result = module.select_stages(rpts)
        self.assertEqual([r['rpt_index'] for r in result], [0, 2, 3, 4])
        with self.assertRaises(ValueError):
            module.select_stages(rpts, targets=(100., 90., 80., 70.))
        with self.assertRaises(ValueError):
            module.select_stages(rpts, targets=(100., 100., 95., 90.))
        with self.assertRaises(ValueError):
            module.select_stages(rpts[:2])


@unittest.skipUnless(os.environ.get('CMU_AGING_RAW_DIR'), 'Set CMU_AGING_RAW_DIR for pinned real-data test')
class PublicationTests(unittest.TestCase):
    """선택한 실제 전체 파일로 발행을 확인한다. 부산물은 지정 data 폴더 안에서만 생성한다."""

    def test_real_catalog_csv_and_source_checksums(self):
        """실제 Ah/Ω/곡선과 CSV 대응·부분 원본 거부를 확인한다. 임시 결과만 쓰고 정리한다."""
        module = subject()
        self.assertTrue(hasattr(module, 'generate'), 'Verified catalog publication is missing')
        raw = Path(os.environ['CMU_AGING_RAW_DIR'])
        with tempfile.TemporaryDirectory(prefix='unit-catalog-', dir=raw.parent) as directory:
            root = Path(directory)
            target = root/'catalog'
            catalog = module.generate(raw, target)
            saved = json.loads((target/'catalog.json').read_text())
            self.assertEqual(saved, catalog)
            self.assertEqual(saved['schema'], 'arion.cmu_battery_profiles.v1')
            self.assertEqual((saved['series_cells'], saved['parallel_cells']), (4, 2))
            self.assertEqual(len(saved['profiles']), 4)
            self.assertEqual(saved['default_profile_id'], 'cmu_vah01_rpt000')
            self.assertEqual([p['id'] for p in saved['profiles']],
                             ['cmu_vah01_rpt000', 'cmu_vah01_rpt002', 'cmu_vah01_rpt006', 'cmu_vah01_rpt013'])
            for profile in saved['profiles']:
                self.assertEqual(set(profile), {'id', 'label_ko', 'capacity_ah', 'pack_resistance_ohm',
                    'initial_soc', 'ocv_soc', 'ocv_cell_v', 'source', 'soh_percent'})
                self.assertAlmostEqual(profile['capacity_ah'], 2*profile['source']['selected_rpt']['cell_capacity_ah'])
                self.assertAlmostEqual(profile['pack_resistance_ohm'], 2*profile['source']['resistance']['cell_resistance_ohm'])
                with (target/profile['source']['curve_csv']).open() as stream:
                    curve = list(csv.DictReader(stream))
                self.assertEqual([float(r['soc']) for r in curve], profile['ocv_soc'])
                self.assertEqual([float(r['ocv_cell_v']) for r in curve], profile['ocv_cell_v'])
            proof = json.loads((target/'provenance.json').read_text())
            self.assertEqual(len(proof['rpt_inventory']), 18)
            for name, expected in proof['artifact_sha256'].items():
                self.assertEqual(hashlib.sha256((target/name).read_bytes()).hexdigest(), expected)
            with self.assertRaises(FileExistsError):
                module.generate(raw, target)
            partial = root/'partial'
            partial.mkdir()
            for source in raw.iterdir():
                if source.name in ('cmu-v3.json', 'README.txt', 'article.xml', 'VAH01_impedance.csv'):
                    (partial/source.name).write_bytes(source.read_bytes())
            with (raw/'VAH01.csv').open('rb') as stream:
                (partial/'VAH01.csv').write_bytes(stream.read(1024))
            with self.assertRaisesRegex(ValueError, 'checksum|size|complete'):
                module.generate(partial, root/'must-not-exist')
            self.assertFalse((root/'must-not-exist').exists())


class CommandLineTests(unittest.TestCase):
    """재생성 CLI 인수를 검증한다. --help/오류 경로라 파일 쓰기·네트워크 실행은 없다."""

    def test_cli_requires_explicit_input_and_output_directories(self):
        """경로를 명시해야 실행되도록 검사한다. 단위 없음, 하위 Python만 실행한다."""
        import subprocess
        import sys
        missing = subprocess.run([sys.executable, '-B', str(MODULE)], capture_output=True, text=True)
        self.assertNotEqual(missing.returncode, 0, 'Missing generation CLI')
        help_result = subprocess.run([sys.executable, '-B', str(MODULE), '--help'], capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0)
        self.assertIn('--raw-dir', help_result.stdout)
        self.assertIn('--output-dir', help_result.stdout)


if __name__ == '__main__':
    unittest.main()
