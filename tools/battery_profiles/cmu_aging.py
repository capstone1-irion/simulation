"""CMU eVTOL v3 실측 RPT로 부하 의존 모델용 등가 파라미터를 추출한다."""
import csv
from collections import deque
import itertools
import math
from pathlib import Path
import statistics
import argparse
import hashlib
import io
import json
import os
import tempfile
from bisect import bisect_left

API_URL = 'https://api.figshare.com/v2/articles/14226830/versions/3'
ARTICLE_URL = 'https://www.ebi.ac.uk/europepmc/webservices/rest/PMC10238407/fullTextXML'
TARGETS = (100.0, 95.0, 90.0, 85.0)
FILE_PINS = {
    'VAH01.csv': (26855147, 110884708, '5b0ac8963f1d161d4b74c731f8d8640a'),
    'VAH01_impedance.csv': (40183168, 1522, '02c687dbcfaa1ccc23c66f5ca51a9514'),
    'README.txt': (26855063, 5832, 'd65302bc58a41b5272594ae2f8766525'),
}
SOURCE_FIELDS = ['time_s', 'Ecell_V', 'I_mA', 'EnergyCharge_W_h', 'QCharge_mA_h',
                 'EnergyDischarge_W_h', 'QDischarge_mA_h', 'Temperature__C', 'cycleNumber', 'Ns']


def _interpolate(xs, ys, target):
    """오름차순 x에서 같은 단위의 y를 선형 보간한다. 범위 밖은 거부하며 상태 효과는 없다."""
    if len(xs) < 2 or len(xs) != len(ys) or not xs[0] <= target <= xs[-1]:
        raise ValueError('Interpolation outside measured coverage')
    position = bisect_left(xs, target)
    if position == 0:
        return ys[0]
    lo, hi = position-1, position
    return ys[lo]+(ys[hi]-ys[lo])*(target-xs[lo])/(xs[hi]-xs[lo])


def estimate_resistance(rpt):
    """두 RPT 완화 step의 1초 ΔV/ΔI를 공통 Ω로 적합한다. 잔차 V와 행 근거 반환, 상태 효과 없음."""
    steps = []
    for ns, nominal_dod in ((4, 0.2), (6, 0.6)):
        before = rpt['_blocks'][ns-1]['rows'][-1]
        after = rpt['_blocks'][ns]['rows']
        times = [r['time_s'] for r in after]
        target = before['time_s']+1
        volts = _interpolate(times, [r['Ecell_V'] for r in after], target)
        milliamps = _interpolate(times, [r['I_mA'] for r in after], target)
        delta_i = (milliamps-before['I_mA'])/1000
        delta_v = volts-before['Ecell_V']
        if not 0.4 < delta_i < 0.65 or not 0 < delta_v/delta_i < 0.2:
            raise ValueError('Unidentifiable DC relaxation current step')
        position = bisect_left(times, target)
        last = after[-1]
        steps.append({'nominal_dod': nominal_dod,
                      'capacity_relative_dod': before['QDischarge_mA_h']/1000/rpt['cell_capacity_ah'],
                      'pre_row': before['source_row'],
                      'one_second_bracket_rows': [after[max(0, position-1)]['source_row'], after[position]['source_row']],
                      'relaxation_row_end': last['source_row'], 'pre_time_s': before['time_s'],
                      'sample_target_time_s': target, 'delta_current_a': delta_i, 'delta_voltage_v': delta_v,
                      'one_second_ohm': delta_v/delta_i,
                      'last_relaxation_elapsed_s': last['time_s']-before['time_s'],
                      'last_relaxation_ohm': (last['Ecell_V']-before['Ecell_V'])/
                          ((last['I_mA']-before['I_mA'])/1000)})
    fitted = sum(s['delta_current_a']*s['delta_voltage_v'] for s in steps)/sum(s['delta_current_a']**2 for s in steps)
    for step in steps:
        step['fit_residual_v'] = step['delta_voltage_v']-fitted*step['delta_current_a']
    return {'method': 'effective_DC_1s_step_estimate', 'cell_resistance_ohm': fitted,
            'response_seconds': 1.0, 'fit': 'least_squares_deltaV_equals_R_deltaI_zero_intercept',
            'fit_rmse_v': math.sqrt(sum(s['fit_residual_v']**2 for s in steps)/len(steps)),
            'fit_max_abs_residual_v': max(abs(s['fit_residual_v']) for s in steps), 'steps': steps,
            'limitations': ['two_nominal_DOD_points_only', 'not_AC_or_EIS',
                            'low_amplitude_C5_to_C50_step_not_high_current_pack_calibration',
                            'constant_R_ignores_SOC_temperature_and_relaxation_history']}


def build_curve(rpt, cell_resistance_ohm):
    """C/5 V에 I·R을 더해 SOC별 등가 OCV(V)를 만든다. 파일·RPT 상태를 바꾸지 않는다."""
    if not math.isfinite(cell_resistance_ohm) or not 0 < cell_resistance_ohm < 0.2:
        raise ValueError('Invalid resistance for equivalent curve')
    blocks = rpt['_blocks']
    q0 = blocks[3]['rows'][0]['QDischarge_mA_h']
    retained = []
    spans = []
    for ns in (3, 5, 7):
        rows = blocks[ns]['rows']
        chosen = [r for r in rows if ns == 3 or r['time_s']-rows[0]['time_s'] >= 5.0]
        retained.extend(chosen)
        spans.append({'ns': ns, 'source_row_start': chosen[0]['source_row'],
                      'source_row_end_exclusive': chosen[-1]['source_row']+1})
    retained.reverse()
    soc = [1-(r['QDischarge_mA_h']-q0)/1000/rpt['cell_capacity_ah'] for r in retained]
    voltage = [r['Ecell_V']-r['I_mA']/1000*cell_resistance_ohm for r in retained]
    if soc[0] != 0 or soc[-1] != 1 or any(b <= a for a, b in zip(soc, soc[1:])):
        raise ValueError('Invalid measured SOC coverage')
    grid = [i/100 for i in range(101)]
    values = [_interpolate(soc, voltage, value) for value in grid]
    if any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Invalid equivalent voltage')
    residuals = [_interpolate(grid, values, s)-v for s, v in zip(soc, voltage)]
    brackets = []
    for point in grid:
        position = bisect_left(soc, point)
        brackets.append([retained[max(0, position-1)]['source_row'], retained[position]['source_row']])
    return {'ocv_soc': grid, 'ocv_cell_v': values,
            'diagnostics': {
                'method': 'C5_discharge_plus_I_R_OCV_equivalent',
                'soc_basis': '1_minus_same_RPT_counter_delta_divided_by_measured_capacity',
                'curve_soc_endpoints': 'start_and_2.5V_loaded_cutoff_of_same_RPT_not_equilibrium_SOC',
                'grid_interpolation': 'linear_101_points_no_extrapolation_no_monotone_smoothing',
                'retained_row_spans': spans, 'grid_source_bracket_rows': brackets,
                'retained_rows': len(retained),
                'excluded_relaxation_and_settling_rows': sum(len(b['rows']) for b in blocks[3:8])-len(retained),
                'reconstruction_rmse_v': math.sqrt(sum(r*r for r in residuals)/len(residuals)),
                'reconstruction_max_abs_error_v': max(abs(r) for r in residuals),
                'reference_discharge_current_a_range': [min(-r['I_mA']/1000 for r in retained),
                                                        max(-r['I_mA']/1000 for r in retained)],
                'reference_temperature_c_range': [min(r['Temperature__C'] for r in retained),
                                                  max(r['Temperature__C'] for r in retained)],
                'limitations': ['not_measured_equilibrium_OCV', 'C5_polarization_and_hysteresis_remain',
                                'cutoff_knee_is_not_a_usable_high_load_flight_envelope',
                                'no_RC_relaxation_temperature_rate_capacity_or_cell_imbalance_model',
                                'reference_reconstruction_residual_is_not_independent_validation']}}


def select_stages(rpts, targets=(100.0, 95.0, 90.0, 85.0)):
    """SOH % 목표에 1%p 이내인 실제 RPT 네 개를 고른다. 수치 합성·상태 변경은 하지 않는다."""
    if len(targets) != 4 or len(rpts) < 4 or any(not math.isfinite(t) for t in targets):
        raise ValueError('Four measured stages required')
    chosen = [min(rpts, key=lambda r: (abs(r['soh_percent']-target), r['rpt_index'])) for target in targets]
    if (len({r['rpt_index'] for r in chosen}) != 4 or
            any(abs(r['soh_percent']-t) > 1.0 for r, t in zip(chosen, targets)) or
            any(b['rpt_index'] <= a['rpt_index'] for a, b in zip(chosen, chosen[1:])) or
            chosen[0]['rpt_index'] != 0):
        raise ValueError('Insufficient distinct measured stages near requested health; no synthesis')
    return chosen


def _segments(rows):
    """CSV 행을 연속 Ns/counter 구간으로 나눈다. 입력 단위 유지, 외부 상태 변경 없음."""
    indexed = enumerate(rows)
    for key, group in itertools.groupby(indexed, lambda pair: (int(pair[1]['Ns']), int(pair[1]['cycleNumber']))):
        samples = []
        for index, row in group:
            converted = {field: float(row[field]) for field in
                         ('time_s', 'Ecell_V', 'I_mA', 'Temperature__C', 'QDischarge_mA_h')}
            converted['source_row'] = index
            samples.append(converted)
        yield {'ns': key[0], 'counter': key[1], 'rows': samples}


def _measure_rpt(blocks, index, missions_before):
    """완전 RPT의 Ah·행 범위·적분 검산값을 반환한다. 입력·외부 상태를 변경하지 않는다."""
    discharge = [row for block in blocks[3:8] for row in block['rows']]
    all_rows = [row for block in blocks for row in block['rows']]
    if any(any(not math.isfinite(float(v)) for v in row.values()) for row in all_rows):
        raise ValueError('Nonfinite RPT measurement')
    if any(b['time_s'] <= a['time_s'] for a, b in zip(all_rows, all_rows[1:])):
        raise ValueError('Nonincreasing RPT time')
    for ns, expected, tolerance in ((2, 880, 20), (3, 3600, 1), (4, 30, 0.1),
                                     (5, 7200, 1), (6, 30, 0.1), (8, 890, 20)):
        samples = blocks[ns]['rows']
        if abs(samples[-1]['time_s']-samples[0]['time_s']-expected) > tolerance:
            raise ValueError('Incomplete RPT segment duration')
    if any(r['I_mA'] >= 0 or r['Ecell_V'] <= 0 for r in discharge):
        raise ValueError('Expected discharge-only RPT current')
    if any(b['time_s']-a['time_s'] > 1.1 or b['QDischarge_mA_h'] < a['QDischarge_mA_h']
           for a, b in zip(discharge, discharge[1:])):
        raise ValueError('RPT sample gap or capacity counter reset')
    for ns in (3, 5, 7):
        if not -650 < statistics.median(r['I_mA'] for r in blocks[ns]['rows']) < -550:
            raise ValueError('RPT must use C/5 reference discharge')
    for ns in (4, 6):
        if not -100 < statistics.median(r['I_mA'] for r in blocks[ns]['rows']) < -40:
            raise ValueError('RPT must use C/50 relaxation steps')
    for ns in (2, 8):
        if any(abs(r['I_mA']) > 1 for r in blocks[ns]['rows']):
            raise ValueError('RPT resting current is nonzero')
    if (not 4.1 < discharge[0]['Ecell_V'] < 4.25 or
            not 2.45 < discharge[-1]['Ecell_V'] < 2.55 or
            discharge[0]['QDischarge_mA_h'] > 0.01 or
            any(r['I_mA'] <= 0 for b in blocks[:2] for r in b['rows'])):
        raise ValueError('RPT charge/cutoff/capacity origin incomplete')
    capacity = (discharge[-1]['QDischarge_mA_h']-discharge[0]['QDischarge_mA_h'])/1000
    integral = sum((b['time_s']-a['time_s'])*(-a['I_mA']-b['I_mA'])/2/3600000
                   for a, b in zip(discharge, discharge[1:]))
    if capacity <= 0 or abs(integral-capacity) > 0.001:
        raise ValueError('Capacity counter/integral mismatch')
    return {'rpt_index': index, 'tester_cycle_number': blocks[2]['counter'],
            'missions_before': missions_before, 'source_row_start': blocks[0]['rows'][0]['source_row'],
            'source_row_end_exclusive': blocks[-1]['rows'][-1]['source_row']+1,
            'cell_capacity_ah': capacity, 'integrated_capacity_ah': integral,
            'capacity_integral_difference_ah': integral-capacity, '_blocks': list(blocks)}


def extract_rpts(rows):
    """순서·전류 기반 RPT를 반환한다. Ah와 상대 SOH %, counter reset 무관·파일 쓰기 없음."""
    history = deque(maxlen=9)
    result = []
    candidate_count = 0
    missions = 0
    for block in _segments(rows):
        history.append(block)
        if block['ns'] == 2:
            candidate_count += 1
        recent = list(history)
        if len(recent) >= 3 and [b['ns'] for b in recent[-3:]] == [4, 5, 6]:
            if all(statistics.median(r['I_mA'] for r in b['rows']) < -2000 for b in recent[-3:]):
                missions += 1
        if [b['ns'] for b in recent] == list(range(9)):
            result.append(_measure_rpt(recent, len(result), missions))
    if not result or candidate_count != len(result):
        raise ValueError('Missing/incomplete RPT protocol')
    baseline = result[0]['cell_capacity_ah']
    for rpt in result:
        rpt['soh_percent'] = 100*rpt['cell_capacity_ah']/baseline
    return result


def _file_identity(path):
    """파일 전체 바이트 수·MD5·SHA-256을 구한다. 파일을 읽기만 하며 단위는 byte다."""
    md5, sha = hashlib.md5(), hashlib.sha256()
    size = 0
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            size += len(chunk)
            md5.update(chunk)
            sha.update(chunk)
    return {'bytes': size, 'md5': md5.hexdigest(), 'sha256': sha.hexdigest()}


def verify_inputs(raw_dir):
    """CMU v3 메타데이터와 전체 원본 해시를 확인한다. prefix를 거부하며 파일은 수정하지 않는다."""
    raw_dir = Path(raw_dir)
    metadata = json.loads((raw_dir/'cmu-v3.json').read_text(encoding='utf-8'))
    if metadata.get('version') != 3 or metadata.get('doi') != '10.1184/R1/14226830.v3':
        raise ValueError('Expected fixed CMU v3 metadata')
    entries = {item['name']: item for item in metadata['files']}
    identities = {}
    for name, (file_id, size, md5) in FILE_PINS.items():
        entry = entries.get(name, {})
        url = f'https://ndownloader.figshare.com/files/{file_id}'
        if (entry.get('id') != file_id or entry.get('size') != size or
                entry.get('computed_md5') != md5 or entry.get('download_url') != url):
            raise ValueError('Repository file metadata disagrees with fixed pins')
        measured = _file_identity(raw_dir/name)
        if measured['bytes'] != size or measured['md5'] != md5:
            raise ValueError(f'Expected complete {name}; checksum/size mismatch')
        identities[name] = {**measured, 'url': url, 'whole_file_md5_verified': True,
                            'repository_md5': md5, 'download_scope': 'complete_file'}
    for name, url in (('cmu-v3.json', API_URL), ('article.xml', ARTICLE_URL)):
        identities[name] = {**_file_identity(raw_dir/name), 'url': url,
                            'repository_md5': None, 'whole_file_md5_verified': False,
                            'download_scope': 'complete_local_reference_document'}
    return identities


def _rpt_summary(rpt):
    """RPT Ah·상대 SOH·0기반 행 범위를 직렬화한다. 입력·외부 상태 효과 없음."""
    result = {key: value for key, value in rpt.items() if not key.startswith('_')}
    result['row_numbering'] = 'zero_based_data_rows_excluding_header_end_exclusive'
    result['capacity_method'] = 'QDischarge_mA_h_end_minus_start_divided_by_1000'
    result['source_step_spans'] = [
        {'ns': block['ns'], 'tester_cycle_number': block['counter'],
         'source_row_start': block['rows'][0]['source_row'],
         'source_row_end_exclusive': block['rows'][-1]['source_row']+1,
         'source_time_start_s': block['rows'][0]['time_s'],
         'source_time_end_s': block['rows'][-1]['time_s']}
        for block in rpt['_blocks']]
    return result


def _author_comparison(rpt, resistance, impedance):
    """원저자 DCIR Ω와 재계산값을 RPT 순서로 교차 확인한다. counter는 식별에 쓰지 않는다."""
    index = rpt['rpt_index']
    if index >= len(impedance):
        raise ValueError('Selected RPT has no author impedance cross-check')
    author = impedance[index]
    differences = []
    for step, label in zip(resistance['steps'], ('20%', '60%')):
        one = float(author[label+'_1_second'])
        thirty = float(author[label+'_30_second'])
        if abs(step['one_second_ohm']-one) > 5e-5 or abs(step['last_relaxation_ohm']-thirty) > 1e-6:
            raise ValueError('Author DCIR table does not match chronological raw RPT')
        differences.append({'nominal_dod_label': label, 'author_one_second_ohm': one,
                            'author_thirty_second_ohm': thirty,
                            'one_second_difference_ohm': step['one_second_ohm']-one,
                            'thirty_second_difference_ohm': step['last_relaxation_ohm']-thirty})
    return {'file': 'VAH01_impedance.csv', 'source_row_zero_based': index,
            'author_cycle_label_not_raw_tester_counter': int(author['cycle numbers']),
            'alignment': 'RPT_chronological_ordinal_confirmed_by_two_30s_DCIR_values',
            'one_second_difference_reason': 'interpolated_exact_1s_from_pre_step_vs_author_discrete_sample',
            'values': differences, 'used_as_AC_EIS_resistance': False}


def generate(raw_dir, output_dir):
    """검증된 원본에서 4S2P catalog/곡선 CSV/근거를 발행한다. Ah·Ω·V 단위, 신규 폴더만 생성한다."""
    raw_dir, output_dir = Path(raw_dir), Path(output_dir)
    if output_dir.exists():
        raise FileExistsError('Refusing to overwrite profile catalog')
    identities = verify_inputs(raw_dir)
    with (raw_dir/'VAH01.csv').open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != SOURCE_FIELDS:
            raise ValueError('Unexpected source schema')
        rpts = extract_rpts(reader)
    with (raw_dir/'VAH01_impedance.csv').open(encoding='utf-8', newline='') as stream:
        impedance = list(csv.DictReader(stream))
    selected = select_stages(rpts, TARGETS)
    pack_assumptions = {
        'electrical_scope': 'chosen_virtual_homogeneous_4S2P_not_measured_x500_pack',
        'capacity_formula': 'pack_Ah = measured_cell_Ah * 2',
        'resistance_formula': 'pack_Ohm = estimated_cell_Ohm * 4 / 2',
        'voltage_formula': 'pack_V = 4 * OCV_equivalent_cell_V(SOC) - pack_A * pack_Ohm',
        'soc_evolution': 'dSOC = -pack_A * simulated_dt_s / (3600 * pack_Ah)',
        'initial_soc': 0.95, 'initial_soc_is_assumed': True,
        'same_cell_replication': True, 'thermal_model_calibrated': False,
        'absolute_SOH_certified': False, 'AI_training_performed': False,
        'wall_time_replay': False,
    }
    records, files = [], {}
    for target, rpt in zip(TARGETS, selected):
        resistance = estimate_resistance(rpt)
        curve = build_curve(rpt, resistance['cell_resistance_ohm'])
        identifier = f"cmu_vah01_rpt{rpt['rpt_index']:03d}"
        csv_name = identifier+'.csv'
        text = io.StringIO(newline='')
        writer = csv.writer(text, lineterminator='\n')
        writer.writerow(['soc', 'ocv_cell_v'])
        writer.writerows(zip(curve['ocv_soc'], curve['ocv_cell_v']))
        files[csv_name] = text.getvalue()
        records.append({
            'id': identifier,
            'label_ko': f"{'초기 정상 기준' if rpt['rpt_index'] == 0 else '실측 열화'} · 상대 용량 {rpt['soh_percent']:.2f}%",
            'capacity_ah': rpt['cell_capacity_ah']*2,
            'pack_resistance_ohm': resistance['cell_resistance_ohm']*4/2,
            'initial_soc': 0.95, 'ocv_soc': curve['ocv_soc'], 'ocv_cell_v': curve['ocv_cell_v'],
            'soh_percent': rpt['soh_percent'],
            'source': {
                'dataset': 'CMU_eVTOL_v3', 'dataset_doi': '10.1184/R1/14226830.v3',
                'dataset_metadata_url': API_URL, 'paper_url': ARTICLE_URL,
                'source_cell': 'VAH01', 'license': 'CC BY 4.0',
                'attribution': 'Alexander Bills et al., eVTOL Battery Dataset, CMU, version 3',
                'raw_files': identities, 'selected_rpt': _rpt_summary(rpt),
                'initial_reference_cell_capacity_ah': rpts[0]['cell_capacity_ah'],
                'health_basis': 'relative_to_same_cell_first_complete_C5_RPT',
                'selection_target_percent_not_measured_label': target,
                'selection': 'nearest_measured_RPT_no_scalar_capacity_or_resistance_synthesis',
                'resistance': resistance,
                'author_impedance_crosscheck': _author_comparison(rpt, resistance, impedance),
                'curve': curve['diagnostics'], 'curve_csv': csv_name,
                'measured': ['source_cell_voltage_current_temperature_charge_counter', 'RPT_capacity'],
                'derived': ['relative_capacity_SOH', 'effective_DC_1s_resistance', 'OCV_equivalent_SOC_curve'],
                'assumed': pack_assumptions,
                'scope': 'simulation_scenario_pack_not_independent_training_cells_or_real_vehicle_validation',
            },
        })
    catalog = {'schema': 'arion.cmu_battery_profiles.v1', 'series_cells': 4, 'parallel_cells': 2,
               'default_profile_id': records[0]['id'], 'profiles': records}
    files['catalog.json'] = json.dumps(catalog, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    provenance = {
        'schema': 'arion.cmu_battery_profile_provenance.v1',
        'extractor': 'tools/battery_profiles/cmu_aging.py',
        'extractor_sha256': _file_identity(__file__)['sha256'], 'raw_files': identities,
        'rpt_inventory': [_rpt_summary(rpt) for rpt in rpts],
        'selected_rpt_indices': [r['rpt_index'] for r in selected],
        'selection_targets_percent': list(TARGETS),
        'observed_soh_percent_range': [min(r['soh_percent'] for r in rpts), max(r['soh_percent'] for r in rpts)],
        'source_cycle_number_warning': 'tester_counter_resets_ignored_for_RPT_identification',
        'unselected_final_rpt_note': '18 raw RPTs but author impedance table has 17 rows; final RPT not selected',
        'resistance_monotonicity': 'not_enforced; selected_95_percent_R_is_slightly_below_initial',
        'assumptions': pack_assumptions,
        'artifact_sha256': {name: hashlib.sha256(text.encode('utf-8')).hexdigest() for name, text in files.items()},
        'preservation': 'existing_independent_initial_3_cell_bundle_and_NASA_sources_not_modified',
    }
    files['provenance.json'] = json.dumps(provenance, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if verify_inputs(raw_dir) != identities:
        raise ValueError('Source changed during extraction')
    # 임시 파일은 raw 입력 옆 분석 폴더에서만 만들고 완성 폴더를 한 번에 공개한다.
    with tempfile.TemporaryDirectory(prefix='cmu-catalog-stage-', dir=raw_dir.parent) as temporary:
        staging = Path(temporary)/'catalog'
        staging.mkdir()
        for name, text in files.items():
            (staging/name).write_text(text, encoding='utf-8')
        if output_dir.exists():
            raise FileExistsError('Refusing to overwrite profile catalog')
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        os.rename(staging, output_dir)
    return catalog


def main(argv=None):
    """검증된 raw 폴더에서 신규 산출물을 재생성하는 CLI다. 경로 입력, 성공 시 Ah/Ω 요약을 출력한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-dir', type=Path, required=True, help='완전한 CMU v3 원본·참고문서 폴더')
    parser.add_argument('--output-dir', type=Path, required=True, help='기존에 존재하지 않는 새 catalog 폴더')
    args = parser.parse_args(argv)
    catalog = generate(args.raw_dir, args.output_dir)
    print(json.dumps({'output_dir': str(args.output_dir), 'profiles': [
        {key: p[key] for key in ('id', 'soh_percent', 'capacity_ah', 'pack_resistance_ohm')}
        for p in catalog['profiles']]}, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
