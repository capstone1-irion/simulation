"""역할: 전체 코드·문서 작성 뒤 한 번만 실행하는 단계형 통합 검증.
입력: 고정 의존성, 새 결과 폴더, 선택적인 읽기 전용 기업 BIN 경로.
출력: 단계 로그/report.json, 실제 headless 비행 원문·정답·종료 증거. 자동 재시도 없음.
호출자: 승인된 최종 검증 또는 팀원의 명시적 전체 검사. 실기체는 연결하지 않는다.
"""
from bisect import bisect_right
from collections import Counter
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import traceback
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True


def require(condition, message):
    """최적화 Python에서도 사라지지 않는 완료 조건이다."""
    if not condition:
        raise RuntimeError(message)


def digest(path):
    """기업 원자료를 올리지 않고 원문 보존/재구성 여부를 로컬에서 대조한다."""
    with Path(path).open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def save(path, value):
    """실패 직전까지의 상태도 남긴다. 디스크 오류는 숨기지 않는다."""
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def source_state():
    """고정 checkout과 git이 추적하는 원본의 변경 유무를 검사한다."""
    source = ROOT / 'external/PX4-Autopilot'
    def git(*args):
        return subprocess.check_output(['git', '-C', str(source), *args], text=True, timeout=120).strip()
    pins = json.loads((ROOT / 'config/sources.lock.json').read_text())
    result = {'head': git('rev-parse', 'HEAD'), 'status': git('status', '--porcelain'),
              'submodules': git('submodule', 'status', '--recursive')}
    require(result['head'] == pins['px4']['commit'], 'PX4 고정 revision 불일치')
    require(not result['status'], 'PX4 또는 초기화한 submodule 원본에 변경이 있음')
    return result


def units(folder):
    """합성 입력 검사만 실행한다. 실제 비행 결과와 구별해 수를 기록한다."""
    suite = unittest.defaultTestLoader.discover(str(ROOT / 'tests'), pattern='test_*.py')
    with (folder / 'unit.log').open('w', encoding='utf-8') as output:
        result = unittest.TextTestRunner(stream=output, verbosity=2).run(suite)
    summary = {'tests_run': result.testsRun, 'failures': len(result.failures),
               'errors': len(result.errors), 'skipped': len(result.skipped)}
    save(folder / 'unit.json', summary)
    require(result.wasSuccessful() and not result.skipped, '단위 검사 실패: unit.log / unit.json 참조')
    return summary


def build(folder, jobs):
    """빌드와 native 합성 검사는 단계마다 딱 한 번 실행한다."""
    for name, command in (
        ('build', [sys.executable, '-B', str(ROOT / 'tools/build.py'), '--jobs', str(jobs)]),
        ('native', [str(ROOT / 'hermes/build/native/native_motor_test')]),
    ):
        with (folder / (name + '.log')).open('w', encoding='utf-8') as output:
            subprocess.run(command, cwd=ROOT, env=os.environ.copy(), stdout=output,
                           stderr=subprocess.STDOUT, check=True, timeout=2700 if name == 'build' else 30)
    return {'build_log': str(folder / 'build.log'), 'native_log': str(folder / 'native.log')}


def pause_check(runtime):
    """실제 pause readback 후 벽시계가 흘러도 적분·고장 시간이 정지하는지 본다."""
    runtime.pause(True)
    before = runtime.link.snapshot()
    time.sleep(3.2)  # 기본 heartbeat 신선도 3초보다 긴 명시적 pause다.
    after = runtime.link.snapshot()
    require(before['motor']['sim_time_s'] == after['motor']['sim_time_s'], 'pause 중 SIM 시각 진행')
    require(before['battery']['consumed_mah'] == after['battery']['consumed_mah'], 'pause 중 소비 적분')
    for key in ('fault_until_s', 'applied_revision', 'fault_active'):
        require(before['motor'][key] == after['motor'][key], 'pause 중 고장 상태 변경: ' + key)
    runtime.pause(False)
    runtime.healthy()
    return {'sim_time_s': before['motor']['sim_time_s'], 'consumed_mah': before['battery']['consumed_mah']}


def flight(folder, name, profile, soc):
    """승인된 독립 가상 기체만 운영하고 저장 원문으로 결과를 다시 확인한다."""
    from capstone.collector import encode_json
    from capstone.preprocess import Preprocessor
    from capstone.scenario import hold, land, run_scenario, takeoff
    from capstone.simulation import Runtime
    live_path = folder / (name + '-live.jsonl')
    runtime = None
    with live_path.open('x', encoding='utf-8') as output:
        processor = Preprocessor('px4_gz', system_id=32,
                                 on_event=lambda event: output.write(encode_json(event) + '\n'))
        runtime = Runtime(profile, soc, instance=31, output_root=folder / name, on_record=processor.consume)
        try:
            runtime.start()
            if name == 'motor':
                takeoff(runtime)
                hold(runtime, 5)
                runtime.fault(motor=1, scale=.9, duration=5)
                hold(runtime, .5)
                pause_check(runtime)
                hold(runtime, 5)
                runtime.wait(lambda: runtime.link.snapshot()['motor'].get('fault_active') is False,
                             timeout=5, description='고장 기간 만료')
                hold(runtime, 2)
                runtime.fault(motor=1, scale=.95, duration=5)
                hold(runtime, 1)
                runtime.fault()
                hold(runtime, 3)
                land(runtime)
            else:
                pause_check(runtime)
                run_scenario(runtime, name)
        except BaseException as error:
            runtime.errors.append(repr(error))
            raise
        finally:
            cleanup = runtime.close()
            processor.finish()
    require(cleanup['clean'], '비행 후 자원 정리 실패: ' + str(runtime.folder))
    return examine_run(runtime.folder, live_path, name, profile, soc)


def examine_run(run, live_path, name, profile, soc):
    """실제 저장파일에서 통과/약화·물리 반응·PX4 재출력·인과 행을 대조한다."""
    from capstone.collector import decode_json, encode_json
    from capstone.preprocess import preprocess_file
    from capstone.simulation import RATES
    def rows(path):
        with Path(path).open(encoding='utf-8') as source:
            return [decode_json(line) for line in source]
    raw = rows(run / 'telemetry/mavlink_raw.jsonl')
    summary = json.loads((run / 'telemetry/summary.json').read_text())
    require(len(raw) == summary['saved_records'], '수집 summary와 원문 행 수 불일치')
    messages = [r for r in raw if r['kind'] == 'message']
    counts = Counter(r['message_name'] for r in messages)
    require(dict(counts) == summary['seen_by_type'], '메시지 종류별 원문 개수 불일치')
    require(all(counts[key] > 0 for key in RATES), '요청한 PX4 스트림 중 미수신 항목: ' + str(counts))
    require(all(r['source_system'] == 32 and r['source_component'] == 1 for r in messages),
            '관측에 PX4 component1 이외의 송신자 혼입')
    forbidden = {'profile_id', 'fault_active', 'fault_started_s', 'output_rad_s', 'joint_rad_s'}
    require(all(not forbidden.intersection(r['fields']) for r in messages), '관측 fields에 정답 혼입')
    truth = rows(run / 'groundtruth/motor_battery.jsonl')
    valid = [r for r in truth if r['battery'] is not None and r['motor']['error'] is None]
    require(bool(valid), '실제 명령/배터리 정답이 없음')
    require(valid[0]['battery']['soc'] == soc and valid[0]['battery']['consumed_mah'] == 0,
            '새 run의 SOC/소비 원점 불일치')
    metadata = json.loads((run / 'groundtruth/run.json').read_text())
    require(metadata['profile_id'] == profile and metadata['initial_soc'] == soc, '프로파일 기록 불일치')
    dynamic = [r for r in valid if not r['motor']['paused'] and min(r['motor']['command_rad_s']) > 200]
    require(len(dynamic) > 20, '실제 회전 중 중계 표본 부족')
    active = []
    normal = []
    for row in dynamic:
        motor = row['motor']
        require(motor['output_input_seq'] == motor['input_seq'], '상태의 입력/출력 sequence 불일치')
        require(motor['output_sim_time_s'] == motor['sim_time_s'], '현재 tick 출력 없음')
        for index, (command, applied) in enumerate(zip(motor['command_rad_s'], motor['output_rad_s'])):
            expected = command * (motor['scale'] if motor['fault_active'] and index + 1 == motor['motor'] else 1)
            require(math.isclose(applied, expected, rel_tol=1e-12, abs_tol=1e-8), '명령 패스스루/배율 불일치')
        require(all(isinstance(value, (int, float)) and math.isfinite(value) for value in motor['joint_rad_s']),
                '실제 joint 관측 결측')
        require(all(value > 0 for value in motor['joint_rad_s'][:2]) and
                all(value < 0 for value in motor['joint_rad_s'][2:]), 'stock 회전 방향 불일치')
        (active if motor['fault_active'] else normal).append(row)
    require(bool(normal), '정상 동적 중계 구간 없음')
    response = None
    if name == 'motor':
        active = [r for r in active if r['motor']['scale'] == .9 and
                  r['motor']['sim_time_s'] - r['motor']['fault_started_s'] > .3]
        require(len(active) > 5, '고장 중 동적 표본 부족')
        ratios = [abs(r['motor']['joint_rad_s'][0]) / r['motor']['command_rad_s'][0] for r in active]
        require(abs(statistics.median(ratios) - .9) < .08, '관절 응답에서 선택 모터 약화를 확인하지 못함')
        begin, end = active[0]['receive_monotonic_ns'], active[-1]['receive_monotonic_ns']
        attitudes = [r for r in messages if r['message_name'] == 'ATTITUDE']
        def tilt(record):
            return math.hypot(record['fields']['roll'], record['fields']['pitch'])
        baseline = [tilt(r) for r in attitudes if begin - 3_000_000_000 <= r['receive_monotonic_ns'] < begin]
        during = [tilt(r) for r in attitudes if begin <= r['receive_monotonic_ns'] <= end]
        require(bool(baseline) and bool(during), '고장 전/중 자세 관측 부족')
        response = {'joint_command_ratio_median': statistics.median(ratios),
                    'baseline_tilt_median_rad': statistics.median(baseline), 'fault_tilt_peak_rad': max(during)}
        require(max(during) > statistics.median(baseline) + .0001, '고장 구간의 자세 반응을 확인하지 못함')
        require(valid[-1]['motor']['fault_active'] is False, '종료 전 패스스루 복귀 실패')
    else:
        require(not active, '정상/배터리 비교에 모터 고장이 혼입됨')
    batteries = [r for r in messages if r['message_name'] == 'BATTERY_STATUS']
    stamps = [r['receive_monotonic_ns'] for r in valid]
    matched = 0
    for record in batteries:
        index = bisect_right(stamps, record['receive_monotonic_ns']) - 1
        if index < 0:
            continue
        model = valid[index]['battery']
        wire = record['fields']
        # 비행 부하 과도기의 송신/재송신 시차와 steady 구간을 구분한다.
        voltage = sum(wire['voltages'][:4]) / 1000
        if abs(voltage - model['voltage_v']) < .08 and abs(wire['current_battery'] / 100 - model['current_a']) < .2:
            require(abs(wire['battery_remaining'] - model['soc'] * 100) <= 1, 'SOC wire 단위 불일치')
            matched += 1
    require(matched >= 10, 'PX4 재출력 배터리와 모델의 대응 표본 부족')
    events = rows(run / 'groundtruth/events.jsonl')
    resumes = [event for event in events if event['event'] == 'resume_ready']
    require(bool(resumes), '새 heartbeat를 확인한 재개 기록 없음')
    require(len(resumes) == sum(event['event'] == 'resume_wait' for event in events), '재개 미완료 기록 존재')
    for event in resumes:
        require(any(record['message_name'] == 'HEARTBEAT' and
                    event['after_monotonic_ns'] < record['receive_monotonic_ns'] <= event['observed_monotonic_ns']
                    for record in messages), '재개 기록의 실제 새 heartbeat 원문 없음')
    require(any(r['event'] == 'airborne_observed' for r in events), '실제 이륙 확인 없음')
    require(any(r['event'] == 'landed_disarmed_observed' for r in events), '착륙/자동 Disarm 확인 없음')
    cleanup = json.loads((run / 'cleanup.json').read_text())
    require(cleanup['clean'], '저장된 cleanup이 실패함')
    for path in (Path('/tmp/px4_lock-31'), Path('/tmp/px4-sock-31')):
        require(not path.exists(), '소유 PX4 IPC 잔류: ' + str(path))
    offline = live_path.with_name(live_path.stem + '-offline.jsonl')
    converted = preprocess_file(run / 'telemetry', offline, firmware='px4_gz', system_id=32)
    live = rows(live_path)
    replayed = [r for r in rows(offline) if r['kind'] != 'metadata']
    require([encode_json(r) for r in live] == [encode_json(r) for r in replayed], 'live/offline 인과 행 불일치')
    features = [r for r in live if r['kind'] == 'row']
    require(any(r['window_battery_ready'] for r in features), '유효 배터리 창 없음')
    require(any(r['window_motion_ready'] for r in features), '유효 기체 응답 창 없음')
    ages = [r['motor']['input_age_s'] for r in dynamic]
    return {'run_dir': str(run), 'messages': dict(counts), 'raw_records': len(raw),
            'battery_wire_ids': sorted({r['fields']['id'] for r in batteries}),
            'battery_matched_samples': matched, 'preprocess': converted, 'resume_checks': resumes,
            'input_age_sim_s_max': max(ages), 'input_age_sim_s_median': statistics.median(ages),
            'age_is_not_end_to_end_latency': True, 'response': response, 'cleanup': cleanup}


def company_bin(path, folder):
    """사용자가 보유한 BIN 하나를 읽기 전용으로 전부 소비하고 원문 바이트를 대조한다."""
    from capstone.collector import encode_json
    from capstone.preprocess import Preprocessor
    from capstone.replay import iter_records
    before = digest(path)
    reconstructed = hashlib.sha256()
    counts = Counter()
    byte_count = 0
    processor = Preprocessor('arducopter_4_4')
    records = iter_records(path)
    with (folder / 'company-features.jsonl').open('x', encoding='utf-8') as output:
        try:
            for record in records:
                counts[record['kind']] += 1
                if record['kind'] in ('message', 'opaque'):
                    raw = bytes.fromhex(record['raw_hex'])
                    reconstructed.update(raw)
                    byte_count += len(raw)
                for event in processor.consume(record):
                    output.write(encode_json(event) + '\n')
            for event in processor.finish():
                output.write(encode_json(event) + '\n')
        finally:
            records.close()
    require(before == digest(path) == reconstructed.hexdigest(), '기업 원문 변경 또는 바이트 재구성 불일치')
    require(byte_count == path.stat().st_size and counts['message'] > 0, '기업 BIN 전체 소비 실패')
    return {'source': str(path), 'sha256': before, 'bytes': byte_count, 'counts': dict(counts),
            'processor': processor.snapshot(), 'health_labels_inferred': False}


def main():
    """실패한 단계 뒤에는 진행하지 않으며, 이전 결과 폴더를 덮어쓰거나 재사용하지 않는다."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--bin', type=Path)
    args = parser.parse_args()
    require(sys.version_info[:2] == (3, 12) and not sys.flags.optimize, '.venv Python 3.12, 최적화 없는 실행 필요')
    folder = args.out.resolve()
    require(folder.is_relative_to(ROOT / 'hermes'), '결과는 프로젝트 hermes 내부여야 함')
    folder.mkdir(parents=True, exist_ok=False)
    temp = folder / 'tmp'
    temp.mkdir()
    os.environ.update(TMPDIR=str(temp), PYTHONDONTWRITEBYTECODE='1', PYTHONPATH='')
    import tempfile
    tempfile.tempdir = str(temp)
    def interrupt(signum, frame):
        raise KeyboardInterrupt('통합 검증 중단 신호')
    signal.signal(signal.SIGTERM, interrupt)
    report = {'status': 'running', 'started_utc_ns': time.time_ns(), 'steps': [],
              'scope': 'headless SITL; synthetic unit tests labeled separately; no automatic retry',
              'not_covered': ['GUI/QGC', 'stock direct-route latency equivalence',
                              'all native fault/disconnect/reset branches', 'AI accuracy', 'physical aircraft']}
    steps = [('source_before', source_state), ('units', lambda: units(folder)),
             ('build_native', lambda: build(folder, args.jobs)),
             ('normal', lambda: flight(folder, 'normal', 'cmu_vah01_rpt000', .95)),
             ('motor', lambda: flight(folder, 'motor', 'cmu_vah01_rpt000', .95)),
             ('battery', lambda: flight(folder, 'battery', 'cmu_vah01_rpt013', .95))]
    if args.bin:
        steps.append(('company_bin', lambda: company_bin(args.bin.resolve(strict=True), folder)))
    try:
        for name, action in steps:
            report['current_step'] = name
            save(folder / 'report.json', report)
            print('STEP:', name, flush=True)
            started = time.monotonic()
            detail = action()
            report['steps'].append({'name': name, 'status': 'passed',
                                    'wall_s': time.monotonic() - started, 'detail': detail})
        report['status'] = 'passed_scoped'
    except BaseException as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        report['traceback'] = traceback.format_exc()
        print(report['traceback'], flush=True)
    finally:
        try:
            report['source_after'] = source_state()
        except BaseException as error:
            report['status'] = 'failed'
            report['source_after_error'] = repr(error)
        report['finished_utc_ns'] = time.time_ns()
        save(folder / 'report.json', report)
    print('RESULT:', report['status'], str(folder / 'report.json'), flush=True)
    return 0 if report['status'] == 'passed_scoped' else 1


if __name__ == '__main__':
    raise SystemExit(main())
