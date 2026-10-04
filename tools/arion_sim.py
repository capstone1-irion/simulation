#!/usr/bin/env python3
"""CMU 배터리·모터 이상 SITL 운영 명령. 설치/실기체 접속/자동 무장은 하지 않습니다."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from sim_service.control_api import request
from sim_service.model_config import validate_catalog
from sim_service.runtime import DEFAULT_CATALOG, REPO, Runtime, atomic_json, load_config, validate_config


def parse_args(argv=None):
    """사용자 제어를 명명된 동작으로 제한한다; 자유 형식 shell/PX4 명령을 받지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=REPO / 'config/service.local.json', help='개인 실행 경로 설정')
    subs = parser.add_subparsers(dest='action', required=True)
    init = subs.add_parser('init', help='설치된 실행파일 경로를 설정 파일로 저장; 기존 파일 덮어쓰기 없음')
    for key in ('px4-binary', 'plugin-build', 'decoder', 'ipc-dir'):
        init.add_argument('--' + key, required=True, type=Path)
    init.add_argument('--px4-source', type=Path, default=REPO / 'PX4-Autopilot')
    init.add_argument('--output-root', type=Path, default=REPO / 'experiments/service')
    init.add_argument('--state-dir', type=Path, default=REPO / '.arion-service')
    init.add_argument('--catalog', type=Path, default=DEFAULT_CATALOG)
    init.add_argument('--instance', type=int, default=35)
    init.add_argument('--rotate-bytes', type=int, default=16777216)
    for action in ('start', 'serve'):
        p = subs.add_parser(action, help='Disarmed 상태로 지속 수집 시작' if action == 'start' else '전경 서비스 실행')
        p.add_argument('--profile', help='profiles에 표시된 ID; 생략 시 정상 기준')
        if action == 'start':
            p.add_argument('--background', action='store_true', help='터미널과 분리하여 stop까지 실행')
    subs.add_parser('profiles', help='선택 가능한 CMU 기반 프로파일 4개와 모델 파라미터')
    subs.add_parser('status', help='서비스/수신기 상태와 기록 경로')
    watch = subs.add_parser('watch', help='현재 MAVLink 값 계속 표시; Ctrl+C는 표시만 종료')
    watch.add_argument('--interval', type=float, default=1.0)
    watch.add_argument('--json', action='store_true', help='정답 설정 없이 수신값만 JSONL 출력')
    takeoff = subs.add_parser('takeoff', help='정상 preflight를 통과한 로컬 SITL 이륙')
    takeoff.add_argument('--height', type=float, default=3.0, help='목표 상대고도 m, 1..10')
    subs.add_parser('land', help='착륙과 자동 Disarm 기다리기')
    p = subs.add_parser('profile', help='지상 Disarmed에서 배터리 프로파일 선택·초기화')
    p.add_argument('id')
    p = subs.add_parser('motor-fault', help='모터 1..4의 전류/RPM 텔레메트리 이상; 추진력 변화 아님')
    p.add_argument('motor', type=int)
    p.add_argument('--current-scale', type=float, default=1.6)
    p.add_argument('--rpm-scale', type=float, default=.85)
    for action, help_text in [('clear', '모터 이상만 해제; 소비한 배터리는 충전하지 않음'),
        ('reset', '선택 배터리의 SOC/소비량 초기화·모터 이상 해제; 지상 전용'),
        ('restore', '서비스 최초 배터리 프로파일로 복원·초기화'),
        ('world-reset', '지상에서 Gazebo 세계/시간 reset 후 수신 확인'),
        ('pause', 'Gazebo 시간 일시정지; 서비스와 수신기는 계속 실행'),
        ('resume', 'Gazebo 시간 재개'), ('gui', '같은 partition의 Gazebo 3D 창 시작 (선택)'),
        ('stop', 'Disarmed 상태에서 소유 서비스/시뮬레이터 정상 종료')]:
        subs.add_parser(action, help=help_text)
    return parser.parse_args(argv)


def initialize(args):
    """명시된 사용자 경로만 기록한다. 패키지 설치·바이너리 교체·자동 시작은 하지 않는다."""
    if args.config.exists():
        raise FileExistsError('Configuration already exists; edit it deliberately: ' + str(args.config))
    payload = {'schema': 'arion.sim_service.v1', 'instance': args.instance, 'rotate_bytes': args.rotate_bytes}
    for key in ('px4_source', 'px4_binary', 'plugin_build', 'decoder', 'ipc_dir', 'output_root', 'state_dir', 'catalog'):
        payload[key] = str(getattr(args, key).expanduser().resolve())
    payload = validate_config(payload)
    args.config.parent.mkdir(parents=True, exist_ok=True)
    with args.config.open('x', encoding='utf-8') as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')
    load_config(args.config)
    return {'ok': True, 'config': str(args.config.resolve()), 'started': False}


def background_start(args, config):
    """독립 서비스 프로세스를 시작하고 실제 API 준비를 확인한다; 실패 시 새로 만든 PID만 종료한다."""
    state_dir = Path(config['state_dir'])
    state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    socket_path = state_dir / 'control.sock'
    if socket_path.exists():
        raise RuntimeError('Service socket already exists; use status/stop, do not start a duplicate')
    argv = [sys.executable, str(Path(__file__).resolve()), '--config', str(args.config.resolve()), 'serve']
    if args.profile:
        argv.extend(['--profile', args.profile])
    launcher_log = state_dir / ('launcher-' + str(time.time_ns()) + '.log')
    with launcher_log.open('xb') as log:
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                   cwd=REPO, start_new_session=True)
    deadline = time.monotonic() + 115
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('Service startup failed; inspect ' + str(launcher_log))
        try:
            result = request(socket_path, {'action': 'status'}, timeout=2)
            if result.get('state') == 'running' and result.get('service_pid') == process.pid:
                return result
        except (OSError, ValueError):
            pass
        time.sleep(.2)
    process.terminate()
    try:
        process.wait(timeout=45)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'Startup exceeded 115s; shutdown requested but PID {process.pid} still needs inspection. Log: {launcher_log}')
    raise TimeoutError('Service startup exceeded 115s; owned process stopped. Log: ' + str(launcher_log))


def watch(config, interval, json_output=False):
    """원문 수신값을 반복 표시한다; 제어·정답 메타데이터는 JSONL 데이터 출력에서 제외한다."""
    if not math.isfinite(interval) or not .1 <= interval <= 60:
        raise ValueError('watch interval must be .1..60 seconds')
    try:
        while True:
            result = request(Path(config['state_dir']) / 'control.sock', {'action': 'telemetry'}, timeout=5)
            if not result.get('ok'):
                raise RuntimeError(result.get('error', 'Telemetry unavailable'))
            if json_output:
                print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
            else:
                messages = result['messages']
                battery, motor = messages.get('BATTERY_STATUS', {}), messages.get('ESC_STATUS', {})
                cells = [v for v in battery.get('voltages', []) if v != 65535]
                volts = sum(cells) / 1000 if cells else None
                current = battery.get('current_battery', -1)
                amps = None if current == -1 else current / 100
                print(f'V={volts}  A={amps}  SOC={battery.get("battery_remaining")}  '
                      f'mAh={battery.get("current_consumed")}  RPM={motor.get("rpm")}  '
                      f'ESC_A={motor.get("current")}', flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        return 0


def main(argv=None):
    """직접 사용할 CLI 진입점; 오류는 nonzero와 구체적인 사유로 알리고 완료를 꾸며내지 않는다."""
    args = parse_args(argv)
    try:
        if args.action == 'init':
            result = initialize(args)
        elif args.action == 'profiles':
            path = Path(load_config(args.config)['catalog']) if args.config.exists() else DEFAULT_CATALOG
            data = validate_catalog(json.loads(path.read_text(encoding='utf-8')))
            result = {'ok': True, 'default_profile_id': data['default_profile_id'],
                      'profiles': [{k: p[k] for k in ('id', 'label_ko', 'soh_percent', 'capacity_ah', 'pack_resistance_ohm')}
                                   for p in data['profiles']],
                      'meaning': 'relative CMU reference capacity; 4S2P simulation pack, not measured x500 SOH'}
        else:
            config = load_config(args.config)
            if args.action in ('start', 'serve'):
                if args.action == 'start' and args.background:
                    result = background_start(args, config)
                else:
                    catalog = json.loads(Path(config['catalog']).read_text(encoding='utf-8'))
                    return Runtime(config, catalog, args.profile).run_forever()
            elif args.action == 'watch':
                return watch(config, args.interval, args.json)
            else:
                payload = {'action': args.action}
                for name in ('height', 'id', 'motor', 'current_scale', 'rpm_scale'):
                    if hasattr(args, name):
                        payload[name] = getattr(args, name)
                try:
                    result = request(Path(config['state_dir']) / 'control.sock', payload)
                except (FileNotFoundError, ConnectionRefusedError):
                    status_path = Path(config['state_dir']) / 'status.json'
                    if args.action != 'status' or not status_path.is_file():
                        raise RuntimeError('Service is not running; start it first')
                    result = json.loads(status_path.read_text())
                    result['live_connection'] = False
                    if result.get('state') not in ('stopped', 'failed'):
                        result.update(ok=False, state='unreachable', error='Cached state only; live service status is unknown')
                if args.action == 'stop' and result.get('ok'):
                    until = time.monotonic() + 65
                    while time.monotonic() < until:
                        status_path = Path(config['state_dir']) / 'status.json'
                        if status_path.is_file():
                            final = json.loads(status_path.read_text())
                            if final.get('state') in ('stopped', 'failed'):
                                result = final
                                break
                        time.sleep(.2)
                    else:
                        raise TimeoutError('Stop accepted but clean shutdown not yet confirmed; inspect status')
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
        return 0 if result.get('ok') else 1
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps({'ok': False, 'error': type(error).__name__ + ': ' + str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
