#!/usr/bin/env python3
"""역할: 사용자의 CLI 선택을 실험·수집·재생·전처리에 전달하는 진입점.
입력: 터미널 인자와 simulate의 명시적 조작 명령. 출력: run/원문/특징 파일·종료 코드.
호출자: 팀원 터미널. import만으로 설치·연결·비행하지 않으며 실행 제어는 simulation에 둔다.
"""
import argparse
from pathlib import Path
import select
import shlex
import signal
import sys
import time

from capstone.battery import PROFILE_DIR, load_profile
from capstone.collector import Collector, encode_json
from capstone.preprocess import Preprocessor, preprocess_file
from capstone.replay import RAW_FILE, iter_records
from capstone.scenario import land, run_scenario, takeoff
from capstone.simulation import Runtime


def show(value):
    """관측 NaN/bytes는 collector와 같은 태그로 표시하여 비표준 JSON을 피한다."""
    print(encode_json(value), flush=True)


def arguments():
    """기본 동작과 자동 비행을 별도 명령으로 분리한다."""
    parser = argparse.ArgumentParser(description='CapstoneV2: 원본 PX4 실험과 수신 전용 데이터 처리')
    commands = parser.add_subparsers(dest='action', required=True)
    commands.add_parser('profiles', help='배터리 선택지; 연결/비행 없음')
    for action in ('simulate', 'experiment'):
        command = commands.add_parser(action, help='지상 대화형 실행' if action == 'simulate' else '명시적 자동 SITL 비행')
        command.add_argument('--profile', default='cmu_vah01_rpt000')
        command.add_argument('--soc', type=float, default=.95, help='시작 충전비율 0~1, SOH 아님')
        command.add_argument('--instance', type=int, default=31)
        command.add_argument('--gui', action='store_true', help='Gazebo 3D 창도 열기')
        if action == 'experiment':
            command.add_argument('scenario', choices=('normal', 'motor', 'battery'))
    collect = commands.add_parser('collect', help='UDP 수신만 수행; heartbeat/제어/배터리 송신 없음')
    collect.add_argument('--host', default='127.0.0.1', help='로컬 bind IP')
    collect.add_argument('--port', type=int, required=True)
    collect.add_argument('--out', type=Path, required=True, help='새 원문 출력 폴더')
    collect.add_argument('--system-id', type=int)
    collect.add_argument('--decoder', type=Path)
    collect.add_argument('--duration', type=float, default=0, help='벽시계 초; 0이면 Ctrl+C까지')
    for action in ('replay', 'preprocess'):
        command = commands.add_parser(action, help='닫힌 원문을 순서대로 읽기' if action == 'replay' else '원문→인과적 특징 행')
        command.add_argument('input', type=Path)
        command.add_argument('--out', type=Path, required=True, help='기존 파일을 덮어쓰지 않는 새 JSONL')
        command.add_argument('--source', choices=('auto', 'raw_jsonl', 'ardupilot_bin'), default='auto')
        if action == 'preprocess':
            command.add_argument('--firmware', choices=('px4_gz', 'arducopter_4_4'), required=True)
            command.add_argument('--system-id', type=int)
            command.add_argument('--battery-id', type=int)
            command.add_argument('--battery-cells', type=int, default=4)
    return parser.parse_args()


def interactive(runtime, processor):
    """입력을 기다리는 동안에도 건강 상태를 감시한다. 임의 PX4 shell 명령은 노출하지 않는다."""
    print('명령: status | takeoff | land | fault <1..4> <배율> <SIM초> | clear | pause | resume | quit', flush=True)
    while True:
        runtime.healthy()
        if not select.select([sys.stdin], [], [], .2)[0]:
            continue
        line = sys.stdin.readline()
        if not line:
            return
        try:
            words = shlex.split(line)
            if not words:
                continue
            if words == ['quit']:
                return
            if words == ['status']:
                show({'runtime': runtime.status(), 'preprocess': processor.snapshot()})
            elif words == ['takeoff']:
                takeoff(runtime)
            elif words == ['land']:
                if runtime.paused:
                    runtime.pause(False)
                land(runtime)
            elif words == ['clear']:
                show(runtime.fault())
            elif words == ['pause'] or words == ['resume']:
                runtime.pause(words[0] == 'pause')
            elif len(words) == 4 and words[0] == 'fault':
                if words[1] not in ('1', '2', '3', '4'):
                    raise ValueError('모터 번호는 정확히 1, 2, 3, 4 중 하나여야 합니다')
                show(runtime.fault(int(words[1]), float(words[2]), float(words[3])))
            else:
                raise ValueError('지원하지 않는 명령/인자입니다')
        except ValueError as exc:
            print(str(exc), file=sys.stderr, flush=True)
            # 문법 오류만 고쳐 입력할 수 있다. 실행·통신 오류는 상위에서 종료한다.


def simulate(args):
    """한 run과 실시간 전처리기를 함께 소유한다. Ctrl+C도 착륙/자원 회수 경로를 거친다."""
    processor = Preprocessor('px4_gz', system_id=args.instance + 1)
    runtime = Runtime(args.profile, args.soc, instance=args.instance, gui=args.gui,
                      on_record=processor.consume)
    try:
        show(runtime.start())
        if args.action == 'experiment':
            run_scenario(runtime, args.scenario)
        else:
            interactive(runtime, processor)
    except KeyboardInterrupt:
        runtime.event('operator_stop')
    except BaseException as exc:
        runtime.errors.append(repr(exc))
        raise
    finally:
        outcome = runtime.close()
        processor.finish()
        show({'run_dir': str(runtime.folder), 'cleanup': outcome})
    return 0 if outcome['clean'] else 1


def collect(args):
    """다른 송신자가 구성한 UDP를 저장한다. 이 명령은 PX4 스트림 설정도 보내지 않는다."""
    if not 0 <= args.duration <= 86400:
        raise ValueError('duration은 유한한 0~86400 벽시계 초여야 합니다')
    receiver = Collector(args.host, args.port, args.out, decoder_path=args.decoder, system_id=args.system_id)
    try:
        receiver.start()
        show({'listening': [args.host, receiver.port], 'output': str(args.out), 'receive_only': True})
        start = time.monotonic()
        while args.duration == 0 or time.monotonic() - start < args.duration:
            snapshot = receiver.snapshot()
            if snapshot['error']:
                raise RuntimeError(snapshot['error'])
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        show(receiver.stop())
    return 0


def replay(args):
    """원본 파일을 변경하지 않고 V2 envelope를 새 파일에 기록한다. 합성 수신 시각은 없다."""
    source, destination = args.input.resolve(), args.out.resolve()
    if source == destination or (source.is_dir() and destination.parent == source
                                 and (RAW_FILE.fullmatch(destination.name) or destination.suffix.lower() == '.bin')):
        raise ValueError('출력은 입력 원문/회전 파일과 분리해야 합니다')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    records = iter_records(args.input, source_kind=args.source)
    try:
        with args.out.open('x', encoding='utf-8') as output:
            for record in records:
                output.write(encode_json(record) + '\n')
                count += 1
    finally:
        records.close()
    show({'records': count, 'output': str(args.out)})
    return 0


def main():
    """오류를 비정상 종료로 노출하고 성공 로그를 대신 만들어 주지 않는다."""
    args = arguments()
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.action == 'profiles':
            fields = ('id', 'label_ko', 'capacity_ah', 'pack_resistance_ohm', 'soh_percent')
            show([{key: profile[key] for key in fields} for path in sorted(PROFILE_DIR.glob('cmu_*.json'))
                  for profile in [load_profile(path)]])
        elif args.action in ('simulate', 'experiment'):
            return simulate(args)
        elif args.action == 'collect':
            return collect(args)
        elif args.action == 'replay':
            return replay(args)
        else:
            show(preprocess_file(args.input, args.out, firmware=args.firmware, source_kind=args.source,
                                 system_id=args.system_id, battery_id=args.battery_id,
                                 battery_cells=args.battery_cells))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        show({'ok': False, 'error': f'{type(exc).__name__}: {exc}'})
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
