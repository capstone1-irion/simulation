"""역할: 원본 PX4와 V2 native 확장을 별도 build 디렉터리에 빌드하고 디코더를 생성한다.
입력: 고정 원본·.venv·시스템 개발 라이브러리. 출력: hermes/build와 hermes/generated.
주요 호출자: 최종 통합 검증 및 팀원의 명시적인 빌드 명령. 실행 시 비행하지 않는다.
"""
from pathlib import Path
import argparse
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def build(jobs=4):
    """원본을 패치하지 않고 out-of-source 빌드한다. 실패한 단계를 자동 재실행하지 않는다."""
    source = ROOT / 'external' / 'PX4-Autopilot'
    output = ROOT / 'hermes' / 'build'
    generated = ROOT / 'hermes' / 'generated'
    python = ROOT / '.venv' / 'bin' / 'python'
    if not 1 <= jobs <= 16:
        raise ValueError('jobs는 1~16 정수여야 합니다')
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('프로젝트 .venv의 Python 3.12로 실행하세요')
    output.mkdir(parents=True, exist_ok=True)
    generated.mkdir(parents=True, exist_ok=True)
    temp = ROOT / 'hermes' / 'build' / 'tmp'
    temp.mkdir(exist_ok=True)
    env = dict(os.environ, PATH=str(ROOT / 'tools' / 'bin') + ':' + str(python.parent) + ':' + os.environ['PATH'],
               PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1', TMPDIR=str(temp), GIT_TERMINAL_PROMPT='0',
               CCACHE_DIR=str(output / 'ccache'), XDG_CACHE_HOME=str(output / 'cache'))
    commands = [
        ['cmake', '-S', str(source), '-B', str(output / 'px4'), '-G', 'Ninja',
         '-DCONFIG=px4_sitl_default', '-DPYTHON_EXECUTABLE=' + str(python), '-DCMAKE_BUILD_TYPE=RelWithDebInfo'],
        ['cmake', '--build', str(output / 'px4'), '--parallel', str(jobs)],
        ['cmake', '-S', str(ROOT), '-B', str(output / 'native'), '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=RelWithDebInfo'],
        ['cmake', '--build', str(output / 'native'), '--parallel', str(jobs)],
        [str(python), str(python.parent / 'mavgen.py'), '--lang=Python3', '--wire-protocol=2.0',
         '--output=' + str(generated / 'px4_mavlink.py'),
         str(source / 'src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml')],
    ]
    for command in commands:
        print('BUILD:', ' '.join(command), flush=True)
        subprocess.run(command, cwd=ROOT, env=env, check=True, timeout=1200)
    return {'px4': str(output / 'px4' / 'bin' / 'px4'),
            'native': str(output / 'native'), 'decoder': str(generated / 'px4_mavlink.py')}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='PX4/V2 빌드; 검증 실행과 비행은 별도')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    build(args.jobs)
