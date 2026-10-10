"""역할: PX4/Gazebo·수집기·배터리 연결의 시작/종료와 소유권을 한곳에서 관리한다.
입력: 실행 전 프로파일/SOC, GUI 선택. 출력: 독립 run 폴더와 관측/정답/종료 기록.
주요 호출자: main.py, tests/integration.py. 시작만으로 무장하거나 비행하지 않는다.
"""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import stat
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from .battery import load_profile
from .battery_link import BatteryLink
from .collector import Collector
from .processes import Child
from .scenario import GroundStation, land

ROOT = Path(__file__).resolve().parents[1]
RATES = {'HEARTBEAT': 2, 'BATTERY_STATUS': 5, 'EXTENDED_SYS_STATE': 5,
         'LOCAL_POSITION_NED': 10, 'ATTITUDE': 20, 'ACTUATOR_OUTPUT_STATUS': 20,
         'GPS_RAW_INT': 5, 'HIGHRES_IMU': 50, 'VIBRATION': 10,
         'ATTITUDE_TARGET': 10, 'POSITION_TARGET_LOCAL_NED': 10}


def write_json(path, value):
    """run 내부 JSON을 원자 교체한다. 부분 상태를 정상 응답으로 읽는 일을 방지한다."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.writing')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


def free_ports(count):
    """필요한 loopback 포트를 동시에 예약해 서로 다른 번호를 선택한다. bind 성공은 별도 확인한다."""
    sockets = []
    try:
        for _ in range(count):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockets.append(sock)
            sock.bind(('127.0.0.1', 0))
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


class Runtime:
    """한 가상 기체를 소유한다. live world reset과 실행 중 프로파일 교체는 지원하지 않는다."""

    def __init__(self, profile='cmu_vah01_rpt000', initial_soc=.95, *, instance=31, gui=False,
                 output_root=None, on_record=None):
        if type(instance) is not int or not 1 <= instance <= 200:
            raise ValueError('instance는 1~200 정수여야 합니다')
        if not 0 < initial_soc <= 1:
            raise ValueError('initial_soc는 0보다 크고 1 이하여야 합니다')
        self.profile = load_profile(profile)
        self.soc, self.instance, self.gui = initial_soc, instance, gui
        self.output_root = Path(output_root) if output_root else ROOT / 'hermes' / 'runs'
        self.on_record = on_record
        self.source = ROOT / 'external' / 'PX4-Autopilot'
        self.build = ROOT / 'hermes' / 'build'
        self.binary = self.build / 'px4' / 'bin' / 'px4'
        self.native = self.build / 'native'
        self.decoder = ROOT / 'hermes' / 'generated' / 'px4_mavlink.py'
        self.model_name = f'capstone_x500_{instance}'
        self.run_id = uuid.uuid4().hex
        self.children = {}
        self.collector = self.link = self.gcs = self.owner = self.events = None
        self.folder = None
        self.env = {}
        self.ports = {}
        self.ipc = {}
        self.state = 'new'
        self.paused = False
        self.errors = []
        self.cleanup = None

    def _preflight(self):
        """설치 파일과 instance 충돌만 조사한다. 외부 기체 연결·자동 설치는 하지 않는다."""
        for path in (self.binary, self.native / 'libMotorFault.so', self.native / 'gz_feed', self.decoder,
                     self.build / 'px4' / 'etc' / 'init.d-posix' / 'rcS', ROOT / 'config' / 'server.config',
                     self.source / 'Tools/simulation/gz/models/x500_base/model.sdf'):
            if not path.is_file():
                raise FileNotFoundError(f'빌드/설치 산출물 없음: {path}')
        if shutil.which('gz') is None:
            raise RuntimeError('Gazebo Harmonic CLI가 없습니다')
        pins = json.loads((ROOT / 'config' / 'sources.lock.json').read_text())
        head = subprocess.check_output(['git', '-C', str(self.source), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = subprocess.check_output(['git', '-C', str(self.source), 'status', '--porcelain'], text=True)
        if head != pins['px4']['commit'] or dirty:
            raise RuntimeError('PX4 고정 revision/무변경 조건 불일치')
        for path in self._ipc_paths():
            if path.exists() or path.is_symlink():
                raise RuntimeError(f'기존 PX4 IPC를 보존합니다. 소유자를 먼저 확인하세요: {path}')

    def _ipc_paths(self):
        """원본 PX4에 고정된 native IPC 경로다. 프로젝트가 임의 임시 파일을 만드는 곳이 아니다."""
        return [Path(f'/tmp/px4_lock-{self.instance}'), Path(f'/tmp/px4-sock-{self.instance}')]

    def _capture_ipc(self):
        """시작 전 없던 파일의 inode/type을 기록하고, 동일 파일만 종료 후 회수한다."""
        if 'px4' not in self.children:
            return
        for path in self._ipc_paths():
            if path in self.ipc:
                continue
            try:
                info = path.lstat()
            except FileNotFoundError:
                continue
            expected = stat.S_ISREG if 'lock-' in path.name else stat.S_ISSOCK
            if info.st_uid != os.getuid() or not expected(info.st_mode):
                raise RuntimeError(f'예상하지 않은 IPC 소유자/형식: {path}')
            self.ipc[path] = (info.st_dev, info.st_ino)

    def _environment(self):
        """이전 PX4/GZ 설정을 자식에 섞지 않고 캐시·HOME·작업 파일을 run 안으로 격리한다."""
        env = {k: v for k, v in os.environ.items() if not k.startswith(('PX4_', 'GZ_', 'IGN_'))}
        for name in ('home', 'tmp', 'cache', 'config', 'rootfs', 'groundtruth', 'telemetry'):
            (self.folder / name).mkdir()
        env.update({'HOME': str(self.folder / 'home'), 'TMPDIR': str(self.folder / 'tmp'),
                    'XDG_CACHE_HOME': str(self.folder / 'cache'), 'XDG_CONFIG_HOME': str(self.folder / 'config'),
                    'GZ_PARTITION': 'capstone_' + self.run_id, 'GZ_IP': '127.0.0.1',
                    'GZ_SIM_RESOURCE_PATH': str(ROOT / 'models') + ':' + str(self.source / 'Tools/simulation/gz/models'),
                    'GZ_SIM_SYSTEM_PLUGIN_PATH': str(self.native),
                    'GZ_SIM_SERVER_CONFIG_PATH': str(ROOT / 'config' / 'server.config'),
                    'PX4_SYS_AUTOSTART': '4001', 'PX4_GZ_STANDALONE': '1',
                    'PX4_GZ_MODEL_NAME': self.model_name, 'PX4_GZ_WORLD': 'default',
                    'PX4_PARAM_SIM_BAT_ENABLE': '0', 'PX4_PARAM_BAT1_SOURCE': '1',
                    'PX4_PARAM_BAT1_N_CELLS': str(self.profile['series_cells']),
                    'PX4_PARAM_BAT1_CAPACITY': str(round(self.profile['capacity_ah'] * 1000)),
                    'PX4_PARAM_COM_RC_IN_MODE': '4', 'PX4_PARAM_MAV_PROTO_VER': '2',
                    'PX4_PARAM_MIS_TAKEOFF_ALT': '3', 'PX4_PARAM_COM_DISARM_LAND': '2',
                    'PYTHONDONTWRITEBYTECODE': '1'})
        return env

    def event(self, event_name, **fields):
        """시나리오·명령은 정답 파일에만 기록한다. 수집 원문에는 주입하지 않는다."""
        if self.events is not None:
            stamp = self.link.snapshot()['motor'].get('sim_time_s') if self.link else None
            self.events.write(json.dumps({'event': event_name, 'utc_ns': time.time_ns(),
                                          'sim_time_s': stamp, **fields}, ensure_ascii=False,
                                         allow_nan=False) + '\n')
            self.events.flush()

    def command(self, text):
        """이 실행이 시작한 PX4 콘솔 명령을 기록한다. 호출 후 실제 상태 확인이 따로 필요하다."""
        reply = self.children['px4'].command(text)
        self.event('console', command=text, reply=reply)
        return reply

    def parameter(self, name, value):
        """run의 표준 파라미터를 설정하고 숫자 readback까지 확인한다. 원본 파일은 수정하지 않는다."""
        self.command(f'param set {name} {value}')
        reply = self.command(f'param show -q {name}')
        values = re.findall(r'^\s*(-?\d+(?:\.\d+)?)\s*$', reply.replace('\r', ''), flags=re.M)
        if not values or float(values[-1]) != float(value):
            raise RuntimeError(f'파라미터 readback 불일치: {name}: {reply}')

    def healthy(self):
        """자식/기록 오류를 정상처럼 숨기지 않는다. pause는 명시적인 별도 상태다."""
        self._capture_ipc()
        for name, child in self.children.items():
            if child.process.poll() is not None:
                raise RuntimeError(f'{name}가 예상하지 않게 종료됨: {child.process.returncode}')
            if child.error:
                raise RuntimeError(f'{name} 로그 오류: {child.error}')
        if self.collector:
            status = self.collector.snapshot()
            if status.get('error') or status.get('errors'):
                raise RuntimeError('수집 오류: ' + str(status))
        if self.link:
            info = self.link.snapshot()
            if info['error']:
                raise RuntimeError('배터리/모터 경로 오류: ' + info['error'])
            if self.state in ('running', 'resuming') and not self.paused:
                if info['age_wall_s'] is None or info['age_wall_s'] > 5:
                    raise TimeoutError('모터 중계 상태가 5초 이상 도착하지 않았습니다')
                # 재개 중에는 pause()가 새 SIM tick/heartbeat를 제한 시간 안에 요구한다.
                if self.state == 'running':
                    clock_age = info['motor'].get('clock_wall_age_s')
                    if clock_age is None or clock_age > 5:
                        raise TimeoutError('요청하지 않은 시뮬레이션 시계 정지입니다')
                    if not self.collector.latest('HEARTBEAT'):
                        raise TimeoutError('PX4 heartbeat가 오래됐습니다')
                if info['battery'] and info['battery']['soc'] <= 0:
                    raise RuntimeError('배터리 모델이 고갈되어 실험을 종료합니다')
        if self.gcs and self.gcs.error:
            raise RuntimeError('관제 heartbeat 오류: ' + self.gcs.error)
        if self.folder and shutil.disk_usage(self.folder).free < 128 * 1024 * 1024:
            raise OSError('기록 공간 부족으로 실험을 중단합니다')

    def wait(self, predicate, timeout=30, description='상태', *, check_health=True):
        """sim-time이 멈춰도 한 연결/상태 대기는 120초를 넘기지 않는다."""
        if not 0 < timeout <= 120:
            raise ValueError('wait timeout은 0~120초 범위여야 합니다')
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if check_health:
                self.healthy()
            result = predicate()
            if result:
                return result
            time.sleep(.05)
        raise TimeoutError(description + ' 대기 시간 초과')

    def start(self):
        """새 run을 지상 Disarmed로 시작한다. 준비 실패도 close()로 반드시 자원을 회수한다."""
        if self.state != 'new':
            raise RuntimeError('Runtime은 한 run에만 사용할 수 있습니다')
        self._preflight()
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.owner = (self.output_root / f'instance-{self.instance}.lock').open('a+')
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.folder = self.output_root / self.run_id
            self.folder.mkdir()
            self.env = self._environment()
            self.state = 'starting'
            self.events = (self.folder / 'groundtruth' / 'events.jsonl').open('x', encoding='utf-8')
            tree = ET.parse(ROOT / 'config' / 'world.sdf')
            include = ET.SubElement(tree.getroot().find('world'), 'include')
            ET.SubElement(include, 'uri').text = 'model://capstone_x500'
            ET.SubElement(include, 'name').text = self.model_name
            ET.SubElement(include, 'pose').text = '0 0 0.2 0 0 0'
            tree.write(self.folder / 'world.sdf', encoding='utf-8', xml_declaration=True)
            metadata = {'schema': 'capstone.run.v1', 'run_id': self.run_id,
                        'system_id': self.instance + 1, 'model': self.model_name,
                        'profile_id': self.profile['id'], 'initial_soc': self.soc,
                        'px4_revision': 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4',
                        'px4_binary_sha256': hashlib.sha256(self.binary.read_bytes()).hexdigest(),
                        'gcs_heartbeat': 'separate loopback controller; collector receive-only',
                        'rc_policy': 'COM_RC_IN_MODE=4, stick input disabled for autonomous SITL only',
                        'failsafe_policy': 'stock sensor checks and NAV_DLL_ACT retained; no force arm',
                        'stock_battery_caveat': 'receiver loses battery ID and temperature sentinel; temperature excluded',
                        'gui': self.gui, 'groundtruth_not_ai_input': True}
            write_json(self.folder / 'groundtruth' / 'run.json', metadata)
            self.collector = Collector('127.0.0.1', 0, self.folder / 'telemetry',
                                       decoder_path=self.decoder, system_id=self.instance + 1,
                                       component_id=1, on_record=self.on_record)
            self.collector.start()
            output_port, input_port = free_ports(2)
            self.ports = {'telemetry': self.collector.port, 'px4_output': output_port, 'px4_input': input_port}
            self.link = BatteryLink(self.native, self.env, self.folder, self.model_name, self.profile,
                                    self.soc, input_port, self.instance + 1)
            self.link.start()
            self.children['gazebo'] = Child(['gz', 'sim', '-s', '-r', str(self.folder / 'world.sdf')],
                                             self.folder / 'gazebo.log', env=self.env, cwd=self.folder)
            self.children['px4'] = Child([str(self.binary), '-i', str(self.instance),
                                          '-w', str(self.folder / 'rootfs'), str(self.build / 'px4' / 'etc')],
                                         self.folder / 'px4.log', env=self.env, cwd=self.folder / 'rootfs',
                                         console=True)
            self.children['px4'].wait_text('pxh> ', timeout=110)
            self._capture_ipc()
            for key, value in (('SIM_BAT_ENABLE', 0), ('BAT1_SOURCE', 1), ('COM_RC_IN_MODE', 4),
                               ('BAT1_N_CELLS', self.profile['series_cells']),
                               ('BAT1_CAPACITY', round(self.profile['capacity_ah'] * 1000)),
                               ('MIS_TAKEOFF_ALT', 3), ('COM_DISARM_LAND', 2)):
                self.parameter(key, value)
            self.command('mavlink stop-all')
            self.command(f'mavlink start -u {output_port} -o {self.collector.port} -t 127.0.0.1 -m custom -r 100000')
            self.command(f'mavlink start -u {input_port} -o 9 -t 127.0.0.1 -m custom -r 100000')
            for name, hz in RATES.items():
                self.command(f'mavlink stream -u {output_port} -s {name} -r {hz}')
            self.link.enabled.set()
            self.gcs = GroundStation(input_port)
            self.gcs.start()
            required = ('HEARTBEAT', 'BATTERY_STATUS', 'ATTITUDE', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE')
            self.wait(lambda: all(self.collector.latest(name) for name in required),
                      timeout=40, description='PX4의 새 지상 텔레메트리')
            heartbeat = self.collector.latest('HEARTBEAT')
            if heartbeat['base_mode'] & 128:
                raise RuntimeError('시작 기체가 Disarmed가 아닙니다')
            self.wait(lambda: self.link.snapshot()['battery'], description='명령 기반 배터리 모델')
            if self.gui:
                self.children['gui'] = Child(['gz', 'sim', '-g'], self.folder / 'gui.log',
                                             env=self.env, cwd=self.folder)
            self.state = 'running'
            self.event('ready', ports=self.ports, battery_id=self.collector.latest('BATTERY_STATUS')['id'])
            return self.status()
        except BaseException as exc:
            self.errors.append(repr(exc))
            self.close()
            raise

    def gz_service(self, service, request_type, response_type, request):
        """소유 GZ_PARTITION의 공식 서비스만 5초 이내에 호출한다."""
        result = subprocess.run(['gz', 'service', '-s', service, '--reqtype', request_type,
                                 '--reptype', response_type, '--timeout', '3000', '--req', request],
                                env=self.env, cwd=self.folder, capture_output=True, text=True, timeout=5)
        if result.returncode:
            raise RuntimeError('Gazebo 서비스 실패: ' + result.stdout + result.stderr)
        return result.stdout

    def fault(self, motor=None, scale=1.0, duration=1.0):
        """명령 중계를 멈추지 않고 배율만 바꾼다. accepted와 실제 중계 반영을 구분한다."""
        if self.paused:
            raise RuntimeError('pause를 해제한 뒤 고장을 변경하세요')
        if motor is not None:
            if self.profile['id'] != 'cmu_vah01_rpt000':
                raise ValueError('첫 버전에서는 열화 배터리와 모터 고장을 함께 사용하지 않습니다')
            if type(motor) is not int or motor not in (1, 2, 3, 4) or not 0 < scale <= 1 or not 0 < duration <= 120:
                raise ValueError('motor=1~4, 0<scale<=1, 0<duration<=120초가 필요합니다')
        command = 'clear' if motor is None else f'set {motor} {scale} {duration}'
        reply = self.gz_service('/capstone/' + self.model_name + '/fault', 'gz.msgs.StringMsg',
                                'gz.msgs.StringMsg', 'data: ' + json.dumps(command))
        self.event('fault_request', request=command, service_reply=reply)
        # StringMsg의 protobuf text 외피 안에 JSON 문자열 하나가 들어 있다.
        match = re.search(r'^data:\s*("(?:[^"\\]|\\.)*")\s*$', reply, re.M)
        if match is None:
            raise RuntimeError('고장 서비스 응답을 해석할 수 없습니다: ' + reply)
        accepted = json.loads(json.loads(match.group(1)))
        if not accepted.get('ok'):
            raise RuntimeError('고장 요청 거부: ' + str(accepted))
        revision = accepted['accepted_revision']
        # transport의 accepted 문자열만으로 성공을 선언하지 않고 topic readback을 기다린다.
        result = self.wait(lambda: (s if (s := self.link.snapshot()['motor']).get('applied_revision', 0) == revision else None),
                           timeout=5, description='고장 중계 적용 revision')
        if bool(result.get('fault_active')) != (motor is not None):
            raise RuntimeError('고장 적용 상태 readback 불일치: ' + str(result))
        return result

    def pause(self, value):
        """SIM 정지/재개를 확인한다. 재개 때는 새 수신과 시계 진행 후 정상 감시로 복귀한다."""
        recovering = not value and self.state == 'running'
        before_sim = self.link.snapshot()['motor'].get('sim_time_s') if recovering else None
        if recovering:
            self.state = 'resuming'
        try:
            reply = self.gz_service('/world/default/control', 'gz.msgs.WorldControl', 'gz.msgs.Boolean',
                                    'pause: ' + ('true' if value else 'false'))
            if 'data: true' not in reply:
                raise RuntimeError('Gazebo pause 응답 실패: ' + reply)
            self.paused = bool(value)
            # 종료 경로는 기존 오류가 있어도 세계를 풀고 자원 정리를 계속해야 한다.
            self.wait(lambda: self.link.snapshot()['motor'].get('paused') is self.paused,
                      timeout=5, description='실제 pause 상태', check_health=self.state != 'stopping')
            self.event('pause', paused=self.paused)
            if recovering:
                after_ns = time.monotonic_ns()
                self.event('resume_wait', after_monotonic_ns=after_ns, timeout_wall_s=5)

                def resumed():
                    """캐시 값이 같아도 수신 시각이 경계 뒤여야 한다. 기존 시각은 바꾸지 않는다."""
                    motor = self.link.snapshot()['motor']
                    sim = motor.get('sim_time_s')
                    return (motor.get('paused') is False and before_sim is not None and
                            sim is not None and sim > before_sim and
                            self.collector.latest('HEARTBEAT', since_monotonic_ns=after_ns))

                self.wait(resumed, timeout=5, description='재개 후 새 heartbeat와 시계 진행')
                self.state = 'running'
                self.healthy()
                self.event('resume_ready', after_monotonic_ns=after_ns,
                           observed_monotonic_ns=time.monotonic_ns())
        except BaseException as exc:
            if recovering:
                self.state = 'failed'
                self.errors.append(repr(exc))
            raise

    def status(self):
        """사용자 운영 스냅샷이다. AI에는 telemetry/preprocess 경로만 전달한다."""
        return {'state': self.state, 'run_dir': str(self.folder) if self.folder else None,
                'paused': self.paused, 'ports': dict(self.ports),
                'simulation_truth': self.link.snapshot() if self.link else {},
                'telemetry': {name: self.collector.latest(name) for name in RATES} if self.collector else {},
                'errors': list(self.errors)}

    def close(self):
        """착륙 시도→PX4→feed/Gazebo→수집 순서로 정리한다. 실패·강제 종료도 결과에 남긴다."""
        if self.cleanup is not None:
            return self.cleanup
        self.state = 'stopping'
        result = {'children': {}, 'errors': list(self.errors)}
        try:
            self._capture_ipc()
            if self.paused:
                self.pause(False)
            if self.collector and self.collector.latest('HEARTBEAT').get('base_mode', 0) & 128:
                land(self, check_health=False)
        except BaseException as exc:
            result['errors'].append('종료 전 착륙: ' + repr(exc))
        # 의도한 PX4 종료 이후 입력 단절을 새 실험 오류로 기록하지 않는다.
        if self.link:
            self.link.enabled.clear()
            self.link.stopping.set()

        def close_part(name, action, destination):
            """한 자원 정리가 실패해도 다른 소유 자원은 계속 정리한다."""
            try:
                destination[name] = action()
            except BaseException as exc:
                result['errors'].append(name + ' 종료: ' + repr(exc))

        if 'px4' in self.children:
            child = self.children['px4']
            try:
                child.send_line('shutdown')
                child.process.wait(timeout=10)
            except BaseException as exc:
                result['errors'].append('PX4 정상 shutdown: ' + repr(exc))
            close_part('px4', child.stop, result['children'])
        if self.gcs:
            close_part('gcs', self.gcs.stop, result)
        if self.link:
            close_part('battery_link', self.link.stop, result)
        for name in ('gui', 'gazebo'):
            if name in self.children:
                close_part(name, self.children[name].stop, result['children'])
        if self.collector:
            close_part('collector', self.collector.stop, result)
        if result['children'].get('px4', {}).get('group_alive') is False:
            for path, identity in self.ipc.items():
                try:
                    info = path.lstat()
                    if (info.st_dev, info.st_ino) != identity:
                        raise RuntimeError('다른 소유자로 바뀐 IPC는 보존: ' + str(path))
                    path.unlink()
                except FileNotFoundError:
                    pass
                except BaseException as exc:
                    result['errors'].append(repr(exc))
        if self.events:
            close_part('event_file', self.events.close, result)
            self.events = None
        if self.owner:
            close_part('owner_lock', self.owner.close, result)
            self.owner = None
        outcomes = list(result['children'].values())
        if 'battery_link' in result:
            outcomes.append(result['battery_link'].get('feed', {}))
            result['errors'].extend(result['battery_link']['errors'])
        if 'gcs' in result and result['gcs'].get('error'):
            result['errors'].append(result['gcs']['error'])
        for outcome in outcomes:
            if outcome.get('forced') or outcome.get('group_alive') or outcome.get('errors'):
                result['errors'].append('자식 종료 이상: ' + str(outcome))
            if outcome.get('returncode', 0) not in (0, -2, 130):
                result['errors'].append('자식 비정상 exit: ' + str(outcome))
        result['clean'] = not result['errors']
        self.state = 'closed' if result['clean'] else 'failed'
        self.cleanup = result
        if self.folder:
            write_json(self.folder / 'cleanup.json', result)
        return result
