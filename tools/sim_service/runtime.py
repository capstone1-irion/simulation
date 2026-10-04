"""CMU 프로파일 PX4 SITL 서비스. 수집은 수신 전용, 제어는 소유 콘솔/Gazebo에만 한정한다."""
import math
import re
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
import uuid

from mavlink_collector.collector import safe_json

from .control_api import ControlServer
from .model_config import MODEL_NAME, validate_catalog, write_model, write_world
from .processes import PipeProcess, PtyConsole, RotatingLog

REPO = Path(__file__).resolve().parents[2]
DEFAULT_CATALOG = REPO / 'profiles/battery/cmu_runtime/catalog.json'
PATH_KEYS = ('px4_source', 'px4_binary', 'plugin_build', 'decoder', 'ipc_dir', 'output_root', 'state_dir', 'catalog')
# 송신 설정과 최신값 공개 목록의 단일 기준. 목표 Hz이며 실제 수신률 보장은 아니다.
TELEMETRY_RATES = {'HEARTBEAT': 2, 'ESC_STATUS': 20, 'BATTERY_STATUS': 5,
                   'LOCAL_POSITION_NED': 10, 'EXTENDED_SYS_STATE': 5, 'ATTITUDE': 10,
                   'POSITION_TARGET_LOCAL_NED': 10, 'ATTITUDE_TARGET': 10, 'ACTUATOR_OUTPUT_STATUS': 20}


def atomic_json(path, value):
    """유한 JSON을 같은 디렉터리의 임시 파일에서 교체하여 부분 상태 파일 읽기를 방지한다."""
    path = Path(path)
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    temporary = path.with_name(path.name + '.writing-' + str(threading.get_ident()))
    try:
        with temporary.open('w', encoding='utf-8') as handle:
            handle.write(text)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def load_config(path):
    """설정 상대 경로는 simulation 저장소 기준으로 해석한다; 파일 생성/프로세스 시작은 하지 않는다."""
    return validate_config(json.loads(Path(path).read_text(encoding='utf-8')))


def validate_config(value):
    """설정 객체를 파일 생성 전에 검사하고 절대 경로 복사본을 반환한다."""
    if not isinstance(value, dict):
        raise ValueError('Service config must be an object')
    data = dict(value)
    if not isinstance(data, dict) or data.get('schema') != 'arion.sim_service.v1':
        raise ValueError('Service configuration schema mismatch')
    for key in PATH_KEYS:
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError('Missing path: ' + key)
        p = Path(data[key]).expanduser()
        data[key] = str((p if p.is_absolute() else REPO / p).resolve())
    if type(data.get('instance')) is not int or not 1 <= data['instance'] <= 250:
        raise ValueError('PX4 instance must be 1..250')
    if type(data.get('rotate_bytes')) is not int or not 65536 <= data['rotate_bytes'] <= 1073741824:
        raise ValueError('rotate_bytes must be 64KiB..1GiB')
    if len(os.fsencode(Path(data['state_dir']) / 'control.sock')) >= 108:
        raise ValueError('Use a shorter state_dir for UNIX control socket')
    return data


def check_runtime_files(config):
    """선택한 바이너리·모델·디코더와 프로젝트 내부 IPC를 검사하고 stock /tmp 실행은 거부한다."""
    for key in ('px4_binary', 'decoder', 'catalog'):
        if not Path(config[key]).is_file():
            raise FileNotFoundError(key + ': ' + config[key])
    for name in ('libArionCoupledPowerPlugin.so', 'arion_battery_sender', 'arion_power_control'):
        if not (Path(config['plugin_build']) / name).is_file():
            raise FileNotFoundError('Build target is missing: ' + name)
    if not os.access(config['px4_binary'], os.X_OK):
        raise ValueError('PX4 binary is not executable')
    ipc = Path(config['ipc_dir'])
    needles = [b'/tmp/px4_lock', b'/tmp/px4-sock-',
               *(os.fsencode(ipc / prefix) for prefix in ('px4_lock', 'px4-sock-'))]
    found = [False] * len(needles)
    overlap = max(map(len, needles)) - 1
    digest = hashlib.sha256()
    with Path(config['px4_binary']).open('rb') as binary:
        magic = binary.read(4)
        digest.update(magic)
        tail = magic
        while chunk := binary.read(1024 * 1024):
            digest.update(chunk)
            window = tail + chunk
            found = [seen or needle in window for seen, needle in zip(found, needles)]
            # 접두사가 읽기 경계를 가로질러도 전체 파일 검사와 같은 결과를 보존한다.
            tail = window[-overlap:]
    if magic != b'\x7fELF' or any(found[:2]):
        raise ValueError('A project-local SITL overlay ELF is required')
    if not all(found[2:]):
        raise ValueError('ELF IPC prefix does not match ipc_dir')
    if len(os.fsencode(ipc / ('px4-sock-' + str(config['instance'])))) >= 108:
        raise ValueError('PX4 IPC socket path is too long')
    for p in (ipc / f'px4_lock-{config["instance"]}', ipc / f'px4-sock-{config["instance"]}'):
        if p.exists() or p.is_symlink():
            raise RuntimeError('Existing PX4 instance IPC; never remove an unknown owner: ' + str(p))
    px4 = Path(config['px4_source'])
    for p in (px4 / 'Tools/simulation/gz/worlds/default.sdf', px4 / 'Tools/simulation/gz/models/x500/model.sdf',
              Path(config['px4_binary']).parent.parent / 'etc/init.d-posix/rcS',
              REPO / 'config/coupled-headless.config'):
        if not p.is_file():
            raise FileNotFoundError(str(p))
    if shutil.which('gz') is None:
        raise RuntimeError('Installed Gazebo Harmonic CLI is required; no automatic installation')
    return digest.hexdigest()


def reserve_ports(count):
    """OS 임시 UDP 포트를 동시에 확보한 뒤 번호를 반환한다. PX4 bind 실패는 별도로 감지한다."""
    holders = []
    try:
        for _ in range(count):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            holders.append(s)
            s.bind(('127.0.0.1', 0))
        return [s.getsockname()[1] for s in holders]
    finally:
        for s in holders:
            s.close()


class Runtime:
    """한 가상 기체·수신기·송신기를 소유한다. 시작만으로 arm/비행/외부 장비 접속을 하지 않는다."""
    def __init__(self, config, catalog, profile_id=None):
        """실행 구성을 저장하고 아직 자원을 만들지 않는다; 생성자는 순수 설정 단계다."""
        self.config, self.catalog = dict(config), validate_catalog(catalog)
        self.profile_id = profile_id or self.catalog['default_profile_id']
        self.profiles = {p['id']: p for p in self.catalog['profiles']}
        if self.profile_id not in self.profiles:
            raise ValueError('Unknown CMU profile')
        self.initial_profile_id = self.profile_id
        self.state, self.paused = 'not_started', False
        self.stop_requested = threading.Event()
        self.action_lock, self.power_lock = threading.Lock(), threading.Lock()
        self.children, self.cleanup, self.errors, self.power_state = {}, {}, [], {}
        self.recorder = self.api = self.event_log = self.owner_lock = self.folder = None
        self.env, self.ports = {}, {}
        self.started_monotonic, self.started_utc_ns = time.monotonic(), time.time_ns()
        self.run_id = uuid.uuid4().hex[:12]
        self.last_health = None
        self.px4_epoch = 0

    def _require_disarmed(self):
        """최신 PX4 heartbeat가 명확히 Disarmed일 때만 배터리 교체/reset/종료를 허용한다."""
        heartbeat = {} if self.recorder is None else self.recorder.latest('HEARTBEAT', max_age_s=3)
        if not heartbeat:
            raise RuntimeError('Fresh Disarmed heartbeat required; resume/wait before changing the model')
        if heartbeat.get('base_mode', 0) & 128:
            raise RuntimeError('Disarm required: land first; no in-flight battery replacement or world reset')

    def event(self, kind, **values):
        """명령·프로파일·이상 정답을 별도 groundtruth 로그에 기록한다; raw MAVLink에 섞지 않는다."""
        if self.event_log is not None:
            row = {'kind': kind, 'utc_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(), **values}
            try:
                self.event_log.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            except OSError as error:
                self.errors.append('Groundtruth log I/O failed: ' + repr(error))
                self.stop_requested.set()

    def command(self, text):
        """내부 화이트리스트 동작이 만든 PX4 명령만 소유 PTY로 보내고 응답을 별도 기록한다."""
        reply = self.children['px4'].command(text)
        self.event('px4_console', command=text, response=reply)
        return reply

    def _healthy(self):
        """필수 자식 종료/기록기 I/O 오류를 즉시 실패로 노출한다; 재무장 자동 재시작은 하지 않는다."""
        for name, child in list(self.children.items()):
            if name == 'gui' or (name in ('px4', 'sender') and self.state == 'world_resetting'):
                continue
            if child.poll() is not None:
                raise RuntimeError(name + ' exited unexpectedly: ' + str(child.poll()))
            if child.error:
                raise RuntimeError(name + ' logging error: ' + child.error)
        if self.recorder is not None and self.recorder.snapshot().get('errors'):
            raise RuntimeError('MAVLink recorder I/O failed: ' + repr(self.recorder.snapshot()['errors']))

    def wait_for(self, predicate, seconds, description):
        """clock pause와 무관한 wall-clock deadline 안에서 상태를 기다린다; 120초 상한을 지킨다."""
        until = time.monotonic() + min(seconds, 115)
        while time.monotonic() < until:
            self._healthy()
            if self.stop_requested.is_set():
                raise RuntimeError('Service shutdown requested')
            result = predicate()
            if result:
                return result
            time.sleep(.1)
        raise TimeoutError(description + ' timed out')

    def power(self, command='status', *, wait=True):
        """Gazebo 요청의 accepted와 실제 적용 revision을 구분하고 read-back된 상태만 반환한다."""
        with self.power_lock:
            def call(text):
                """소유 GZ_PARTITION의 control CLI를 5초 상한으로 실행하고 JSON 오류를 전파한다."""
                result = subprocess.run([str(Path(self.config['plugin_build']) / 'arion_power_control'),
                    '--service', self.power_service, '--command', text, '--timeout-ms', '2000'],
                    env=self.env, cwd=self.folder, capture_output=True, text=True, timeout=5)
                if result.returncode:
                    raise RuntimeError('Power control: ' + result.stderr[-2000:] + result.stdout[-2000:])
                value = json.loads(result.stdout)
                if not value.get('ok'):
                    raise RuntimeError('Power control rejected: ' + str(value))
                return value
            result = call(command)
            accepted = result.get('accepted_revision')
            if accepted is not None and wait and result.get('applied_revision', -1) < accepted:
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    result = call('status')
                    if result.get('applied_revision', -1) >= accepted:
                        break
                    time.sleep(.05)
                else:
                    raise TimeoutError('Power command accepted but not applied; simulation may be paused')
            if 'profile_id' in result:
                self.power_state = result
                self.profile_id = result['profile_id']
            return result

    def world(self, request):
        """소유 partition의 공식 WorldControl 서비스만 호출하고 Boolean 응답을 읽는다."""
        result = subprocess.run(['gz', 'service', '-s', '/world/default/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', request], env=self.env, cwd=self.folder,
            capture_output=True, text=True, timeout=5)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError('Gazebo world control failed: ' + result.stdout + result.stderr)
        self.event('world_control', request=request, response=result.stdout)

    def telemetry(self):
        """원문/freshness를 JSON-safe 사본으로 반환한다; 비유한 값은 태그로 보존하고 정답은 넣지 않는다."""
        return {'utc_ns': time.time_ns(), 'receive_only': True,
                'messages': {name: safe_json(self.recorder.latest(name, max_age_s=3)) if self.recorder else {}
                             for name in TELEMETRY_RATES}}

    def status(self):
        """운영 상태·PID·데이터 경로·수신 건강을 반환한다. 이 운영 화면 전체를 AI 입력으로 쓰지 않는다."""
        return {'ok': not self.errors, 'state': self.state, 'run_id': self.run_id,
                'service_pid': os.getpid(), 'px4_epoch': self.px4_epoch, 'elapsed_wall_s': time.monotonic() - self.started_monotonic,
                'run_dir': None if self.folder is None else str(self.folder), 'paused': self.paused,
                'selected_profile': self.profile_id, 'power': dict(self.power_state),
                'ports': dict(self.ports), 'owned_pids': {n: p.process.pid for n, p in self.children.items()},
                'recorder': {} if self.recorder is None else self.recorder.snapshot(),
                'telemetry': self.telemetry(), 'errors': list(self.errors), 'cleanup': dict(self.cleanup),
                'physics_scope': 'one-way telemetry/electrical-load model; no actuator/thrust perturbation'}

    def _takeoff(self, height):
        """정상 preflight를 통과한 SITL만 목표 m로 이륙시킨다; force-arm이나 안전 검사 해제는 없다."""
        self._require_disarmed()
        self.wait_for(lambda: self.recorder.latest('LOCAL_POSITION_NED'), 25, 'position')
        self.wait_for(lambda: 'Preflight check: OK' in self.command('commander check'), 30, 'preflight')
        self.command('param set MIS_TAKEOFF_ALT ' + str(height))
        baseline = self.recorder.latest('LOCAL_POSITION_NED')['z']
        self.command('commander takeoff')
        def hovering():
            """PX4 무장·공중 상태·상대고도와 수직 속도를 동시에 확인한다."""
            hb = self.recorder.latest('HEARTBEAT')
            position = self.recorder.latest('LOCAL_POSITION_NED')
            landed = self.recorder.latest('EXTENDED_SYS_STATE')
            return bool(hb.get('base_mode', 0) & 128) and landed.get('landed_state') == 2 and \
                baseline - position.get('z', baseline) >= height - .6 and abs(position.get('vz', 100)) < .4
        self.wait_for(hovering, 40, 'takeoff/hover')
        return {'height_m': baseline - self.recorder.latest('LOCAL_POSITION_NED')['z']}

    def _land(self):
        """일반 land 명령 후 자동 Disarm을 기다린다; 공중 강제 disarm은 하지 않는다."""
        if self.paused:
            self.world('pause: false')
            self.paused = False
            self.wait_for(lambda: self.recorder.latest('HEARTBEAT'), 8, 'resumed heartbeat')
        self.command('commander land')
        def landed():
            """최신 landed-state와 heartbeat 모두 지상·Disarmed일 때 착륙 완료로 본다."""
            hb = self.recorder.latest('HEARTBEAT')
            return bool(hb) and not hb.get('base_mode', 0) & 128 and self.recorder.latest('EXTENDED_SYS_STATE').get('landed_state') == 1
        self.wait_for(landed, 50, 'landing/disarm')
        return {'disarmed': True}

    def dispatch(self, payload):
        """읽기는 병렬, 상태 변경은 하나씩 수행한다. 성공은 실제 상태 read-back 후에만 기록한다."""
        request = validate_action(payload)
        action = request['action']
        if action == 'status':
            return self.status()
        if action == 'telemetry':
            return {'ok': True, **self.telemetry()}
        if action == 'profiles':
            return {'ok': True, 'selected_profile': self.profile_id, 'profiles': self.catalog['profiles']}
        if self.state != 'running':
            raise RuntimeError('Service is not ready: ' + self.state)
        if not self.action_lock.acquire(blocking=False):
            raise RuntimeError('Another control operation is in progress')
        try:
            if action in ('profile', 'reset', 'restore', 'world-reset', 'stop'):
                self._require_disarmed()
            if self.paused and action not in ('resume', 'land'):
                raise RuntimeError('Resume the simulation before this action')
            self.event('action_requested', request=request)
            if action == 'stop':
                self.state = 'stopping'
                self.stop_requested.set()
                result = {'stopping': True}
            elif action == 'takeoff':
                result = self._takeoff(request['height'])
            elif action == 'land':
                result = self._land()
            elif action == 'motor-fault':
                result = self.power(f'fault {request["motor"] - 1} {request["current_scale"]} {request["rpm_scale"]}')
            elif action in ('clear', 'reset', 'restore'):
                result = self.power(action)
                if action == 'restore':
                    self.command('param set BAT1_CAPACITY ' + str(round(self.profiles[self.profile_id]['capacity_ah'] * 1000)))
            elif action == 'profile':
                if request['id'] not in self.profiles:
                    raise ValueError('Unknown profile: ' + request['id'])
                result = self.power('profile ' + request['id'])
                self.command('param set BAT1_CAPACITY ' + str(round(self.profiles[self.profile_id]['capacity_ah'] * 1000)))
            elif action in ('pause', 'resume'):
                self.world('pause: ' + ('true' if action == 'pause' else 'false'))
                self.paused = action == 'pause'
                result = {'paused': self.paused, 'power': self.power()}
            elif action == 'world-reset':
                result = self._world_reset()
            elif action == 'gui':
                if 'gui' in self.children and self.children['gui'].poll() is None:
                    result = {'pid': self.children['gui'].process.pid}
                elif not self.env.get('DISPLAY') and not self.env.get('WAYLAND_DISPLAY'):
                    raise RuntimeError('No local desktop display available for Gazebo GUI')
                else:
                    self.children['gui'] = PipeProcess(['gz', 'sim', '-g'], self.folder / ('gui-' + str(time.time_ns()) + '.log'),
                                                       env=self.env, cwd=self.folder, log_limit=self.config['rotate_bytes'])
                    result = {'pid': self.children['gui'].process.pid, 'launched_not_window_verified': True}
            else:
                raise ValueError('Unhandled action')
            self.event('action_applied', request=request, result=result)
            return {'ok': True, 'action': action, 'result': result}
        except Exception as error:
            self.event('action_error', request=request, error=repr(error))
            raise
        finally:
            self.action_lock.release()

    def _start_px4(self):
        """현재 clock 세대에 새 PX4를 붙이고 새 메시지별 카운터와 Disarmed를 확인한다; 자동 Arm은 없다."""
        cfg = self.config
        output_port, input_port = self.ports['px4_output'], self.ports['battery_input']
        counts = self.recorder.snapshot()['saved_by_type']
        env = dict(self.env, PX4_PARAM_BAT1_CAPACITY=str(round(self.profiles[self.profile_id]['capacity_ah'] * 1000)))
        self.children['px4'] = PtyConsole([cfg['px4_binary'], '-i', str(cfg['instance']), '-w', str(self.folder / 'rootfs'),
            str(Path(cfg['px4_binary']).parent.parent / 'etc')], self.folder / ('px4.log' if self.px4_epoch == 0 else f'px4-epoch{self.px4_epoch}.log'), env=env,
            cwd=self.folder / 'rootfs', log_limit=cfg['rotate_bytes'])
        self.children['px4'].wait_ready(timeout=100)
        if 'Disarmed' not in self.command('commander status'):
            raise RuntimeError('SITL did not start Disarmed')
        # rcS may overwrite environment defaults. Read back the explicit local-SITL policy.
        for name, value in [('NAV_DLL_ACT', 0), ('COM_RC_IN_MODE', 4), ('COM_DISARM_LAND', 2),
                            ('BAT1_CAPACITY', round(self.profiles[self.profile_id]['capacity_ah'] * 1000))]:
            self.command(f'param set {name} {value}')
            self.command('param show ' + name)
        self.command('mavlink stop-all')
        self.command(f'mavlink start -u {output_port} -o {self.recorder.port} -t 127.0.0.1 -m custom -r 100000')
        self.command(f'mavlink start -u {input_port} -o 9 -t 127.0.0.1 -m custom -r 100000')
        for name, hz in TELEMETRY_RATES.items():
            self.command(f'mavlink stream -u {output_port} -s {name} -r {hz}')
        topic = '/arion/' + self.model_instance
        self.command('arion_motor_bridge start -t ' + topic + '/motor_state')
        self.command('arion_motor_bridge status')
        self.command('mavlink status')
        for name in ('SIM_BAT_ENABLE', 'BAT1_SOURCE', 'SIM_GZ_ESC_ST', 'BAT1_CAPACITY'):
            self.command('param show ' + name)
        topic = '/arion/' + self.model_instance
        self.children['sender'] = PipeProcess([str(Path(cfg['plugin_build']) / 'arion_battery_sender'),
            '--send', '--topic', topic + '/battery_state', '--px4-port', str(input_port),
            '--system-id', str(cfg['instance'] + 1), '--duration', '0', '--stale-policy', 'wait'],
            self.folder / ('sender.log' if self.px4_epoch == 0 else f'sender-epoch{self.px4_epoch}.log'), env=self.env, cwd=self.folder, log_limit=cfg['rotate_bytes'])
        names = ('BATTERY_STATUS', 'ESC_STATUS', 'HEARTBEAT', 'LOCAL_POSITION_NED', 'EXTENDED_SYS_STATE')
        def fresh_epoch():
            """reset 전 cache가 아니라 새 PX4의 각 필수 메시지가 도착했는지 검사한다."""
            current = self.recorder.snapshot()['saved_by_type']
            return all(current.get(n, 0) > counts.get(n, 0) and self.recorder.latest(n) for n in names)
        self.wait_for(fresh_epoch, 25, 'new PX4 epoch telemetry')
        self._require_disarmed()

    def _stop_px4_for_reset(self):
        """시계 역행 전에 sender/PX4를 순서대로 회수한다; Gazebo/공용 수집기는 계속 살아 있다."""
        sender = self.children['sender']
        sender_outcome = sender.stop()
        del self.children['sender']
        self.cleanup['sender_epoch_' + str(self.px4_epoch)] = sender_outcome
        if sender_outcome.get('returncode') != 0 or sender_outcome.get('forced_kill') or sender_outcome.get('group_alive') or sender_outcome.get('error'):
            raise RuntimeError('Cannot reset after unclean sender shutdown: ' + repr(sender_outcome))
        child = self.children['px4']
        child.send_line('shutdown')
        child.process.wait(timeout=8)
        outcome = child.stop()
        del self.children['px4']
        self.cleanup['px4_epoch_' + str(self.px4_epoch)] = outcome
        self.event('px4_epoch_closed', epoch=self.px4_epoch, cleanup=outcome)
        if outcome.get('returncode') != 0 or outcome.get('forced_kill') or outcome.get('group_alive') or outcome.get('error'):
            raise RuntimeError('Cannot reset after unclean PX4 shutdown: ' + repr(outcome))
        for name in (f'px4_lock-{self.config["instance"]}', f'px4-sock-{self.config["instance"]}'):
            (Path(self.config['ipc_dir']) / name).unlink(missing_ok=True)

    def _world_reset(self):
        """기체를 초기 world에 보존하고 세계 reset 전후 PX4 clock 세대를 분리한다. 수신 공백은 숨기지 않는다."""
        before = self.power()
        self.state = 'world_resetting'
        try:
            self._stop_px4_for_reset()
            self.world('reset: {all: true}')
            result = self.wait_for(lambda: (s if (s := self.power()).get('reset_count', 0) > before.get('reset_count', 0) else None),
                                   15, 'world reset application')
            self.px4_epoch += 1
            self._start_px4()
            self._require_disarmed()
            self.state = 'running'
            return {**result, 'px4_epoch': self.px4_epoch, 'telemetry_gap_expected': True}
        except Exception as error:
            self.errors.append('World reset failed: ' + repr(error))
            self.state = 'failed'
            self.stop_requested.set()
            raise

    def start(self):
        """검증된 프로젝트 구성으로 Gazebo/PX4/무기한 sender/공용 수신기를 시작한다; 무장은 하지 않는다."""
        from mavlink_collector.collector import MavlinkRecorder
        binary_sha = check_runtime_files(self.config)
        cfg = self.config
        state_dir = Path(cfg['state_dir'])
        state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.owner_lock = (state_dir / 'owner.lock').open('a+')
        try:
            fcntl.flock(self.owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.owner_lock.close()
            self.owner_lock = None
            raise RuntimeError('Another service owns this state directory')
        if (state_dir / 'control.sock').exists():
            raise RuntimeError('Stale control socket exists; inspect the prior PID before removing it')
        Path(cfg['output_root']).mkdir(parents=True, exist_ok=True)
        Path(cfg['ipc_dir']).mkdir(parents=True, exist_ok=True)
        self.folder = Path(tempfile.mkdtemp(prefix='cmu-' + self.run_id + '-', dir=cfg['output_root']))
        for name in ('rootfs', 'home', 'tmp', 'cache', 'config', 'models'):
            (self.folder / name).mkdir()
        self.event_log = RotatingLog(self.folder / 'groundtruth.events.jsonl', cfg['rotate_bytes'])
        write_model(self.folder / 'models', self.catalog, self.profile_id)
        self.state = 'starting'
        self.model_instance = MODEL_NAME + '_' + str(cfg['instance'])
        world_file = write_world(Path(cfg['px4_source']) / 'Tools/simulation/gz/worlds/default.sdf',
                                 self.folder / 'world.sdf', self.model_instance)
        self.power_service = '/arion/' + self.model_instance + '/power_control'
        px4 = Path(cfg['px4_source'])
        self.env = os.environ.copy()
        self.env.update({'GZ_PARTITION': 'arion_cmu_' + self.run_id, 'GZ_IP': '127.0.0.1',
            'HEADLESS': '1', 'HOME': str(self.folder / 'home'), 'TMPDIR': str(self.folder / 'tmp'),
            'XDG_CACHE_HOME': str(self.folder / 'cache'), 'XDG_CONFIG_HOME': str(self.folder / 'config'),
            'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_' + MODEL_NAME,
            'PX4_GZ_MODEL_NAME': self.model_instance,
            'PX4_GZ_WORLD': 'default', 'PX4_GZ_STANDALONE': '1', 'PX4_GZ_MODELS': str(self.folder / 'models'),
            'PX4_GZ_WORLDS': str(px4 / 'Tools/simulation/gz/worlds'),
            'GZ_SIM_RESOURCE_PATH': str(self.folder / 'models') + ':' + str(px4 / 'Tools/simulation/gz/models'),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': cfg['plugin_build'],
            'GZ_SIM_SERVER_CONFIG_PATH': str(REPO / 'config/coupled-headless.config'),
            'PX4_PARAM_SIM_BAT_ENABLE': '0', 'PX4_PARAM_BAT1_SOURCE': '1', 'PX4_PARAM_BAT1_N_CELLS': '4',
            'PX4_PARAM_BAT1_CAPACITY': str(round(self.profiles[self.profile_id]['capacity_ah'] * 1000)),
            'PX4_PARAM_SIM_GZ_ESC_ST': '0', 'PX4_PARAM_COM_RC_IN_MODE': '4',
            'PX4_PARAM_NAV_DLL_ACT': '0', 'PX4_PARAM_MIS_TAKEOFF_ALT': '3',
            'PX4_PARAM_COM_DISARM_LAND': '2', 'PX4_PARAM_MAV_PROTO_VER': '2'})
        metadata = {'schema': 'arion.sim_run.v1', 'run_id': self.run_id, 'started_utc_ns': self.started_utc_ns,
                    'config': cfg, 'catalog': self.catalog, 'initial_profile': self.profile_id,
                    'px4_binary_sha256': binary_sha,
                    'catalog_sha256': hashlib.sha256(Path(cfg['catalog']).read_bytes()).hexdigest(),
                    'environment': {k: v for k, v in self.env.items() if k.startswith(('PX4_', 'GZ_', 'HEADLESS'))},
                    'physical_scope': 'telemetry/electrical-load perturbations only; original propulsion intact',
                    'control': 'owned local SITL console only; no force arm/disarm', 'collector_receive_only': True,
                    'groundtruth_not_ai_input': ['catalog', 'initial_profile', 'groundtruth.events.jsonl']}
        atomic_json(self.folder / 'run.metadata.json', metadata)
        atomic_json(state_dir / 'status.json', self.status())
        self.recorder = MavlinkRecorder(Path(cfg['decoder']), self.folder / 'telemetry',
                                       source_system=cfg['instance'] + 1, source_component=1,
                                       quality_profile='arion_cmu_v1', rotate_bytes=cfg['rotate_bytes'])
        self.recorder.start()
        output_port, input_port = reserve_ports(2)
        self.ports = {'receiver': self.recorder.port, 'px4_output': output_port, 'battery_input': input_port}
        self.children['gazebo'] = PipeProcess(['gz', 'sim', '-s', '-r', str(world_file)],
            self.folder / 'gazebo.log', env=self.env, cwd=self.folder, log_limit=cfg['rotate_bytes'])
        self._start_px4()
        self.power()
        if self.power_state.get('profile_id') != self.initial_profile_id:
            raise RuntimeError('Initial profile mismatch')
        self.api = ControlServer(state_dir / 'control.sock', self.dispatch).start()
        self.state = 'running'
        self.event('service_ready', ports=self.ports, model=self.model_instance)
        atomic_json(state_dir / 'status.json', self.status())
        return self.status()

    def run_forever(self):
        """사용자 중지까지 상태/디스크/자식·수신기 오류를 감시한다. 종료 뒤에도 자동 재무장하지 않는다."""
        previous = {}
        def request_stop(signum, frame):
            """신호 핸들러에서는 종료 플래그만 세우고 복잡한 cleanup은 주 실행 흐름에 맡긴다."""
            self.stop_requested.set()
        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous[signum] = signal.signal(signum, request_stop)
            self.start()
            print(json.dumps({'state': 'running', 'run_dir': str(self.folder), 'pid': os.getpid()}, ensure_ascii=False), flush=True)
            while not self.stop_requested.wait(.5):
                self._healthy()
                if shutil.disk_usage(self.folder).free < 64 * 1024 * 1024:
                    raise RuntimeError('Less than 64MiB disk remains; stopping instead of silently losing records')
                snapshot = self.status()
                health = bool(snapshot['telemetry']['messages']['BATTERY_STATUS']) and bool(snapshot['telemetry']['messages']['ESC_STATUS'])
                if health != self.last_health:
                    self.event('telemetry_health', fresh=health, paused=self.paused)
                    self.last_health = health
                atomic_json(Path(self.config['state_dir']) / 'status.json', snapshot)
        except Exception as error:
            self.errors.append(type(error).__name__ + ': ' + str(error))
            self.event('service_error', error=self.errors[-1])
        finally:
            self.close()
            for signum, handler in previous.items():
                signal.signal(signum, handler)
        return 0 if not self.errors else 1

    def close(self):
        """정상 착륙을 우선 시도한 뒤 소유 자원만 회수한다. 강제 종료/잔류 그룹을 성공으로 숨기지 않는다."""
        self.state = 'stopping'
        if self.api is not None:
            self.api.close()
        with self.action_lock:
            if self.paused and 'gazebo' in self.children and self.children['gazebo'].poll() is None:
                try:
                    self.world('pause: false')
                    self.paused = False
                except Exception as error:
                    self.errors.append('Resume during shutdown: ' + repr(error))
            px4 = self.children.get('px4')
            if px4 is not None and px4.poll() is None:
                try:
                    hb = {} if self.recorder is None else self.recorder.latest('HEARTBEAT', max_age_s=15)
                    if hb.get('base_mode', 0) & 128:
                        # The normal wait helper refuses a stop request; shutdown landing uses an independent bounded wait.
                        self.command('commander land')
                        deadline = time.monotonic() + 45
                        while time.monotonic() < deadline:
                            hb = self.recorder.latest('HEARTBEAT')
                            if hb and not hb.get('base_mode', 0) & 128:
                                break
                            time.sleep(.2)
                        else:
                            self.errors.append('Normal landing not confirmed before simulator shutdown')
                    px4.send_line('shutdown')
                    try:
                        px4.process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        self.errors.append('PX4 shutdown console did not exit in 8 seconds')
                except Exception as error:
                    self.errors.append('PX4 shutdown: ' + repr(error))
            for name in ('px4', 'sender', 'gui', 'gazebo'):
                child = self.children.get(name)
                if child is None:
                    continue
                try:
                    result = child.stop()
                    self.cleanup[name] = result
                    if result.get('returncode') != 0 or result.get('forced_kill') or result.get('group_alive') or result.get('error'):
                        self.errors.append(name + ' unclean shutdown: ' + repr(result))
                except BaseException as error:
                    self.errors.append(name + ' cleanup interrupted: ' + repr(error))
                    try:
                        self.cleanup[name] = child.stop(timeout=2)
                    except BaseException as retry:
                        self.errors.append(name + ' cleanup retry: ' + repr(retry))
            if self.recorder is not None:
                try:
                    self.recorder.close()
                    health = self.recorder.snapshot()
                    if health['errors'] or health['worker_alive']:
                        self.errors.append('Recorder unclean shutdown: ' + repr({
                            'errors': health['errors'], 'worker_alive': health['worker_alive']}))
                except Exception as error:
                    self.errors.append('Recorder close: ' + repr(error))
            if self.event_log is not None:
                try:
                    self.event('service_stopped', cleanup=self.cleanup, errors=self.errors)
                finally:
                    self.event_log.close()
            # Only the exact fresh instance paths owned by this run may be removed after all owned groups stopped.
            if self.folder is not None and self.cleanup.get('px4') and not self.cleanup['px4'].get('group_alive'):
                for name in (f'px4_lock-{self.config["instance"]}', f'px4-sock-{self.config["instance"]}'):
                    (Path(self.config['ipc_dir']) / name).unlink(missing_ok=True)
            self.state = 'failed' if self.errors else 'stopped'
            if self.folder is not None:
                snapshot = self.status()
                atomic_json(self.folder / 'service.summary.json', snapshot)
                atomic_json(Path(self.config['state_dir']) / 'status.json', snapshot)
            if self.owner_lock is not None:
                self.owner_lock.close()
                self.owner_lock = None



def validate_action(payload):
    """공개 제어의 허용 키/범위를 검사한다; arbitrary console/shell/실기체 주소를 받지 않는다."""
    allowed = {
        'status': set(), 'telemetry': set(), 'profiles': set(), 'clear': set(),
        'reset': set(), 'restore': set(), 'world-reset': set(), 'pause': set(),
        'resume': set(), 'land': set(), 'stop': set(), 'gui': set(),
        'takeoff': {'height'}, 'profile': {'id'},
        'motor-fault': {'motor', 'current_scale', 'rpm_scale'},
    }
    if not isinstance(payload, dict) or payload.get('action') not in allowed:
        raise ValueError('Unknown service action')
    value = dict(payload)
    action = value['action']
    if set(value) - {'action'} - allowed[action]:
        raise ValueError('Unexpected action parameters')
    if action == 'takeoff':
        value.setdefault('height', 3.0)
        height = value['height']
        if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height) or not 1 <= height <= 10:
            raise ValueError('SITL takeoff height must be 1..10 metres')
    if action == 'profile' and (not isinstance(value.get('id'), str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', value['id'])):
        raise ValueError('Invalid profile identifier')
    if action == 'motor-fault':
        if type(value.get('motor')) is not int or not 1 <= value['motor'] <= 4:
            raise ValueError('Motor number must be 1..4')
        for key, default, maximum in [('current_scale', 1.6, 3.0), ('rpm_scale', .85, 1.0)]:
            value.setdefault(key, default)
            v = value[key]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= maximum:
                raise ValueError(key + ' outside supported range')
    return value
