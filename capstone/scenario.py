"""역할: 명시적으로 요청한 SITL 비행과 실험 순서를 실행한다.
입력: simulation.Runtime, 실험 종류. 출력: 정답 이벤트·실제 비행 상태 확인 결과.
주요 호출자: main.py, 최종 통합 검증. 수집·배터리 계산은 이 파일에서 하지 않는다.
"""
from __future__ import annotations

import math
import socket
import threading

from pymavlink.dialects.v20 import common


class GroundStation:
    """소유 PX4에 관제 heartbeat만 보낸다. 수집기가 송신하지 않도록 역할을 분리한다."""

    def __init__(self, port):
        self.target = ('127.0.0.1', port)
        self.socket = None
        self.thread = None
        self.stopping = threading.Event()
        self.error = None
        self.sent = 0

    def write(self, data):
        """MAVLink writer의 loopback 전송. 외부 주소를 받을 수 없다."""
        count = self.socket.sendto(data, self.target)
        if count != len(data):
            raise OSError('GCS heartbeat 부분 전송')
        return count

    def start(self):
        """실제 GCS 링크 생존 신호를 제공한다. 데이터링크 failsafe를 끄지 않는다."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.encoder = common.MAVLink(self, srcSystem=255, srcComponent=190)
        self.thread = threading.Thread(target=self._run, name='gcs-heartbeat', daemon=True)
        try:
            self.thread.start()
        except BaseException:
            self.socket.close()
            raise

    def _run(self):
        """벽시계 1초 주기로 송신한다. 오류는 supervisor에 그대로 노출한다."""
        try:
            while not self.stopping.is_set():
                self.encoder.send(self.encoder.heartbeat_encode(
                    type=common.MAV_TYPE_GCS, autopilot=common.MAV_AUTOPILOT_INVALID,
                    base_mode=0, custom_mode=0, system_status=common.MAV_STATE_ACTIVE,
                    mavlink_version=3))
                self.sent += 1
                self.stopping.wait(1)
        except BaseException as exc:
            self.error = repr(exc)

    def stop(self):
        """이 객체의 스레드와 소켓만 닫는다. 실제 관제 앱은 종료하지 않는다."""
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                self.error = 'GCS heartbeat 스레드 종료 확인 실패'
        if self.socket:
            self.socket.close()
        return {'sent': self.sent, 'error': self.error}


def airborne(runtime):
    """관측된 무장·높이를 함께 확인한다. 명령 접수만으로 이륙 성공을 선언하지 않는다."""
    hb = runtime.collector.latest('HEARTBEAT')
    pos = runtime.collector.latest('LOCAL_POSITION_NED')
    z = pos.get('z')
    return bool(hb.get('base_mode', 0) & 128) and isinstance(z, (int, float)) and math.isfinite(z) and z < -2


def takeoff(runtime):
    """소유한 시뮬레이션을 정상 arm/takeoff 절차로 띄운다. force-arm은 사용하지 않는다."""
    if runtime.state != 'running' or runtime.paused:
        raise RuntimeError('실행 중이고 pause가 해제된 SITL이 필요합니다')
    hb = runtime.collector.latest('HEARTBEAT')
    if not hb:
        raise RuntimeError('새 heartbeat가 없습니다')
    if hb['base_mode'] & 128:
        raise RuntimeError('이미 무장되어 있습니다')
    runtime.wait(lambda: 'Preflight check: OK' in runtime.command('commander check'),
                 description='센서·위치 정상 preflight', timeout=40)
    runtime.event('takeoff_requested')
    # 원본 commander takeoff는 모드 접수 후 정상 arm도 수행한다.
    runtime.command('commander takeoff')
    runtime.wait(lambda: airborne(runtime), description='실제 이륙 높이', timeout=40)
    runtime.event('airborne_observed', position=runtime.collector.latest('LOCAL_POSITION_NED'))


def land(runtime, *, check_health=True):
    """정상 착륙 명령 후 지상 판정과 자동 Disarm을 기다린다. 강제 disarm하지 않는다."""
    hb = runtime.collector.latest('HEARTBEAT')
    if not hb:
        raise RuntimeError('착륙 전 새 heartbeat를 확인할 수 없습니다')
    if not hb['base_mode'] & 128:
        return
    runtime.event('land_requested')
    runtime.command('commander land')

    def landed():
        heartbeat = runtime.collector.latest('HEARTBEAT')
        state = runtime.collector.latest('EXTENDED_SYS_STATE')
        return bool(heartbeat) and not (heartbeat['base_mode'] & 128) and state.get('landed_state') == 1

    runtime.wait(landed, description='착륙·자동 Disarm', timeout=60, check_health=check_health)
    runtime.event('landed_disarmed_observed')


def hold(runtime, seconds):
    """hover 구간 길이는 시뮬레이션 초다. 무수신/pause로 무한 대기하지 않는다."""
    if not math.isfinite(seconds) or not 0 < seconds <= 60:
        raise ValueError('hold 길이는 0~60 시뮬레이션 초 범위여야 합니다')
    start = runtime.link.snapshot()['motor']['sim_time_s']
    runtime.wait(lambda: runtime.link.snapshot()['motor'].get('sim_time_s', start) - start >= seconds,
                 timeout=115, description='시뮬레이션 시간 경과')


def run_scenario(runtime, name):
    """동일 비행 순서에 모터 약화 유무만 바꾼다. 배터리는 run 시작 전에 이미 선택돼 있다."""
    if name not in ('normal', 'motor', 'battery'):
        raise ValueError('scenario는 normal/motor/battery 중 하나입니다')
    if name == 'motor' and runtime.profile['id'] != 'cmu_vah01_rpt000':
        raise ValueError('모터 시나리오는 기준 배터리에서만 실행합니다')
    runtime.event('scenario_start', name=name)
    takeoff(runtime)
    hold(runtime, 5)
    if name == 'motor':
        runtime.fault(motor=1, scale=.9, duration=3)
        hold(runtime, 4)
        runtime.wait(lambda: runtime.link.snapshot()['motor'].get('fault_active') is False,
                     timeout=5, description='고장 자동 해제')
    else:
        hold(runtime, 4)
    hold(runtime, 3)
    land(runtime)
    runtime.event('scenario_end', name=name)
