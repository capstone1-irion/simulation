"""역할: Gazebo의 원래 출력 명령·시뮬레이션 시계를 배터리 계산과 MAVLink에 연결한다.
입력: motor_fault 상태 JSON, 선택한 프로파일. 출력: 소유 PX4 loopback BATTERY_STATUS,
분리된 groundtruth 기록. 주요 호출자: simulation.py. 비행 제어와 수집은 하지 않는다.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import socket
import threading
import time

from pymavlink.dialects.v20 import common

from .battery import Battery
from .processes import Child


class BatteryLink:
    """명령 기반 모델값을 보내는 시뮬레이션 전용 연결이다. 실기체에 연결하지 않는다."""

    def __init__(self, native_dir, env, run_dir, model_name, profile, initial_soc, input_port, system_id):
        self.native_dir = Path(native_dir)
        self.env = env
        self.folder = Path(run_dir)
        self.topic = '/capstone/' + model_name + '/status'
        self.model = Battery(profile, initial_soc=initial_soc)
        self.series_cells = profile['series_cells']
        self.target = ('127.0.0.1', input_port)
        self.system_id = system_id
        self.child = self.socket = self.file = self.thread = None
        self.lock = threading.Lock()
        self.enabled = threading.Event()
        self.stopping = threading.Event()
        self.error = None
        self.last = {}
        self.battery = {}
        self.last_wall = 0.0
        self.sent = 0

    def write(self, data):
        """pymavlink writer: 이 실행의 loopback 배터리 입력 포트로만 전송한다."""
        count = self.socket.sendto(data, self.target)
        if count != len(data):
            raise OSError('배터리 MAVLink 부분 전송')
        return count

    def start(self):
        """feed를 Gazebo보다 먼저 구독시킨다. PX4 입력 준비 전에는 송신하지 않는다."""
        try:
            self.file = (self.folder / 'groundtruth' / 'motor_battery.jsonl').open('x', encoding='utf-8')
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.encoder = common.MAVLink(self, srcSystem=self.system_id, srcComponent=180)
            self.child = Child([str(self.native_dir / 'gz_feed'), '--topic', self.topic],
                               self.folder / 'feed.log', env=self.env, cwd=self.folder, pipe_output=True)
            self.thread = threading.Thread(target=self._read, name='battery-link', daemon=True)
            self.thread.start()
        except BaseException:
            self.stop()
            raise

    def _send(self, sample):
        """SI 모델값을 MAVLink wire 단위로 변환한다. 모르는 온도/에너지는 sentinel이다."""
        cells = self.series_cells
        cell_mv = round(sample['voltage_v'] * 1000 / cells)
        current_ca = round(sample['current_a'] * 100)
        consumed = round(sample['consumed_mah'])
        if not (1 <= cells <= 10 and 1 <= cell_mv < 65535 and 0 <= current_ca < 32767
                and 0 <= consumed < 2147483647):
            raise ValueError('배터리 모델값이 BATTERY_STATUS 표현 범위를 벗어났습니다')
        msg = self.encoder.battery_status_encode(
            id=0, battery_function=common.MAV_BATTERY_FUNCTION_ALL,
            type=common.MAV_BATTERY_TYPE_UNKNOWN, temperature=32767,
            voltages=[cell_mv] * cells + [65535] * (10 - cells),
            current_battery=current_ca, current_consumed=consumed, energy_consumed=-1,
            battery_remaining=round(sample['soc'] * 100), time_remaining=0,
            charge_state=common.MAV_BATTERY_CHARGE_STATE_UNDEFINED,
            voltages_ext=[0, 0, 0, 0], mode=0, fault_bitmask=0)
        self.encoder.send(msg)
        self.sent += 1

    def _read(self):
        """원래 명령으로만 부하를 계산한다. 고장 배율·joint 관측은 정답 파일에만 남긴다."""
        had_input = False
        try:
            for raw in self.child.process.stdout:
                if self.stopping.is_set():
                    break
                row = json.loads(raw)
                if row.get('schema') != 'capstone.motor.v1':
                    raise ValueError('motor status schema 불일치')
                now = time.monotonic()
                error = row.get('error')
                if error and (error != 'no_input' or had_input):
                    raise RuntimeError('모터 중계 오류: ' + str(error))
                sample = None
                commands = row.get('command_rad_s')
                if commands is not None and error is None:
                    sim_time = row['sim_time_s']
                    if not isinstance(sim_time, (float, int)) or not math.isfinite(sim_time):
                        raise ValueError('유효한 시뮬레이션 시각이 필요합니다')
                    sample = self.model.update(commands, sim_time)
                    had_input = True
                    if self.enabled.is_set():
                        self._send(sample)
                record = {'kind': 'simulation_truth', 'receive_monotonic_ns': time.monotonic_ns(),
                          'motor': row, 'battery': sample, 'battery_packets_sent': self.sent}
                self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
                self.file.flush()
                with self.lock:
                    self.last = row
                    if sample is not None:
                        self.battery = sample
                    self.last_wall = now
            if not self.stopping.is_set():
                raise RuntimeError('Gazebo feed가 예상하지 않게 종료됐습니다')
        except BaseException as exc:
            if not self.stopping.is_set():
                self.error = repr(exc)

    def snapshot(self):
        """모델/정답 상태 사본이다. AI 입력 또는 PX4 수신 성공 증거로 사용하지 않는다."""
        with self.lock:
            return {'motor': dict(self.last), 'battery': dict(self.battery),
                    'age_wall_s': None if not self.last_wall else time.monotonic() - self.last_wall,
                    'packets_sent': self.sent, 'error': self.error}

    def stop(self):
        """feed 종료로 blocking read를 해제한 뒤 이 객체의 소켓과 정답 파일을 닫는다."""
        self.stopping.set()
        result = {'errors': []}
        if self.child is not None:
            result['feed'] = self.child.stop()
        if self.thread is not None:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                result['errors'].append('battery reader 종료 확인 실패')
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if self.file is not None:
            self.file.close()
            self.file = None
        if self.error:
            result['errors'].append(self.error)
        return result
