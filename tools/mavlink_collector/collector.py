#!/usr/bin/env python3
"""수신 전용 MAVLink 수집기: 독점 UDP·원문 보존·회전·서비스용 최신값. 범위는 README 참조."""
import json
import math
import argparse
import ipaddress
from pathlib import Path
import re
import copy
import hashlib
import importlib.metadata
import socket
import threading
import time
from collections import Counter, deque

SIMULATION = Path(__file__).resolve().parents[2]
DEFAULT_DECODER = SIMULATION.parent / 'hermes/mavlink/px4_messages.py'
SELECTED = frozenset(('HEARTBEAT EXTENDED_SYS_STATE ATTITUDE LOCAL_POSITION_NED '
                      'SERVO_OUTPUT_RAW BATTERY_STATUS ESC_INFO ESC_STATUS '
                      'POSITION_TARGET_LOCAL_NED ATTITUDE_TARGET ACTUATOR_OUTPUT_STATUS').split())


def parse_args(argv=None):
    """CLI 초·바이트·ID·경로를 사전 검사한다; 잘못된 입력에는 소켓을 열지 않는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--endpoint', default='udpin:127.0.0.1:14540')
    parser.add_argument('--output-root', type=Path, default=SIMULATION / 'experiments')
    parser.add_argument('--label', default='capture')
    parser.add_argument('--decoder', type=Path, default=DEFAULT_DECODER)
    parser.add_argument('--ulog', type=Path)
    parser.add_argument('--source-system', type=int)
    parser.add_argument('--source-component', type=int, default=1)
    parser.add_argument('--quality-profile', choices=QUALITY_PROFILES, default='standard')
    parser.add_argument('--rotate-bytes', type=int, default=16777216)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be nonnegative and finite; 0 means until SIGINT/SIGTERM')
    try:
        protocol, host, port = args.endpoint.split(':')
        ipaddress.IPv4Address(host)
        if protocol != 'udpin' or not 0 <= int(port) <= 65535:
            raise ValueError()
    except ValueError:
        parser.error('--endpoint must be udpin:<IPv4 address>:<port 0..65535>')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', args.label):
        parser.error('--label must contain 1..64 ASCII letters, digits, _ or -')
    for name in ('decoder', 'ulog'):
        path = getattr(args, name)
        if path is not None:
            if not path.is_file():
                parser.error(f'--{name} must be an existing file: {path}')
            setattr(args, name, path.resolve())
    if args.rotate_bytes < 1024:
        parser.error('--rotate-bytes must be >= 1024 bytes')
    for name in ('source_system', 'source_component'):
        value = getattr(args, name)
        if value is not None and not 1 <= value <= 255:
            parser.error(f'--{name.replace("_", "-")} must be 1..255')
    if args.output_root.exists() and not args.output_root.is_dir():
        parser.error('--output-root must be a directory path')
    return args


def load_decoder(path):
    """신뢰한 생성 Python 디코더를 독립 import한다; mavutil 전역이나 sys.path를 바꾸지 않는다."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('collector_px4_messages', path)
    if spec is None or spec.loader is None:
        raise ValueError(f'cannot load decoder: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


QUALITY_PROFILES = ('standard', 'arion_cmu_v1', 'legacy_inspected_px4_setup')


def quality_annotations(name, fields, profile='standard'):
    """원문 단위를 바꾸지 않고 프로필별 의미/sentinel만 주석 처리한다; 물리 유효성 인증이 아니다."""
    if profile not in QUALITY_PROFILES:
        raise ValueError(f'unknown quality profile: {profile}')
    modeled = profile == 'arion_cmu_v1'
    legacy = profile == 'legacy_inspected_px4_setup'
    labels = {}
    if name == 'BATTERY_STATUS':
        current = fields.get('current_battery')
        if modeled:
            labels.update(current_battery='modeled_pack_load', current_consumed='modeled_coulomb_count',
                          battery_remaining='modeled_soc_not_soh', temperature='not_thermal_measurement')
            if fields.get('temperature') == 2500:
                labels['temperature'] = 'constant_25c_not_thermal_model'
        if current == -1:
            labels['current_battery'] = 'unavailable_sentinel_minus1'
        elif legacy and current == -100:
            labels['current_battery'] = 'invalid_setup_placeholder'
        for key in ('current_consumed', 'energy_consumed'):
            if fields.get(key, 0) < 0:
                labels[key] = 'unavailable_negative'
        if fields.get('battery_remaining') == -1:
            labels['battery_remaining'] = 'unavailable_sentinel_minus1'
        if fields.get('temperature') == 32767:
            labels['temperature'] = 'unavailable_sentinel_32767'
        if 'voltages' in fields:
            labels['voltages'] = ['unavailable_sentinel_65535' if v == 65535 else
                                  'modeled_equal_cell_voltage' if modeled else 'not_validated'
                                  for v in fields['voltages']]
    elif name == 'ESC_STATUS':
        if modeled:
            labels = {'rpm': 'simulated_joint_rpm_telemetry_scale_not_physical_fault',
                      'voltage': 'modeled_pack_terminal_voltage_one_way_coupling',
                      'current': 'bench_lookup_model_with_telemetry_scale'}
        elif legacy:
            labels = {'rpm': 'command_derived_units_unverified',
                      'voltage': 'unpopulated_in_inspected_bridge',
                      'current': 'unpopulated_in_inspected_bridge'}
    elif name == 'ESC_INFO':
        labels['temperature'] = ['unavailable_sentinel_32767' if v == 32767 else
                                 'unpopulated_in_inspected_bridge' if legacy or modeled else 'not_validated'
                                 for v in fields.get('temperature', [])]
        if modeled:
            labels['failure_flags'] = 'unimplemented_not_fault_ground_truth'
            labels['error_count'] = 'unimplemented_not_fault_ground_truth'
    elif name == 'POSITION_TARGET_LOCAL_NED':
        labels = {key: 'commanded_not_measured_check_frame_and_type_mask'
                  for key in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'afx', 'afy', 'afz', 'yaw', 'yaw_rate')}
        labels.update(coordinate_frame='required_for_setpoint_interpretation',
                      type_mask='ignored_fields_not_fault_flags')
    elif name == 'ATTITUDE_TARGET':
        labels = {key: 'commanded_body_rate_rad_s_freshness_unverified'
                  for key in ('body_roll_rate', 'body_pitch_rate', 'body_yaw_rate')}
        labels.update(q='commanded_attitude_wxyz_check_type_mask_and_validity',
                      thrust='normalized_thrust_command_not_measured_force',
                      type_mask='ignored_fields_not_fault_flags',
                      time_boot_ms='source_timestamp_not_per_field_freshness_guarantee')
    elif name == 'ACTUATOR_OUTPUT_STATUS':
        labels = {'actuator': 'raw_output_commands_driver_units_and_motor_mapping_unverified',
                  'active': 'producer_specific_active_outputs_not_fault_flags'}
    return {'profile': profile, 'fields': labels}


def collect(args):
    """검증한 CLI 설정으로 수집한다; duration0은 신호까지 유지하고 종료 요약·코드를 반환한다."""
    import signal
    import sys
    import uuid
    from datetime import datetime, timezone

    directory = args.output_root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
                                    + '-' + args.label + '-' + uuid.uuid4().hex)
    recorder = None
    stop = threading.Event()
    received_signal = None
    old_handlers = {}
    code, reason = 0, 'duration_elapsed'

    def request_stop(signum, frame):
        """SIGINT/SIGTERM 번호를 보관하고 wall-clock 수집 대기를 깨운다; 파일 정리는 본문에서 한다."""
        nonlocal received_signal
        received_signal = signum
        stop.set()

    try:
        _, host, port = args.endpoint.split(':')
        recorder = MavlinkRecorder(args.decoder, directory, host, int(port), args.source_system,
                                   args.source_component, args.quality_profile, args.rotate_bytes)
        recorder._run_metadata = {
            'duration_seconds': args.duration, 'label': args.label,
            'ulog': None if args.ulog is None else {'path': str(args.ulog),
                    'association': 'user_associated_time_alignment_unverified'}}
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        recorder.start()
        print(f'READY {directory.resolve()}', flush=True)
        started = time.monotonic()
        while not stop.is_set() and (args.duration == 0 or time.monotonic() - started < args.duration):
            if recorder.snapshot()['errors']:
                code, reason = 1, 'error'
                break
            stop.wait(0.05)
        if received_signal is not None:
            reason = 'interrupted' if received_signal == signal.SIGINT else 'terminated'
            code = 130 if received_signal == signal.SIGINT else 0
        state = recorder.snapshot()
        if reason == 'duration_elapsed' and state['seen_total'] == state['unknown_count'] + state['bad_count']:
            code, reason = 3, 'no_data'
    except KeyboardInterrupt:
        code, reason = 130, 'interrupted'
    except Exception as exc:
        print(f'collector: {type(exc).__name__}: {exc}', file=sys.stderr)
        code, reason = 1, 'error'
    finally:
        if recorder is not None:
            with recorder._lock:
                if not recorder._errors:
                    recorder._exit_reason = reason
            recorder.close()
            if recorder.snapshot()['errors']:
                code = 1
        for signum, previous in old_handlers.items():
            signal.signal(signum, previous)
    return code

class MavlinkRecorder:
    """독점 UDP 수신기로 원문을 보존한다; 초 단위 freshness와 별도 최신값을 제공한다."""

    def __init__(self, decoder_path, output_dir, bind_host='127.0.0.1', bind_port=0,
                 source_system=None, source_component=1, quality_profile='arion_cmu_v1',
                 rotate_bytes=16777216):
        """설정만 보관한다; 디코더 경로·IPv4·포트와 새 출력 경로를 검사하며 소켓은 열지 않는다."""
        self.decoder_path = Path(decoder_path).resolve()
        if not self.decoder_path.is_file():
            raise ValueError('decoder_path must be an existing trusted Python file')
        self.output_dir = Path(output_dir).absolute()
        if self.output_dir.exists():
            raise FileExistsError(f'output_dir must be new: {self.output_dir}')
        if not isinstance(bind_host, str):
            raise ValueError('bind_host must be a numeric IPv4 string')
        ipaddress.IPv4Address(bind_host)
        if type(bind_port) is not int or not 0 <= bind_port <= 65535:
            raise ValueError('bind_port must be an integer in 0..65535')
        for value in (source_system, source_component):
            if value is not None and (type(value) is not int or not 1 <= value <= 255):
                raise ValueError('source IDs must be None or integers in 1..255')
        if type(rotate_bytes) is not int or rotate_bytes < 1024:
            raise ValueError('rotate_bytes must be an integer >= 1024 bytes')
        if quality_profile not in QUALITY_PROFILES:
            raise ValueError(f'unknown quality profile: {quality_profile}')
        self.bind_host, self.port = str(bind_host), bind_port
        self.source_system, self.source_component = source_system, source_component
        self.quality_profile, self.rotate_bytes = quality_profile, rotate_bytes
        self._lock = threading.RLock()
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._thread = self._socket = self._raw = None
        self._started = self._closed = self._owns_directory = False
        self._started_ns = time.monotonic_ns()
        self._seen, self._saved = Counter(), Counter()
        self._latest = {}
        self._errors = deque(maxlen=16)
        self._error_total = 0
        self._last_receive_ns = self._last_matching_ns = None
        self._datagrams = self._bytes = self._filtered = 0
        self._run_metadata = {}
        self._raw_index = self._raw_size = 0
        self._exit_reason = 'not_started'

    def start(self):
        """새 결과 폴더와 비재사용 UDP 소켓을 열고 worker를 시작한다; 자기 자신을 반환한다."""
        with self._lifecycle:
            if self._closed:
                raise RuntimeError('a closed recorder cannot restart; use a new instance')
            if self._started:
                return self
            self.output_dir.mkdir(parents=True, exist_ok=False)
            self._owns_directory = True
            self._started_ns = time.monotonic_ns()
            try:
                self._decoder = load_decoder(self.decoder_path)
                self._raw = (self.output_dir / 'mavlink_raw.jsonl').open('xb')
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.bind((self.bind_host, self.port))
                self._socket.settimeout(0.05)
                self.port = self._socket.getsockname()[1]
                metadata = {
                    'schema_version': 2, 'started_utc_ns': time.time_ns(),
                    'receive_only': True, 'endpoint': f'udpin:{self.bind_host}:{self.port}',
                    'decoder_path': str(self.decoder_path),
                    'decoder_sha256': hashlib.sha256(self.decoder_path.read_bytes()).hexdigest(),
                    'collector_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    'pymavlink_version': importlib.metadata.version('pymavlink'),
                    'quality_profile': self.quality_profile,
                    'source_system': self.source_system, 'source_component': self.source_component,
                    'source_ids_are_authentication': False,
                    'rotate_bytes': self.rotate_bytes,
                    'quality_scope': {
                        'physical_validation': False,
                        'joint_feedback_required': self.quality_profile == 'arion_cmu_v1',
                        'temperature_model': 'constant_25C_not_thermal' if self.quality_profile == 'arion_cmu_v1' else 'unverified',
                        'coupling': 'one_way_telemetry_electrical_model_not_physical_fault' if self.quality_profile == 'arion_cmu_v1' else 'unverified',
                        'ground_truth_controls_are_live_features': False},
                    'warnings': ['Source IDs are not authentication. Quality is a declared producer contract, not sensor validation.'],
                    **self._run_metadata,
                }
                with (self.output_dir / 'metadata.json').open('x', encoding='utf-8') as stream:
                    stream.write(json_text(metadata) + '\n')
                self._exit_reason = 'running'
                self._thread = threading.Thread(target=self._run, name='MavlinkRecorder', daemon=True)
                self._started = True
                self._thread.start()
                return self
            except BaseException as exc:
                self._remember_error('start', exc)
                self._release()
                self._closed = True
                self._write_summary()
                raise

    def _remember_error(self, stage, exc):
        """오류를 최대 16개·각 1024자로 보관하고 전체 오류 개수를 증가시킨다."""
        with self._lock:
            self._errors.append(f'{stage}: {type(exc).__name__}: {exc}'[:1024])
            self._error_total += 1
            self._exit_reason = 'error'

    def _release(self):
        """소유 파일·소켓만 닫는다; 닫기 오류도 health에 남긴다."""
        for name in ('_raw', '_socket'):
            resource = getattr(self, name)
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    self._remember_error('close' + name, exc)
                setattr(self, name, None)

    def _run(self):
        """wall-clock 50ms timeout으로 수신한다; MAVLink를 보내지 않고 오류는 부모에게 공개한다."""
        try:
            while not self._stop.is_set():
                try:
                    data, peer = self._socket.recvfrom(65535)
                except socket.timeout:
                    continue
                now = time.monotonic_ns()
                utc = time.time_ns()
                with self._lock:
                    self._last_receive_ns = now
                    self._datagrams += 1
                    self._bytes += len(data)
                parser = self._decoder.MAVLink(None)
                parser.robust_parsing = True
                message = parser.parse_char(data)
                while message is not None:
                    self._record(message, peer, now, utc)
                    message = parser.parse_char(b'')
                if parser.buf_len():
                    # UDP 경계를 넘는 조립은 하지 않는다. peer가 다른 조각을 섞지 않고 원문을 보존한다.
                    partial = bytearray(parser.buf[parser.buf_index:])
                    self._record(self._decoder.MAVLink_bad_data(partial, 'truncated UDP datagram'), peer, now, utc)
        except Exception as exc:
            self._remember_error('worker', exc)
            self._stop.set()
        finally:
            self._release()

    def _matches(self, system, component):
        """최신값은 autopilot component1과 선택 ID만 허용한다; ID는 인증이 아니다."""
        return (component == 1 and (self.source_system is None or system == self.source_system)
                and (self.source_component is None or component == self.source_component))

    def _record(self, message, peer, now, utc):
        """파서 결과를 집계하고 선택 원문을 즉시 flush한다; 최신값은 타입당 하나만 보관한다."""
        name = message.get_type()
        kind = 'UNKNOWN' if name.startswith('UNKNOWN') else name
        system, component = message.get_srcSystem(), message.get_srcComponent()
        with self._lock:
            self._seen[kind] += 1
        if name not in SELECTED and name != 'BAD_DATA' and kind != 'UNKNOWN':
            return
        fields = message.to_dict()
        record = {'receive_utc_ns': utc, 'receive_monotonic_ns': now,
                  'elapsed_monotonic_ns': now - self._started_ns, 'peer': list(peer),
                  'source_system': system, 'source_component': component,
                  'message_id': message.get_msgId(), 'message_name': name,
                  'packet_sequence': message.get_seq(), 'fields': fields,
                  'quality': quality_annotations(name, fields, self.quality_profile
                      if self._matches(system, component) else 'standard')}
        self._write_record(record)
        with self._lock:
            self._saved[kind] += 1
            if name in SELECTED and self._matches(system, component):
                self._latest[name] = (now, fields)
                self._last_matching_ns = now
            else:
                self._filtered += 1

    def _write_record(self, record):
        """UTF-8 바이트 상한 전 새 JSONL로 회전한다; 단일 줄 초과/디스크 오류는 조용히 버리지 않는다."""
        data = (json_text(record) + '\n').encode('utf-8')
        if len(data) > self.rotate_bytes:
            raise ValueError(f'record size {len(data)} exceeds rotate_bytes={self.rotate_bytes}')
        if self._raw_size + len(data) > self.rotate_bytes:
            self._raw.close()
            next_index = self._raw_index + 1
            path = self.output_dir / f'mavlink_raw.{next_index:06d}.jsonl'
            self._raw = path.open('xb')
            with self._lock:
                self._raw_index, self._raw_size = next_index, 0
        self._raw.write(data)
        self._raw.flush()
        with self._lock:
            self._raw_size += len(data)

    def latest(self, message_name, max_age_s=3.0):
        """선택 autopilot 원문 사본을 반환한다; 수신 나이가 초 한도를 넘으면 빈 dict를 반환한다."""
        if not math.isfinite(max_age_s) or max_age_s < 0:
            raise ValueError('max_age_s must be finite and nonnegative seconds')
        with self._lock:
            sample = self._latest.get(message_name)
            if sample is None or (time.monotonic_ns() - sample[0]) / 1e9 > max_age_s:
                return {}
            return copy.deepcopy(sample[1])

    def snapshot(self):
        """JSON-safe health·개수·초 단위 수신 나이·경로를 복사한다; raw 이력은 반환하지 않는다."""
        with self._lock:
            now = time.monotonic_ns()
            receive_age = None if self._last_receive_ns is None else (now - self._last_receive_ns) / 1e9
            matching_age = None if self._last_matching_ns is None else (now - self._last_matching_ns) / 1e9
            alive = self._thread is not None and self._thread.is_alive()
            return {'seen_total': sum(self._seen.values()), 'saved_total': sum(self._saved.values()),
                    'seen_by_type': dict(self._seen), 'saved_by_type': dict(self._saved),
                    'unknown_count': self._seen['UNKNOWN'], 'bad_count': self._seen['BAD_DATA'],
                    'datagrams_received': self._datagrams, 'bytes_received': self._bytes,
                    'latest_filtered_total': self._filtered, 'latest_slots': len(self._latest),
                    'receive_age_s': receive_age, 'matching_receive_age_s': matching_age,
                    'latest_age_s': {name: (now - value[0]) / 1e9 for name, value in self._latest.items()},
                    'worker_alive': alive, 'running': alive and not self._stop.is_set(),
                    'healthy': alive and not self._errors and matching_age is not None and matching_age <= 3,
                    'errors': list(self._errors), 'error_total': self._error_total,
                    'error': self._errors[-1] if self._errors else None,
                    'warnings': ['silence: no valid decoded messages received (UNKNOWN/BAD_DATA excluded)']
                          if sum(self._seen.values()) == self._seen['UNKNOWN'] + self._seen['BAD_DATA'] else [],
                    'exit_reason': self._exit_reason, 'elapsed_monotonic_ns': now - self._started_ns,
                    'port': self.port, 'output_dir': str(self.output_dir),
                    'raw_file_count': self._raw_index + 1 if self._started else 0,
                    'raw_current_bytes': self._raw_size, 'rotate_bytes': self.rotate_bytes,
                    'paths': {'metadata': str(self.output_dir / 'metadata.json'),
                              'raw': str(self.output_dir / 'mavlink_raw.jsonl'),
                              'raw_current': str(self.output_dir / ('mavlink_raw.jsonl' if self._raw_index == 0
                                      else f'mavlink_raw.{self._raw_index:06d}.jsonl')),
                              'raw_rotation_pattern': 'mavlink_raw.[0-9]*.jsonl',
                              'summary': str(self.output_dir / 'summary.json')}}

    def _write_summary(self):
        """소유한 새 폴더에 최종 summary를 한 번 기록한다; 디스크 오류는 health에 남긴다."""
        if self._owns_directory:
            try:
                with (self.output_dir / 'summary.json').open('x', encoding='utf-8') as stream:
                    stream.write(json_text(self.snapshot()) + '\n')
            except Exception as exc:
                self._remember_error('summary', exc)

    def close(self):
        """정지 요청 후 최대 2 wall초 join한다; 반복 호출은 안전하며 timeout은 오류로 공개한다."""
        with self._lifecycle:
            if self._closed:
                return
            self._stop.set()
            if self._thread is not None and self._thread.ident is not None:
                self._thread.join(timeout=2)
                if self._thread.is_alive():
                    self._remember_error('shutdown', TimeoutError('worker did not stop within 2 seconds'))
                    return
            if not self._errors and self._exit_reason in ('running', 'not_started'):
                self._exit_reason = 'closed'
            self._closed = True
            self._write_summary()


def safe_json(value):
    """비유한 실수와 bytes를 JSON 태그로 표시한다; 원래 입력값·sentinel은 변경하지 않는다."""
    if isinstance(value, float) and not math.isfinite(value):
        return {'$float': str(value)}
    if isinstance(value, (bytes, bytearray)):
        return {'$bytes_hex': bytes(value).hex()}
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    return value


def json_text(value):
    """원문/요약을 엄격한 UTF-8 JSON 문자열로 직렬화한다; 단위 변환은 하지 않는다."""
    return json.dumps(safe_json(value), ensure_ascii=False, allow_nan=False)


if __name__ == '__main__':
    raise SystemExit(collect(parse_args()))
