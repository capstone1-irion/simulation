"""상시 수집기 집중 시험: 합성 UDP만 사용하며 PX4/실기체에 연결하지 않는다."""
import importlib.util
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest

HERE = Path(__file__).resolve().parent
ARTIFACTS = HERE.parents[2] / 'hermes/cmu-service/io'


def collector_module():
    """제품 모듈을 경로로 읽는다; 전역 검색 경로와 설치 파일은 변경하지 않는다."""
    spec = importlib.util.spec_from_file_location('continuous_collector', HERE / 'collector.py')
    if spec is None or spec.loader is None:
        raise RuntimeError('collector module loader unavailable')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ContinuousRecorderTests(unittest.TestCase):
    """임시 loopback 포트와 프로젝트 안의 고유 합성 결과만 사용한다."""
    def setUp(self):
        """새 시험 폴더와 디코더를 준비한다; 기존 결과는 덮어쓰지 않는다."""
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        self.root = Path(tempfile.mkdtemp(prefix='synthetic-recorder-', dir=ARTIFACTS))
        self.c = collector_module()
        self.m = self.c.load_decoder(self.c.DEFAULT_DECODER)

    def recorder(self, **kwargs):
        """기본 API로 수집기를 생성하고 정상 종료를 시험 정리에 등록한다."""
        self.assertTrue(hasattr(self.c, 'MavlinkRecorder'), 'reusable receive-only API is missing')
        recorder = self.c.MavlinkRecorder(self.c.DEFAULT_DECODER, self.root / 'raw', **kwargs)
        self.addCleanup(recorder.close)
        return recorder

    def wait_count(self, recorder, count):
        """최대 2 wall초 내 실제 저장 개수를 기다리며 worker 오류도 검사한다."""
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            state = recorder.snapshot()
            self.assertEqual(state['errors'], [])
            if state['saved_total'] >= count:
                return state
            time.sleep(0.01)
        self.fail(f'synthetic packets not recorded: {recorder.snapshot()}')

    def test_quality_profiles_are_explicit_and_do_not_rewrite_fields(self):
        """기본 표준/명시 legacy/CMU joint·모델 주석을 구분하고 원문 sentinel을 보존한다."""
        fields = {'current_battery': -100, 'temperature': 2500, 'current_consumed': 5,
                  'energy_consumed': -1, 'voltages': [4100, 65535], 'battery_remaining': 90}
        raw_before = self.c.json_text(fields)
        standard = self.c.quality_annotations('BATTERY_STATUS', fields)
        self.assertEqual(standard['profile'], 'standard')
        self.assertNotEqual(standard['fields'].get('current_battery'), 'invalid_setup_placeholder')
        legacy = self.c.quality_annotations('BATTERY_STATUS', fields, 'legacy_inspected_px4_setup')
        self.assertEqual(legacy['fields']['current_battery'], 'invalid_setup_placeholder')
        modeled = self.c.quality_annotations('BATTERY_STATUS', fields, 'arion_cmu_v1')
        self.assertEqual(modeled['fields']['temperature'], 'constant_25c_not_thermal_model')
        self.assertEqual(modeled['fields']['voltages'], ['modeled_equal_cell_voltage', 'unavailable_sentinel_65535'])
        esc = self.c.quality_annotations('ESC_STATUS', {}, 'arion_cmu_v1')
        self.assertEqual(esc['fields']['rpm'], 'simulated_joint_rpm_telemetry_scale_not_physical_fault')
        self.assertEqual(esc['fields']['current'], 'bench_lookup_model_with_telemetry_scale')
        self.assertNotIn('true_fault', modeled)
        self.assertEqual(self.c.json_text(fields), raw_before)
        with self.assertRaises(ValueError):
            self.c.quality_annotations('ESC_STATUS', {}, 'typo')

    def test_control_quality_does_not_turn_commands_into_measurements(self):
        """목표·구동 출력의 의미를 주석으로 구분하고 mask/원문/비유한값을 재작성하지 않는다."""
        cases = {
            'POSITION_TARGET_LOCAL_NED': ({'coordinate_frame': 1, 'type_mask': 1, 'x': float('nan'), 'vx': .5},
                {'x': 'commanded_not_measured_check_frame_and_type_mask',
                 'type_mask': 'ignored_fields_not_fault_flags'}),
            'ATTITUDE_TARGET': ({'time_boot_ms': 0, 'type_mask': 128, 'q': [0., 0., 0., 0.], 'thrust': .5},
                {'q': 'commanded_attitude_wxyz_check_type_mask_and_validity',
                 'thrust': 'normalized_thrust_command_not_measured_force'}),
            'ACTUATOR_OUTPUT_STATUS': ({'time_usec': 0, 'active': 4, 'actuator': [100., 0., float('nan')]},
                {'actuator': 'raw_output_commands_driver_units_and_motor_mapping_unverified',
                 'active': 'producer_specific_active_outputs_not_fault_flags'}),
        }
        for profile in self.c.QUALITY_PROFILES:
            for name, (fields, expected) in cases.items():
                with self.subTest(profile=profile, message=name):
                    before = self.c.json_text(fields)
                    quality = self.c.quality_annotations(name, fields, profile)
                    self.assertEqual(quality['profile'], profile)
                    for key, value in expected.items():
                        self.assertEqual(quality['fields'].get(key), value)
                    self.assertEqual(self.c.json_text(fields), before)

    def test_every_selected_packet_rotates_and_latest_filters(self):
        """합성 관측·제어 11종과 진단을 저장하되 필터·만료·reset 시각과 원문 mask를 보존한다."""
        from unittest import mock
        recorder = self.recorder(source_system=42, rotate_bytes=4096).start()
        m = self.m
        messages = [m.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3),
                    m.MAVLink_extended_sys_state_message(0, 1),
                    m.MAVLink_attitude_message(123, 0, 0, 0, 0, 0, 0),
                    m.MAVLink_local_position_ned_message(123, 1, 2, 3, 0, 0, 0),
                    m.MAVLink_servo_output_raw_message(123, 0, *([1000] * 8)),
                    m.MAVLink_battery_status_message(0, 0, 0, 2500, [4100]*4+[65535]*6, -1, 5, -1, 90),
                    m.MAVLink_esc_info_message(0, 123, 1, 4, 0, 0, [0]*4, [0]*4, [32767]*4),
                    m.MAVLink_esc_status_message(0, 123, [500]*4, [float('nan'), 0, 0, 0], [1]*4),
                    m.MAVLink_position_target_local_ned_message(123, 1, 1, float('nan'), 2, -3,
                                                              .5, 0, 0, 0, 0, 0, 0, 0),
                    m.MAVLink_attitude_target_message(123, 0, [1, 0, 0, 0], .5, 0, 0, .5),
                    m.MAVLink_actuator_output_status_message(123000, 4, [100, 200, 300, 400] + [0]*28)]
        encoder = m.MAVLink(None, srcSystem=42, srcComponent=1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for repeat in range(3):
                for message in messages:
                    sender.sendto(message.pack(encoder), ('127.0.0.1', recorder.port))
                self.wait_count(recorder, (repeat + 1) * len(messages))
            for system, component in ((42, 180), (99, 1)):
                encoder.srcSystem, encoder.srcComponent = system, component
                sender.sendto(m.MAVLink_heartbeat_message(2, 3, 128, 0, 4, 3).pack(encoder), ('127.0.0.1', recorder.port))
            encoder.srcSystem, encoder.srcComponent = 42, 1
            sender.sendto(m.MAVLink_ping_message(123, 0, 0, 0).pack(encoder), ('127.0.0.1', recorder.port))
            unknown = bytearray(messages[0].pack(encoder))
            unknown[7:10] = (60000).to_bytes(3, 'little')
            sender.sendto(unknown, ('127.0.0.1', recorder.port))
            sender.sendto(b'garbage', ('127.0.0.1', recorder.port))
            before_reset = 3 * len(messages) + 4
            self.wait_count(recorder, before_reset)
            self.assertEqual(recorder.latest('HEARTBEAT')['base_mode'], 0)
            then = time.monotonic_ns()
            with mock.patch.object(self.c.time, 'monotonic_ns', return_value=then + 6_000_000_000):
                self.assertEqual(recorder.latest('HEARTBEAT'), {})
                self.assertFalse(recorder.snapshot()['healthy'])
            sender.sendto(m.MAVLink_attitude_message(0, 0, 0, 0, 0, 0, 0).pack(encoder), ('127.0.0.1', recorder.port))
            self.wait_count(recorder, before_reset + 1)
            self.assertEqual(recorder.latest('ATTITUDE')['time_boot_ms'], 0)
            changed = recorder.latest('ATTITUDE')
            changed['time_boot_ms'] = 999
            self.assertEqual(recorder.latest('ATTITUDE')['time_boot_ms'], 0)
        recorder.close()
        files = sorted((self.root / 'raw').glob('mavlink_raw*.jsonl'))
        self.assertGreater(len(files), 1, 'raw JSONL did not rotate')
        self.assertTrue(all(path.stat().st_size <= 4096 for path in files))
        records = [json.loads(line) for path in files for line in path.read_text().splitlines()]
        state = recorder.snapshot()
        self.assertEqual(len(records), before_reset + 1)
        self.assertEqual(state['seen_total'], len(records) + 1)
        self.assertEqual(state['unknown_count'], 1)
        self.assertEqual(state['bad_count'], 1)
        self.assertEqual(state['latest_slots'], len(messages))
        self.assertEqual({message.get_type() for message in messages}, self.c.SELECTED)
        self.assertEqual(state['raw_file_count'], len(files))
        esc = next(record for record in records if record['message_name'] == 'ESC_STATUS')
        self.assertEqual(esc['fields']['voltage'][0], {'$float': 'nan'})
        self.assertEqual(esc['quality']['profile'], 'arion_cmu_v1')
        for message in messages[-3:]:
            name = message.get_type()
            rows = [row for row in records if row['message_name'] == name]
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]['fields'], self.c.safe_json(message.to_dict()))
            self.assertEqual(state['saved_by_type'][name], 3)
            self.assertEqual(self.c.safe_json(recorder.latest(name)), rows[0]['fields'])
        other = next(record for record in records if record['source_component'] == 180)
        self.assertEqual(other['quality']['profile'], 'standard')
        self.assertFalse(any(isinstance(value, list) and len(value) >= 29 for value in vars(recorder).values()))

    def test_continuous_cli_graceful_signals_and_defaults(self):
        """duration0 CLI의 SIGTERM/SIGINT 요약·무수신 결과와 기본 standard 프로필을 검사한다."""
        import select
        import signal
        import subprocess
        import sys
        args = self.c.parse_args(['--duration', '0', '--endpoint', 'udpin:127.0.0.1:0',
                                  '--source-system', '42', '--source-component', '1', '--rotate-bytes', '4096'])
        self.assertEqual(args.quality_profile, 'standard')
        self.assertEqual(args.duration, 0)
        for sig, expected_code in ((signal.SIGTERM, 0), (signal.SIGINT, 130)):
            with self.subTest(signal=sig):
                command = [sys.executable, str(HERE/'collector.py'), '--duration', '0',
                           '--endpoint', 'udpin:127.0.0.1:0', '--output-root', str(self.root),
                           '--label', 'synthetic-signal']
                with (self.root/f'signal-{sig}.stderr').open('w') as error_log:
                    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=error_log, text=True)
                    try:
                        ready, _, _ = select.select([process.stdout], [], [], 3)
                        self.assertTrue(ready)
                        line = process.stdout.readline()
                        self.assertTrue(line.startswith('READY '), line)
                        directory = Path(line.strip()[6:])
                        self.assertIsNone(process.poll())
                        process.send_signal(sig)
                        process.communicate(timeout=3)
                        self.assertEqual(process.returncode, expected_code)
                        summary = json.loads((directory/'summary.json').read_text())
                        self.assertEqual(summary['errors'], [])
                        self.assertFalse(summary['worker_alive'])
                        self.assertEqual(summary['exit_reason'], 'terminated' if sig == signal.SIGTERM else 'interrupted')
                    finally:
                        if process.poll() is None:
                            process.kill()
                        process.communicate(timeout=3)

    def test_truncated_datagram_diagnostic_then_recovery(self):
        """잘린 UDP 조각은 BAD_DATA로 보존하고 다른 peer/다음 프레임에 섞지 않는다."""
        recorder = self.recorder().start()
        encoder = self.m.MAVLink(None, srcSystem=42, srcComponent=1)
        packet = self.m.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3).pack(encoder)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(packet[:5], ('127.0.0.1', recorder.port))
            self.wait_count(recorder, 1)
            sender.sendto(packet, ('127.0.0.1', recorder.port))
            self.wait_count(recorder, 2)
        recorder.close()
        records = [json.loads(line) for line in (self.root/'raw/mavlink_raw.jsonl').read_text().splitlines()]
        self.assertEqual(records[0]['message_name'], 'BAD_DATA')
        self.assertEqual(records[0]['fields']['data'], {'$bytes_hex': bytes(packet[:5]).hex()})
        self.assertEqual(records[1]['message_name'], 'HEARTBEAT')

    def test_io_failure_surfaces_and_releases_resources_without_overwrite(self):
        """미리 만든 rotation 파일 충돌을 실제로 유발해 worker 오류·FD 정리·원문 보존을 확인한다."""
        recorder = self.recorder(rotate_bytes=1024).start()
        collision = self.root/'raw/mavlink_raw.000001.jsonl'
        collision.write_text('synthetic-existing-do-not-overwrite')
        encoder = self.m.MAVLink(None, srcSystem=42, srcComponent=1)
        packet = self.m.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3).pack(encoder)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(packet, ('127.0.0.1', recorder.port))
            self.wait_count(recorder, 1)
            sender.sendto(packet, ('127.0.0.1', recorder.port))
            sender.sendto(packet, ('127.0.0.1', recorder.port))
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and not recorder.snapshot()['errors']:
            time.sleep(0.01)
        state = recorder.snapshot()
        self.assertTrue(state['errors'], state)
        self.assertIn('FileExistsError', state['errors'][0])
        recorder.close()
        self.assertFalse(recorder.snapshot()['worker_alive'])
        self.assertEqual(collision.read_text(), 'synthetic-existing-do-not-overwrite')
        self.assertEqual(json.loads((self.root/'raw/summary.json').read_text())['exit_reason'], 'error')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', recorder.port))
        with self.assertRaises(FileExistsError):
            self.c.MavlinkRecorder(self.c.DEFAULT_DECODER, self.root/'raw')

    def test_invalid_arguments_precede_socket_creation(self):
        """경로·ID·품질·byte 한도 검증이 소켓 생성보다 앞서는지 검사한다."""
        from unittest import mock
        cases = ({'bind_port': -1}, {'bind_port': True}, {'bind_host': 'localhost'}, {'bind_host': 2130706433},
                 {'source_system': 0}, {'source_component': 256}, {'rotate_bytes': 0},
                 {'rotate_bytes': 1.5}, {'quality_profile': 'typo'})
        for invalid in cases:
            with self.subTest(invalid=invalid), mock.patch.object(self.c.socket, 'socket') as socket_factory:
                with self.assertRaises(ValueError):
                    self.c.MavlinkRecorder(self.c.DEFAULT_DECODER, self.root/'invalid', **invalid)
                socket_factory.assert_not_called()
        self.assertFalse((self.root/'invalid').exists())

    def test_positive_cli_silence_and_data_finish_with_summary(self):
        """양의 짧은 CLI 기간에서 무수신3·실제 합성 수신0과 최종 파일을 확인한다."""
        import select
        import subprocess
        import sys
        for send_packet, expected in ((False, 3), (True, 0)):
            with self.subTest(send_packet=send_packet):
                process = subprocess.Popen([sys.executable, str(HERE/'collector.py'), '--duration', '0.2',
                        '--endpoint', 'udpin:127.0.0.1:0', '--output-root', str(self.root),
                        '--label', 'synthetic-finite'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    ready, _, _ = select.select([process.stdout], [], [], 3)
                    self.assertTrue(ready)
                    line = process.stdout.readline()
                    self.assertTrue(line.startswith('READY '), line)
                    directory = Path(line.strip()[6:])
                    metadata = json.loads((directory/'metadata.json').read_text())
                    self.assertEqual(metadata['quality_profile'], 'standard')
                    port = int(metadata['endpoint'].split(':')[-1])
                    if send_packet:
                        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                            sender.sendto(self.m.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3).pack(
                                self.m.MAVLink(None, srcSystem=42, srcComponent=1)), ('127.0.0.1', port))
                    _, stderr = process.communicate(timeout=3)
                    self.assertEqual(process.returncode, expected, stderr)
                    summary = json.loads((directory/'summary.json').read_text())
                    self.assertEqual(summary['saved_total'], int(send_packet))
                    self.assertEqual(summary['errors'], [])
                finally:
                    if process.poll() is None:
                        process.kill()
                    process.communicate(timeout=3)

    def test_start_failure_keeps_errors_and_does_not_leak_socket(self):
        """이미 사용 중인 로컬 포트로 bind 실패를 반복해 실제 FD와 오류 summary를 확인한다."""
        import os
        before = len(os.listdir('/proc/self/fd'))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
            occupied.bind(('127.0.0.1', 0))
            for index in range(3):
                directory = self.root/f'collision-{index}'
                recorder = self.c.MavlinkRecorder(self.c.DEFAULT_DECODER, directory,
                                                  bind_port=occupied.getsockname()[1])
                with self.assertRaises(OSError):
                    recorder.start()
                recorder.close()
                self.assertTrue(recorder.snapshot()['errors'])
                self.assertEqual(json.loads((directory/'summary.json').read_text())['exit_reason'], 'error')
            self.assertEqual(len(os.listdir('/proc/self/fd')), before+1)
        self.assertEqual(len(os.listdir('/proc/self/fd')), before)

    def test_library_exclusive_bind_receive_identity_and_close(self):
        """실제 합성 heartbeat를 원문 저장하고 독점 bind·무응답·종료를 확인한다."""
        recorder = self.recorder(source_system=42)
        self.assertIs(recorder.start(), recorder)
        port = recorder.port
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rival:
            rival.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            with self.assertRaises(OSError):
                rival.bind(('127.0.0.1', port))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1', 0))
            peer_port = sender.getsockname()[1]
            encoder = self.m.MAVLink(None, srcSystem=42, srcComponent=1)
            encoder.seq = 91
            message = self.m.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3)
            sender.sendto(message.pack(encoder), ('127.0.0.1', port))
            self.wait_count(recorder, 1)
            sender.settimeout(0.05)
            with self.assertRaises(socket.timeout):
                sender.recvfrom(1024)
        self.assertEqual(recorder.latest('HEARTBEAT')['base_mode'], 0)
        recorder.close()
        recorder.close()
        state = recorder.snapshot()
        self.assertFalse(state['worker_alive'])
        self.assertEqual(state['saved_total'], 1)
        json.dumps(state, allow_nan=False)
        record = json.loads((self.root / 'raw/mavlink_raw.jsonl').read_text())
        self.assertEqual(record['source_system'], 42)
        self.assertEqual(record['source_component'], 1)
        self.assertEqual(record['packet_sequence'], 91)
        self.assertEqual(record['peer'], ['127.0.0.1', peer_port])
        self.assertEqual(record['fields'], message.to_dict())
        self.assertEqual(json.loads((self.root / 'raw/summary.json').read_text())['saved_total'], 1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rival:
            rival.bind(('127.0.0.1', port))


if __name__ == '__main__':
    unittest.main()
