"""운영 서비스의 합성 설정/로컬 자원 시험; CMU 실측 또는 비행 증거가 아니다."""
import importlib
import os
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'tools'))
ARTIFACTS = REPO.parent / 'hermes/cmu-service/service-tests'
ARTIFACTS.mkdir(parents=True, exist_ok=True)


def synthetic_catalog():
    """SDF 전달 검사에만 쓸 합성 네 프로파일을 만들며 실측 출처를 붙이지 않는다."""
    return {'schema': 'arion.cmu_battery_profiles.v1', 'series_cells': 4,
            'parallel_cells': 2, 'default_profile_id': 'test100',
            'profiles': [{'id': 'test' + str(p), 'label_ko': 'SYNTHETIC ONLY',
                          'capacity_ah': 6 * p / 100, 'pack_resistance_ohm': 0.04,
                          'initial_soc': 0.95, 'soh_percent': p,
                          'ocv_soc': [0.0, 0.5, 1.0], 'ocv_cell_v': [3.0, 3.7, 4.2],
                          'source': {'test_fixture': 'synthetic_NOT_CMU'}}
                         for p in [100, 95, 90, 85]]}


class ModelConfigTest(unittest.TestCase):
    def test_catalog_reaches_sdf_without_unit_or_profile_loss(self):
        """실제 실행기 SDF 생성 경로가 4개 팩 수치·셀 곡선과 fallback-off를 보존해야 한다."""
        spec = importlib.util.find_spec('sim_service')
        self.assertIsNotNone(spec, '운영용 sim_service 패키지 미구현')
        m = importlib.import_module('sim_service.model_config')
        data = m.validate_catalog(synthetic_catalog())
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            sdf = m.write_model(Path(d), data, 'test90')
            root = ET.parse(sdf).getroot()
            plugin = root.find('model/plugin')
            self.assertEqual(plugin.findtext('initial_profile'), 'test90')
            self.assertEqual(plugin.findtext('allow_command_fallback'), 'false')
            rows = plugin.findall('battery_profiles/profile')
            self.assertEqual(len(rows), 4)
            self.assertEqual(float(rows[2].findtext('capacity_ah')), data['profiles'][2]['capacity_ah'])
            self.assertEqual(rows[2].findtext('ocv_cell_v'), '3.0 3.7 4.2')
    def test_world_statically_includes_the_vehicle_for_full_reset(self):
        """실제 reset이 동적 spawn 기체를 삭제했던 오류: 초기 world SDF에 기체가 있어야 한다."""
        m = importlib.import_module('sim_service.model_config')
        self.assertTrue(hasattr(m, 'write_world'), 'reset-safe initial world generator missing')
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            original = Path(d) / 'base.sdf'
            original.write_text('<sdf version="1.9"><world name="default"><gravity>0 0 -9.8</gravity></world></sdf>')
            target = Path(d) / 'world.sdf'
            m.write_world(original, target, 'custom_x500_cmu_35')
            root = ET.parse(target).getroot()
            self.assertEqual(root.findtext('world/include/name'), 'custom_x500_cmu_35')
            self.assertEqual(root.findtext('world/include/uri'), 'model://custom_x500_cmu')
            self.assertEqual(root.findtext('world/gravity'), '0 0 -9.8')
            self.assertIsNone(ET.parse(original).getroot().find('world/include'))
            with self.assertRaises(FileExistsError):
                m.write_world(original, target, 'custom_x500_cmu_35')

    def test_invalid_catalog_is_rejected_before_writing(self):
        """NaN·중복 ID·뒤집힌 SOC·다른 팩 크기/개수는 정상 카탈로그로 실행하지 않는다."""
        m = importlib.import_module('sim_service.model_config')
        for label, mutate in [
            ('NaN', lambda d: d['profiles'][0].update(capacity_ah=float('nan'))),
            ('duplicate', lambda d: d['profiles'][1].update(id='test100')),
            ('soc_order', lambda d: d['profiles'][0].update(ocv_soc=[0, 1, .5])),
            ('curve_length', lambda d: d['profiles'][0].update(ocv_cell_v=[3, 4])),
            ('negative_r', lambda d: d['profiles'][0].update(pack_resistance_ohm=-.1)),
            ('3S', lambda d: d.update(series_cells=3)),
            ('missing_profile', lambda d: d['profiles'].pop()),
            ('bad_id', lambda d: d['profiles'][0].update(id='../other')),
        ]:
            data = synthetic_catalog()
            mutate(data)
            with self.subTest(label=label), tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
                with self.assertRaises(ValueError):
                    m.write_model(Path(d), data, 'test100')
                self.assertEqual(list(Path(d).iterdir()), [])
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            with self.assertRaises(ValueError):
                m.write_model(Path(d), synthetic_catalog(), 'not-a-profile')
            self.assertEqual(list(Path(d).iterdir()), [])


class ProcessResourcesTest(unittest.TestCase):
    def test_owned_console_rotates_and_stops_without_leaking_fds(self):
        """합성 콘솔의 완전한 명령 응답·bounded 로그·실제 소유 프로세스 종료를 확인한다."""
        spec = importlib.util.find_spec('sim_service.processes')
        self.assertIsNotNone(spec, '운영용 프로세스/PTY 소유권 처리 미구현')
        m = importlib.import_module('sim_service.processes')
        import time
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            log = m.RotatingLog(Path(d) / 'rotate.log', limit=32)
            payload = ('0123456789\n' * 40).encode()
            log.write(payload)
            log.close()
            self.assertGreater(len(log.paths), 1)
            self.assertEqual(b''.join(p.read_bytes() for p in log.paths), payload)
            self.assertTrue(all(p.stat().st_size <= 32 for p in log.paths))
            before = len(list(Path('/proc/self/fd').iterdir()))
            child = m.PtyConsole([sys.executable, '-u', '-c',
                "import sys; print('pxh>', flush=True); "
                "exec(\"for line in sys.stdin:\\n print('RESULT:' + line.strip() + '\\\\npxh>', flush=True)\")"],
                Path(d) / 'console.log', env=os.environ.copy(), cwd=Path(d))
            try:
                child.wait_ready(timeout=3)
                reply = child.command('status', timeout=3)
                self.assertIn('RESULT:status', reply)
                self.assertEqual(child.poll(), None)
            finally:
                outcome = child.stop(timeout=3)
            self.assertFalse(outcome['group_alive'])
            self.assertFalse(outcome['forced_kill'])
            self.assertLessEqual(len(list(Path('/proc/self/fd').iterdir())), before)


class LocalApiTest(unittest.TestCase):
    def test_private_socket_roundtrip_exclusive_owner_and_request_limit(self):
        """실제 로컬 UNIX socket만 사용하고 중복 서버/과대 요청을 거부해야 한다."""
        spec = importlib.util.find_spec('sim_service.control_api')
        self.assertIsNotNone(spec, '운영 제어 API 미구현')
        m = importlib.import_module('sim_service.control_api')
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            p = Path(d) / 'control.sock'
            server = m.ControlServer(p, lambda request: {'ok': True, 'echo': request}).start()
            try:
                self.assertEqual(p.stat().st_mode & 0o777, 0o600)
                self.assertEqual(m.request(p, {'action': 'status'})['echo'], {'action': 'status'})
                with self.assertRaises((OSError, RuntimeError)):
                    m.ControlServer(p, lambda request: {}).start()
                with self.assertRaises(ValueError):
                    m.request(p, {'action': 'x' * 17000})
            finally:
                server.close()
            self.assertFalse(p.exists())


class PowerStatusTest(unittest.TestCase):
    def test_completed_status_needs_one_request(self):
        """이미 적용된 합성 status는 재조회 없이 반환하며 마지막 모델 상태를 보존한다."""
        import json
        from types import SimpleNamespace
        from unittest.mock import patch
        m = importlib.import_module('sim_service.runtime')
        rt = m.Runtime({'plugin_build': '/synthetic/not-executed'}, synthetic_catalog())
        rt.power_service = '/arion/synthetic/power_control'
        state = {'ok': True, 'accepted_revision': 3, 'applied_revision': 3, 'profile_id': 'test90'}
        reply = SimpleNamespace(returncode=0, stdout=json.dumps(state), stderr='')
        with patch.object(m.subprocess, 'run', return_value=reply) as call:
            self.assertEqual(rt.power(), state)
        self.assertEqual(call.call_count, 1)
        self.assertEqual(rt.power_state, state)
        self.assertEqual(rt.profile_id, 'test90')

    def test_pending_commands_still_wait_for_application(self):
        """accepted-only ACK와 미적용 status 모두 실제 applied revision까지 기다린다."""
        import json
        from types import SimpleNamespace
        from unittest.mock import patch
        m = importlib.import_module('sim_service.runtime')
        for initial in ({'ok': True, 'accepted_revision': 4},
                        {'ok': True, 'accepted_revision': 4, 'applied_revision': 3}):
            with self.subTest(initial=initial):
                rt = m.Runtime({'plugin_build': '/synthetic/not-executed'}, synthetic_catalog())
                rt.power_service = '/arion/synthetic/power_control'
                pending = {'ok': True, 'accepted_revision': 4, 'applied_revision': 3}
                done = dict(pending, applied_revision=4, profile_id='test95')
                replies = [SimpleNamespace(returncode=0, stdout=json.dumps(s), stderr='')
                           for s in (initial, pending, done)]
                with patch.object(m.subprocess, 'run', side_effect=replies) as call, patch.object(m.time, 'sleep'):
                    self.assertEqual(rt.power('clear'), done)
                self.assertEqual(call.call_count, 3)
                self.assertEqual(rt.power_state, done)

    def test_power_errors_and_no_wait_are_preserved(self):
        """CLI 실패·거부·적용 timeout은 계속 실패하며 wait=False는 ACK를 즉시 반환한다."""
        import json
        from types import SimpleNamespace
        from unittest.mock import patch
        m = importlib.import_module('sim_service.runtime')
        rt = m.Runtime({'plugin_build': '/synthetic/not-executed'}, synthetic_catalog())
        rt.power_service = '/arion/synthetic/power_control'
        ack = {'ok': True, 'accepted_revision': 4}
        reply = SimpleNamespace(returncode=0, stdout=json.dumps(ack), stderr='')
        with patch.object(m.subprocess, 'run', return_value=reply) as call:
            self.assertEqual(rt.power('clear', wait=False), ack)
            self.assertEqual(call.call_count, 1)
        with patch.object(m.subprocess, 'run', return_value=reply), \
                patch.object(m.time, 'monotonic', side_effect=[0, 9]):
            with self.assertRaises(TimeoutError):
                rt.power('clear')
        for code, value in ((1, ack), (0, {'ok': False})):
            with self.subTest(code=code), patch.object(m.subprocess, 'run',
                    return_value=SimpleNamespace(returncode=code, stdout=json.dumps(value), stderr='test rejection')):
                with self.assertRaises(RuntimeError):
                    rt.power()


class ServiceTelemetryTest(unittest.TestCase):
    def test_nonfinite_cache_values_remain_strict_json(self):
        """합성 cache 경계: 원문 수치는 유지하고 상태 파일/API만 기존 JSON 태그로 표현한다."""
        import json
        import math
        import time
        m = importlib.import_module('sim_service.runtime')
        collector = importlib.import_module('mavlink_collector.collector')
        api = importlib.import_module('sim_service.control_api')
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            rt = m.Runtime({}, synthetic_catalog())
            recorder = collector.MavlinkRecorder(Path(__file__), Path(d) / 'not-started')
            rt.recorder = recorder
            now = time.monotonic_ns()
            recorder._latest = {
                'ESC_STATUS': (now, {'voltage': [float('nan'), float('inf'), -float('inf'), 0.0]}),
                'BATTERY_STATUS': (now, {'voltages': [3700, 65535], 'current_battery': -1}),
                'HEARTBEAT': (now, {'base_mode': 0}),
                'ATTITUDE': (now - 10_000_000_000, {'roll': float('nan')}),
            }
            result = rt.telemetry()
            try:
                json.dumps(result, allow_nan=False)
            except ValueError as error:
                self.fail('nonfinite telemetry broke strict JSON: ' + str(error))
            expected = [{'$float': 'nan'}, {'$float': 'inf'}, {'$float': '-inf'}, 0.0]
            self.assertEqual(result['messages']['ESC_STATUS']['voltage'], expected)
            self.assertEqual(result['messages']['BATTERY_STATUS'], recorder.latest('BATTERY_STATUS'))
            self.assertEqual(result['messages']['ATTITUDE'], {})
            self.assertEqual(result['messages']['HEARTBEAT'], {'base_mode': 0})
            m.atomic_json(Path(d) / 'status.json', rt.status())
            saved = json.loads((Path(d) / 'status.json').read_text())
            self.assertEqual(saved['telemetry']['messages']['ESC_STATUS']['voltage'], expected)
            path = Path(d) / 'control.sock'
            server = api.ControlServer(path, rt.dispatch).start()
            try:
                for action in ('telemetry', 'status'):
                    response = api.request(path, {'action': action}, timeout=2)
                    messages = response['messages'] if action == 'telemetry' else response['telemetry']['messages']
                    self.assertEqual(messages['ESC_STATUS']['voltage'], expected)
                    self.assertTrue(response['ok'])
            finally:
                server.close()
                recorder.close()
            result['messages']['ESC_STATUS']['voltage'][0]['$float'] = 'changed copy'
            self.assertTrue(math.isnan(recorder.latest('ESC_STATUS')['voltage'][0]))
            self.assertFalse(path.exists())
            self.assertFalse((Path(d) / 'not-started').exists())


class RecorderShutdownTest(unittest.TestCase):
    def test_summary_io_failure_fails_service_exit(self):
        """실제 summary 파일 충돌을 숨기지 않는다; decoder/UDP/시뮬레이터는 시작하지 않는다."""
        import contextlib
        import io
        import json
        from unittest.mock import patch
        m = importlib.import_module('sim_service.runtime')
        collector = importlib.import_module('mavlink_collector.collector')
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            root = Path(d)
            rt = m.Runtime({'state_dir': d}, synthetic_catalog())
            rt.folder = root
            recorder = collector.MavlinkRecorder(Path(__file__), root / 'telemetry')
            recorder.output_dir.mkdir()
            recorder._owns_directory = True
            collision = recorder.output_dir / 'summary.json'
            collision.write_text('synthetic collision; must not overwrite')
            rt.recorder = recorder
            rt.event_log = m.RotatingLog(root / 'events.jsonl')
            with patch.object(rt, 'start', side_effect=rt.stop_requested.set), contextlib.redirect_stdout(io.StringIO()):
                result = rt.run_forever()
            self.assertEqual(result, 1, 'recorder final I/O failure reported as service success')
            self.assertEqual(rt.state, 'failed')
            saved = json.loads((root / 'service.summary.json').read_text())
            self.assertFalse(saved['ok'])
            self.assertIn('FileExistsError', str(saved['errors']))
            self.assertIn('FileExistsError', str(saved['recorder']['errors']))
            self.assertEqual(json.loads((root / 'status.json').read_text()), saved)
            event = json.loads((root / 'events.jsonl').read_text())
            self.assertEqual(event['kind'], 'service_stopped')
            self.assertEqual(event['errors'], saved['errors'])
            self.assertEqual(collision.read_text(), 'synthetic collision; must not overwrite')

    def test_final_worker_health_controls_exit_status(self):
        """종료 seam의 정상·오류·남은 worker를 구분하고 정상 종료 0은 유지한다."""
        import contextlib
        import io
        from types import SimpleNamespace
        from unittest.mock import patch
        m = importlib.import_module('sim_service.runtime')
        for errors, alive, expected in (([], False, 0), (['synthetic shutdown timeout'], True, 1), ([], True, 1)):
            with self.subTest(errors=errors, alive=alive):
                rt = m.Runtime({}, synthetic_catalog())
                rt.recorder = SimpleNamespace(close=lambda: None, latest=lambda *a, **kw: {},
                    snapshot=lambda: {'errors': errors, 'worker_alive': alive})
                with patch.object(rt, 'start', side_effect=rt.stop_requested.set), contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(rt.run_forever(), expected)
                self.assertEqual(rt.state, 'failed' if expected else 'stopped')
                self.assertEqual(rt.status()['ok'], expected == 0)


class ServicePolicyTest(unittest.TestCase):
    def test_control_validation_rejects_console_injection_and_bad_factors(self):
        """서비스 공개 입력은 화이트리스트와 수치 범위를 통과해야 하며 임의 shell/PX4 코드는 없다."""
        spec = importlib.util.find_spec('sim_service.runtime')
        self.assertIsNotNone(spec, '운영 Runtime 미구현')
        m = importlib.import_module('sim_service.runtime')
        self.assertEqual(m.validate_action({'action': 'motor-fault', 'motor': 1,
                            'current_scale': 1.6, 'rpm_scale': .85})['motor'], 1)
        for request in [{'action': 'console', 'command': 'shutdown'},
                        {'action': 'takeoff', 'height': float('nan')},
                        {'action': 'takeoff', 'height': 50},
                        {'action': 'motor-fault', 'motor': 0},
                        {'action': 'motor-fault', 'motor': True},
                        {'action': 'motor-fault', 'motor': 1, 'current_scale': 9},
                        {'action': 'profile', 'id': 'healthy\nshutdown'},
                        {'action': 'stop', 'command': 'anything'}]:
            with self.subTest(request=request), self.assertRaises(ValueError):
                m.validate_action(request)


    def test_runtime_blocks_profile_reset_and_world_reset_while_armed(self):
        """합성 armed heartbeat에서 파괴적 모델 변경이 실제 제어 함수에 도달하지 않아야 한다."""
        m = importlib.import_module('sim_service.runtime')
        self.assertTrue(hasattr(m, 'Runtime'), '운영 lifecycle/비행 안전 상태기계 미구현')
        from types import SimpleNamespace
        rt = m.Runtime({}, synthetic_catalog())
        rt.state = 'running'
        rt.recorder = SimpleNamespace(latest=lambda name, **kw: {'base_mode': 128})
        for action in ['profile', 'reset', 'restore', 'world-reset', 'stop']:
            req = {'action': action}
            if action == 'profile':
                req['id'] = 'test90'
            with self.subTest(action=action), self.assertRaisesRegex(RuntimeError, 'Disarm|land'):
                rt.dispatch(req)
        self.assertFalse(rt.stop_requested.is_set())


class WorldResetLifecycleTest(unittest.TestCase):
    def test_world_reset_reboots_px4_across_clock_epoch(self):
        """실제 역행 clock의 수신 정지를 재현한 뒤, 종료→세계reset→새PX4 순서로 분리한다."""
        from unittest.mock import Mock
        m = importlib.import_module('sim_service.runtime')
        rt = m.Runtime({}, synthetic_catalog())
        rt.state = 'running'
        self.assertTrue(hasattr(rt, '_world_reset'), 'PX4 clock epoch reset handling missing')
        order = []
        states = iter([{'reset_count': 0, 'profile_id': 'test100'},
                       {'reset_count': 1, 'profile_id': 'test100', 'consumed_mah': .1}])
        rt.power = lambda: next(states)
        rt._stop_px4_for_reset = lambda: order.append('stop_px4')
        rt.world = lambda req: order.append('reset_world')
        rt._start_px4 = lambda: order.append('start_px4')
        rt.wait_for = lambda predicate, seconds, description: predicate()
        rt._require_disarmed = lambda: order.append('disarmed_readback')
        result = rt._world_reset()
        self.assertEqual(order, ['stop_px4', 'reset_world', 'start_px4', 'disarmed_readback'])
        self.assertEqual(result['px4_epoch'], 1)
        self.assertEqual(rt.state, 'running')
        rt._stop_px4_for_reset = Mock(side_effect=RuntimeError('shutdown failed'))
        rt.power = lambda: {'reset_count': 1}
        with self.assertRaisesRegex(RuntimeError, 'shutdown failed'):
            rt._world_reset()
        self.assertEqual(rt.state, 'failed')
        self.assertTrue(rt.stop_requested.is_set())


class CliTest(unittest.TestCase):
    def test_invalid_init_creates_no_configuration(self):
        """잘못된 인스턴스 번호는 사용자 설정 파일을 남기기 전에 거부한다."""
        import subprocess
        with tempfile.TemporaryDirectory(dir=ARTIFACTS) as d:
            target = Path(d) / 'bad.json'
            argv = [sys.executable, str(REPO / 'tools/arion_sim.py'), '--config', str(target), 'init',
                    '--px4-binary', d, '--plugin-build', d, '--decoder', d, '--ipc-dir', d, '--instance', '0']
            result = subprocess.run(argv, text=True, capture_output=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(target.exists(), 'invalid init left a configuration behind')

    def test_operator_cli_explains_supported_controls_without_starting(self):
        """--help만으로 비행을 시작하지 않고 직접 쓸 제어 명령이 노출되어야 한다."""
        import subprocess
        result = subprocess.run([sys.executable, str(REPO / 'tools/arion_sim.py'), '--help'],
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        for action in ['profiles', 'motor-fault', 'world-reset', 'watch', 'stop']:
            self.assertIn(action, result.stdout)


if __name__ == '__main__':
    unittest.main()
