"""역할: 실행 자원·콘솔·pause 복귀 경계·배터리 wire 변환의 합성 검사를 작성한다.
입력: 임시 자식, 분할 문자열, 가상 수신 시각/모터 상태, 배터리 샘플, SDF. 출력: unittest 결과.
호출자: tests/integration.py의 마지막 통합 검증 단계. 작성 중 실행하지 않는다.
"""
from collections import deque
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from pymavlink.dialects.v20 import common

from capstone.battery import load_profile
from capstone.battery_link import BatteryLink
from capstone.collector import Collector
from capstone.processes import Child
from capstone.simulation import Runtime

ROOT = Path(__file__).resolve().parents[1]


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        folder = ROOT / 'hermes' / 'test-runtime'
        folder.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=folder)
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    def console(self, fragments, *, previous='pxh> '):
        """실제 PX4가 아닌 결정론적 수신 조각이다. wait마다 다음 조각을 전달한다."""
        child = Child.__new__(Child)
        child.buffer = previous
        child.offset = len(previous)
        child.error = None
        child.condition = threading.Condition()
        child.command_lock = threading.Lock()
        child.process = Mock()
        child.process.poll.return_value = None
        pending = deque(fragments)

        def receive(_):
            self.assertTrue(pending, '모든 수신 조각 뒤에도 완료 응답을 인식하지 못함')
            fragment = pending.popleft()
            child.buffer += fragment
            child.offset += len(fragment)

        child.send_line = Mock(side_effect=receive)
        waiting = patch.object(child.condition, 'wait', side_effect=receive)
        waiting.start()
        self.addCleanup(waiting.stop)
        return child, pending

    def test_console_waits_through_redraw_and_split_final_prompt(self):
        command = 'param show -q SIM_BAT_ENABLE'
        # 최초 실패 원문의 글자별 redraw/ANSI/CRLF 순서를 합성한다.
        redraw = ''.join('\x1b[2K\rpxh> ' + command[:end]
                         for end in range(1, len(command) + 1))
        trace = redraw + '\r\n\x1b[0m0\r\n\x1b[0mpxh> '
        child, pending = self.console(list(trace))
        self.assertEqual(child.command(command), '0\r\n')
        self.assertFalse(pending)
        child.send_line.assert_called_once_with(command)

    def test_console_ignores_old_reply_and_embedded_prompt_in_log(self):
        command = 'commander check'
        previous = 'pxh> commander check\r\nPreflight check: OK\r\npxh> '
        fragments = ['\rpxh> c', '\npxh> ', '\x1b[2K\rpxh> ' + command,
                     '\nINFO [test] embedded pxh> is not completion\n',
                     'Preflight check: FAIL\n', 'pxh', '> ']
        child, pending = self.console(fragments, previous=previous)
        reply = child.command(command)
        self.assertIn('Preflight check: FAIL', reply)
        self.assertNotIn('Preflight check: OK', reply)
        self.assertFalse(pending)

    def test_console_accepts_completed_command_with_empty_reply(self):
        command = 'param set SIM_BAT_ENABLE 0'
        child, pending = self.console(['\rpxh> p', '\rpxh> ' + command + '\r',
                                       '\n', 'px', 'h> '])
        self.assertEqual(child.command(command), '')
        self.assertFalse(pending)

    def test_console_rejects_invalid_timeout_before_sending(self):
        child, _ = self.console([])
        for timeout in (0, 121, float('nan')):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                child.command('commander takeoff', timeout=timeout)
        child.send_line.assert_not_called()

    def test_console_timeout_does_not_accept_value_without_final_prompt(self):
        command = 'param show -q SIM_BAT_ENABLE'
        child, _ = self.console(['\rpxh> ' + command + '\r\n0\r\n'])
        with patch('capstone.processes.time.monotonic', side_effect=[0, 1]):
            with self.assertRaisesRegex(TimeoutError, '콘솔 응답 대기 초과'):
                child.command(command, timeout=.1)

    def test_console_exit_during_redraw_is_failure(self):
        child, _ = self.console(['\rpxh> p'])
        child.process.poll.return_value = 7
        with self.assertRaisesRegex(RuntimeError, '콘솔이 먼저 종료됨'):
            child.command('param show -q SIM_BAT_ENABLE')

    def test_console_reader_error_is_not_hidden_by_complete_reply(self):
        command = 'param show -q SIM_BAT_ENABLE'
        child, _ = self.console(['\rpxh> ' + command + '\r\n0\r\npxh> '])
        child.error = 'synthetic_log_write_failure'
        with self.assertRaisesRegex(RuntimeError, '콘솔 기록 실패'):
            child.command(command)

    def test_failed_console_start_does_not_leak_descriptors(self):
        before = len(tuple(Path('/proc/self/fd').iterdir()))
        with self.assertRaises(FileNotFoundError):
            Child([str(self.folder / 'missing-binary')], self.folder / 'missing.log',
                  env=os.environ.copy(), cwd=self.folder, console=True)
        self.assertEqual(len(tuple(Path('/proc/self/fd').iterdir())), before)

    def paused_runtime(self):
        """프로세스/UDP 없이 실제 Runtime.wait/healthy와 Collector.latest를 연결한다."""
        runtime = Runtime(output_root=self.folder)
        runtime.state = 'running'
        runtime.paused = True
        runtime.event = Mock()
        runtime.collector = Collector('127.0.0.1', 0, self.folder / 'synthetic', system_id=32)
        runtime.collector._running = True
        self.now_ns = 10_000_000_000
        self.ticks = 0
        self.motor = {'paused': True, 'sim_time_s': 2.0, 'clock_wall_age_s': 8.0}
        self.info = {'motor': self.motor, 'battery': {'soc': .95}, 'error': None, 'age_wall_s': 0}
        runtime.link = Mock()
        runtime.link.snapshot.side_effect = lambda: self.info
        self.heartbeat(runtime, stamp=1_000_000_000)

        def service(*args):
            self.motor['paused'] = args[-1] == 'pause: true'
            return 'data: true\n'

        def sleep(seconds):
            self.now_ns += round(seconds * 1e9)
            self.ticks += 1
            self.on_tick(runtime)

        runtime.gz_service = Mock(side_effect=service)
        self.on_tick = lambda _: None
        for target, action in (
            ('capstone.simulation.time.monotonic', lambda: self.now_ns / 1e9),
            ('capstone.simulation.time.monotonic_ns', lambda: self.now_ns),
            ('capstone.simulation.time.sleep', sleep),
        ):
            replacement = patch(target, side_effect=action)
            replacement.start()
            self.addCleanup(replacement.stop)
        return runtime

    def heartbeat(self, runtime, *, stamp=None, component=1, system=32):
        """값이 같아도 실제 수신 시각이 다른 합성 heartbeat를 공급한다."""
        runtime.collector._update_latest({
            'message_name': 'HEARTBEAT', 'source_system': system, 'source_component': component,
            'receive_monotonic_ns': self.now_ns if stamp is None else stamp,
            'fields': {'base_mode': 29, 'autopilot': 12},
        })

    def advance(self, runtime):
        """pause 해제 뒤 새 SIM tick과 새 PX4 수신을 함께 공급한다."""
        self.motor.update(sim_time_s=2.004, clock_wall_age_s=0)
        self.heartbeat(runtime)

    def test_resume_waits_for_new_heartbeat_and_restores_normal_watchdog(self):
        runtime = self.paused_runtime()
        self.on_tick = self.advance
        runtime.pause(False)
        self.assertEqual(self.ticks, 1)
        self.assertEqual(runtime.state, 'running')
        self.assertFalse(runtime.paused)
        events = [call.args[0] for call in runtime.event.call_args_list]
        self.assertEqual(events, ['pause', 'resume_wait', 'resume_ready'])
        self.assertEqual(runtime.collector._latest['HEARTBEAT'][0], self.now_ns)
        self.now_ns += 3_000_000_001
        with self.assertRaisesRegex(TimeoutError, 'heartbeat가 오래됐습니다'):
            runtime.healthy()

    def test_resume_ignores_fresh_old_heartbeat_and_other_senders(self):
        runtime = self.paused_runtime()
        self.heartbeat(runtime)  # 값도 신선하지만 복귀 경계 이전에 받은 표본이다.

        def incoming(current):
            self.motor.update(sim_time_s=2.004, clock_wall_age_s=0)
            if self.ticks == 1:
                self.heartbeat(current, component=180)
                self.heartbeat(current, system=33)
            else:
                self.heartbeat(current)

        self.on_tick = incoming
        runtime.pause(False)
        self.assertEqual(self.ticks, 2)
        self.assertEqual(runtime.state, 'running')

    def test_resume_times_out_without_heartbeat_and_keeps_old_stamp(self):
        runtime = self.paused_runtime()
        self.on_tick = lambda _: self.motor.update(sim_time_s=2.004, clock_wall_age_s=0)
        with self.assertRaisesRegex(TimeoutError, '재개 후 새 heartbeat와 시계 진행'):
            runtime.pause(False)
        self.assertEqual(self.now_ns, 15_000_000_000)
        self.assertEqual(runtime.state, 'failed')
        self.assertEqual(runtime.collector._latest['HEARTBEAT'][0], 1_000_000_000)
        self.assertTrue(runtime.errors)
        self.assertNotIn('resume_ready', [call.args[0] for call in runtime.event.call_args_list])

    def test_resume_rejects_heartbeat_without_simulation_progress(self):
        runtime = self.paused_runtime()
        self.on_tick = self.heartbeat
        with self.assertRaisesRegex(TimeoutError, '재개 후 새 heartbeat와 시계 진행'):
            runtime.pause(False)
        self.assertEqual(runtime.state, 'failed')

    def test_resume_does_not_hide_process_recording_or_link_errors(self):
        for failure in ('process', 'log', 'collector', 'link', 'gcs', 'transport', 'battery'):
            with self.subTest(failure=failure):
                runtime = self.paused_runtime()
                child = Mock(error=None)
                child.process.poll.return_value = None
                runtime.children['px4'] = child
                runtime._capture_ipc = Mock()  # 실제 /tmp IPC와 무관한 합성 자식이다.
                if failure == 'process':
                    child.process.poll.return_value = 7
                    child.process.returncode = 7
                elif failure == 'log':
                    child.error = 'synthetic_log_error'
                elif failure == 'collector':
                    runtime.collector._fail(RuntimeError('synthetic_collector_error'))
                elif failure == 'link':
                    self.info['error'] = 'synthetic_link_error'
                elif failure == 'gcs':
                    runtime.gcs = Mock(error='synthetic_gcs_error')
                elif failure == 'transport':
                    self.info['age_wall_s'] = 6
                else:
                    self.info['battery']['soc'] = 0
                with self.assertRaises((RuntimeError, TimeoutError)):
                    runtime.pause(False)
                self.assertEqual(self.ticks, 0)
                self.assertEqual(runtime.state, 'failed')

    def test_resume_requires_actual_unpaused_readback(self):
        runtime = self.paused_runtime()
        runtime.gz_service.side_effect = None
        runtime.gz_service.return_value = 'data: true\n'
        with self.assertRaisesRegex(TimeoutError, '실제 pause 상태'):
            runtime.pause(False)
        self.assertEqual(runtime.state, 'failed')
        runtime.event.assert_not_called()

    def test_resume_rejected_service_does_not_change_paused_truth(self):
        runtime = self.paused_runtime()
        runtime.gz_service.side_effect = None
        runtime.gz_service.return_value = 'data: false\n'
        with self.assertRaisesRegex(RuntimeError, 'Gazebo pause 응답 실패'):
            runtime.pause(False)
        self.assertTrue(runtime.paused)
        self.assertEqual(runtime.state, 'failed')
        self.assertEqual(self.ticks, 0)

    def test_stopping_unpauses_without_requiring_healthy_telemetry(self):
        runtime = self.paused_runtime()
        runtime.state = 'stopping'
        runtime.healthy = Mock(side_effect=RuntimeError('already_failed'))
        runtime.pause(False)
        self.assertEqual(runtime.state, 'stopping')
        self.assertFalse(runtime.paused)
        self.assertEqual(self.ticks, 0)
        runtime.healthy.assert_not_called()
        self.assertEqual(runtime.collector._latest['HEARTBEAT'][0], 1_000_000_000)

    def test_pause_does_not_require_heartbeat_while_clock_is_stopped(self):
        runtime = self.paused_runtime()
        runtime.paused = False
        self.motor['paused'] = False
        runtime.pause(True)
        self.assertTrue(runtime.paused)
        self.assertEqual(runtime.state, 'running')
        self.assertEqual(self.ticks, 0)

    def test_running_clock_watchdog_remains_enabled_after_resume(self):
        runtime = self.paused_runtime()
        self.on_tick = self.advance
        runtime.pause(False)
        self.motor['clock_wall_age_s'] = 6
        with self.assertRaisesRegex(TimeoutError, '요청하지 않은 시뮬레이션 시계 정지'):
            runtime.healthy()

    def test_owned_child_is_reaped_and_stop_is_idempotent(self):
        child = Child([sys.executable, '-c', 'print("synthetic_child")'],
                      self.folder / 'child.log', env=os.environ.copy(), cwd=self.folder)
        try:
            child.process.wait(timeout=3)
        finally:
            result = child.stop()
        self.assertEqual(result['returncode'], 0)
        self.assertFalse(result['group_alive'])
        self.assertFalse(result['forced'])
        self.assertEqual(result['errors'], [])
        self.assertEqual(child.stop(), result)

    def test_sender_uses_standard_wire_units_and_unknown_temperature(self):
        link = BatteryLink(self.folder, {}, self.folder, 'synthetic',
                           load_profile('cmu_vah01_rpt000'), .95, 9, 32)
        messages = []
        class Sink:
            def send(self, message):
                messages.append(message)
            battery_status_encode = staticmethod(common.MAVLink_battery_status_message)
        link.encoder = Sink()
        link._send({'voltage_v': 16.0, 'current_a': 5.12, 'soc': .95, 'consumed_mah': 12.4})
        message = messages[0]
        self.assertEqual(message.voltages, [4000] * 4 + [65535] * 6)
        self.assertEqual(message.current_battery, 512)
        self.assertEqual(message.current_consumed, 12)
        self.assertEqual(message.battery_remaining, 95)
        self.assertEqual(message.temperature, 32767)
        self.assertEqual(message.id, 0)
        self.assertEqual(link.sent, 1)

    def test_model_changes_only_approved_motor_route(self):
        original = ET.parse(ROOT / 'external/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf')
        custom = ET.parse(ROOT / 'models/capstone_x500/model.sdf')
        motor_name = 'gz::sim::systems::MulticopterMotorModel'
        def motors(tree):
            return [plugin for plugin in tree.findall('.//plugin') if plugin.get('name') == motor_name]
        old, new = motors(original), motors(custom)
        self.assertEqual(len(old), 4)
        self.assertEqual(len(new), 4)
        for before, after in zip(old, new):
            self.assertEqual(before.attrib, after.attrib)
            before_fields = {element.tag: element.text for element in before}
            after_fields = {element.tag: element.text for element in after}
            self.assertEqual(after_fields.pop('commandSubTopic'), 'command/motor_speed_applied')
            self.assertEqual(before_fields.pop('commandSubTopic'), 'command/motor_speed')
            self.assertEqual(before_fields, after_fields)
        self.assertEqual(custom.find('.//include/uri').text, 'model://x500_base')
        plugins = custom.findall('.//plugin')
        self.assertEqual(len(plugins), 5)
        self.assertEqual(plugins[0].get('name'), 'capstone::MotorFault')


if __name__ == '__main__':
    unittest.main()
