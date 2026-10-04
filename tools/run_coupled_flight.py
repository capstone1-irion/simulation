#!/usr/bin/env python3
"""Owned, loopback-only PX4 SITL battery roundtrip or coupled takeoff/hover/land.

Requires the project-local SITL overlay, never a physical flight controller.
The telemetry collector is receive-only. Flight uses the owned PX4 console.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import socket
import subprocess
import sys
import tempfile
import threading
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'tests'))
from run_motor_fixed_e2e import Console, cleanup_owned_process, record_error
from power_flight_analysis import validate_fixed, validate_flight


def safe_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': repr(value)}
    if isinstance(value, dict):
        return {k: safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    if isinstance(value, bytes):
        return {'bytes_hex': value.hex()}
    return value


class Collector:
    def __init__(self, receiver, decoder, log, system_id):
        self.receiver, self.parser, self.log = receiver, decoder.MAVLink(None), log
        self.parser.robust_parsing = True
        self.system_id = system_id
        self.phase = 'startup'
        self.rows, self.errors = [], []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, name='receive-only-telemetry')

    def run(self):
        try:
            while not self.done.is_set():
                try:
                    packet, peer = self.receiver.recvfrom(65535)
                except socket.timeout:
                    continue
                for message in self.parser.parse_buffer(packet) or []:
                    row = {'phase': self.phase, 'receive_monotonic_ns': time.monotonic_ns(),
                           'receive_utc_ns': time.time_ns(), 'peer': peer,
                           'source_system': message.get_srcSystem(),
                           'source_component': message.get_srcComponent(),
                           'fields': safe_json(message.to_dict())}
                    self.log.write(json.dumps(row, allow_nan=False) + '\n')
                    self.log.flush()
                    self.rows.append(row)
        except BaseException as exc:
            self.errors.append(repr(exc))
            self.done.set()

    def latest(self, kind):
        for row in reversed(self.rows):
            if row['source_system'] == self.system_id and row['source_component'] == 1 and row['fields'].get('mavpackettype') == kind:
                return row['fields']
        return {}

    def close(self):
        self.done.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError('Collector did not stop')
        if self.errors:
            raise RuntimeError('Collector errors: ' + repr(self.errors))


def reserve_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as handle:
        handle.bind(('127.0.0.1', 0))
        return handle.getsockname()[1]


def wait_for(predicate, timeout, label, children, collector):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if collector.errors:
            raise RuntimeError(str(collector.errors))
        for name, process in children.items():
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'{name} exited unexpectedly: {process.returncode}')
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(label + f' exceeded {timeout}s')


def record_interval(collector, phase, seconds, children):
    collector.phase = phase
    deadline = time.monotonic() + seconds
    wait_for(lambda: time.monotonic() >= deadline, seconds + 2, phase, children, collector)


def run(args):
    if not __debug__:
        raise RuntimeError('Run without Python -O/-OO')
    if args.mode == 'flight' and not args.fly:
        raise ValueError('Explicit --fly required for simulated flight')
    if not 1 <= args.instance <= 250:
        raise ValueError('instance must be 1..250')
    args.artifacts.mkdir(parents=True, exist_ok=True)
    args.ipc_dir.mkdir(parents=True, exist_ok=True)
    binary = args.px4_binary.read_bytes()
    if b'/tmp/px4_lock' in binary or b'/tmp/px4-sock-' in binary:
        raise ValueError('Refusing stock /tmp IPC binary; build the project-local SITL overlay')
    for prefix in ['px4_lock', 'px4-sock-']:
        if str(args.ipc_dir / prefix).encode() not in binary:
            raise ValueError('Executable IPC prefix does not match --ipc-dir: ' + prefix)
    del binary
    ipc_paths = [args.ipc_dir / f'px4_lock-{args.instance}', args.ipc_dir / f'px4-sock-{args.instance}']
    if any(p.exists() for p in ipc_paths):
        raise RuntimeError('Refusing to reuse existing instance IPC')
    if len(str(ipc_paths[1]).encode()) >= 108:
        raise ValueError('UNIX socket path is too long')
    folder = Path(tempfile.mkdtemp(prefix=args.mode + '-', dir=args.artifacts))
    for name in ['rootfs', 'home', 'tmp', 'cache', 'config']:
        (folder / name).mkdir()
    start_ns = time.time_ns()
    report = {'passed': False, 'mode': args.mode, 'instance': args.instance,
              'system_id': args.instance + 1, 'receiver_transmits': False,
              'started_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'PX4 SITL only; standard MAVLink; uncalibrated reference power model',
              'commands': [], 'owned_pids': {}, 'cleanup': {}}
    children: dict[str, subprocess.Popen | None] = {'px4': None, 'gazebo': None, 'sender': None, 'observer': None}
    master = slave = receiver = collector = console = None
    acceptance = False
    model = 'custom_x500_battery_fixture' if args.mode == 'fixed' else 'custom_x500_power'
    model_instance = f'{model}_{args.instance}'
    env = os.environ.copy()
    env.update({'GZ_PARTITION': 'arion_' + folder.name, 'GZ_IP': '127.0.0.1',
                'HEADLESS': '1', 'HOME': str(folder / 'home'), 'TMPDIR': str(folder / 'tmp'),
                'XDG_CACHE_HOME': str(folder / 'cache'), 'XDG_CONFIG_HOME': str(folder / 'config'),
                'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_' + model,
                'PX4_GZ_WORLD': 'default', 'PX4_GZ_STANDALONE': '1',
                'PX4_GZ_MODELS': str(REPO / 'models'),
                'PX4_GZ_WORLDS': str(args.px4_source / 'Tools/simulation/gz/worlds'),
                'GZ_SIM_RESOURCE_PATH': str(REPO / 'models') + ':' + str(args.px4_source / 'Tools/simulation/gz/models'),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': str(args.plugin_build),
                'GZ_SIM_SERVER_CONFIG_PATH': str(REPO / 'config/coupled-headless.config'),
                'PX4_PARAM_SIM_BAT_ENABLE': '0', 'PX4_PARAM_BAT1_SOURCE': '1',
                'PX4_PARAM_BAT1_N_CELLS': '4', 'PX4_PARAM_BAT1_CAPACITY': '5000',
                'PX4_PARAM_SIM_GZ_ESC_ST': '0', 'PX4_PARAM_COM_RC_IN_MODE': '4',
                'PX4_PARAM_NAV_DLL_ACT': '0', 'PX4_PARAM_MIS_TAKEOFF_ALT': '3',
                'PX4_PARAM_COM_DISARM_LAND': '2', 'PX4_PARAM_MAV_PROTO_VER': '2'})
    report['environment'] = {k: v for k, v in env.items() if k.startswith(('PX4_', 'GZ_', 'HEADLESS'))}
    report['flight_safety'] = {'force_arm': False, 'force_disarm': False,
                              'manual_control_disabled_for_console_only_sitl': True,
                              'datalink_loss_action_disabled_for_console_only_sitl': True}
    with ExitStack() as files:
        try:
            logs = {name: files.enter_context((folder / name).open('w')) for name in
                    ['px4.log', 'gazebo.log', 'sender.jsonl', 'sender.stderr.log',
                     'producer.jsonl', 'observer.stderr.log', 'mavlink_raw.jsonl']}
            spec = importlib.util.spec_from_file_location('arion_pinned_decoder', args.decoder)
            if spec is None or spec.loader is None:
                raise ValueError('Cannot load decoder')
            decoder = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(decoder)
            if 291 not in decoder.mavlink_map:
                raise ValueError('Decoder lacks ESC_STATUS')
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(0.2)
            destination, output_port, input_port = receiver.getsockname()[1], reserve_port(), reserve_port()
            if len({destination, output_port, input_port}) != 3:
                raise RuntimeError('UDP port reservation collision; retry with a fresh run')
            report['ports'] = {'receiver': destination, 'px4_output': output_port, 'px4_battery_input': input_port}
            collector = Collector(receiver, decoder, logs['mavlink_raw.jsonl'], args.instance + 1)
            collector.thread.start()
            children['gazebo'] = subprocess.Popen(
                ['gz', 'sim', '-s', '-r', str(args.px4_source / 'Tools/simulation/gz/worlds/default.sdf')],
                env=env, stdout=logs['gazebo.log'], stderr=subprocess.STDOUT, start_new_session=True)
            report['owned_pids']['gazebo'] = children['gazebo'].pid
            master, slave = pty.openpty()
            children['px4'] = subprocess.Popen(
                [str(args.px4_binary), '-i', str(args.instance), '-w', str(folder / 'rootfs'),
                 str(args.px4_binary.parent.parent / 'etc')], env=env, cwd=folder / 'rootfs',
                stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
            report['owned_pids']['px4'] = children['px4'].pid
            os.close(slave)
            slave = None
            console = Console(children['px4'], master, logs['px4.log'])
            console.read_until('pxh>', 90)

            def command(text):
                response = console.command(text)
                report['commands'].append({'command': text, 'response': response,
                                           'monotonic_ns': time.monotonic_ns()})
                return response

            status = command('commander status')
            if 'Disarmed' not in status:
                raise RuntimeError('Owned simulator did not start disarmed')
            if args.mode == 'flight':
                # rcS/airframe defaults run after PX4_PARAM_ overrides. Apply this
                # declared console-only SITL policy after startup, never force-arm.
                command('param set NAV_DLL_ACT 0')
                report['console_datalink_policy'] = command('param show NAV_DLL_ACT')
            command('mavlink stop-all')
            command(f'mavlink start -u {output_port} -o {destination} -t 127.0.0.1 -m custom -r 100000')
            command(f'mavlink start -u {input_port} -o 9 -t 127.0.0.1 -m custom -r 100000')
            for stream, rate in [('HEARTBEAT', 2), ('ESC_STATUS', 20), ('BATTERY_STATUS', 5),
                                 ('LOCAL_POSITION_NED', 10), ('EXTENDED_SYS_STATE', 5)]:
                command(f'mavlink stream -u {output_port} -s {stream} -r {rate}')
            topic = '/arion/' + model_instance
            command('arion_motor_bridge start -t ' + topic + '/motor_state')
            report['link_status'] = command('mavlink status')
            for param in ['SIM_BAT_ENABLE', 'BAT1_SOURCE', 'SIM_GZ_ESC_ST']:
                command('param show ' + param)
            for name, executable, argv, out, err in [
                ('sender', 'arion_battery_sender', ['--send', '--topic', topic + '/battery_state',
                    '--px4-port', str(input_port), '--system-id', str(args.instance + 1), '--duration', '120'],
                    'sender.jsonl', 'sender.stderr.log'),
                ('observer', 'arion_power_observer', [model_instance, '120'], 'producer.jsonl', 'observer.stderr.log')]:
                children[name] = subprocess.Popen([str(args.plugin_build / executable), *argv], env=env,
                                                  stdout=logs[out], stderr=logs[err], start_new_session=True)
                report['owned_pids'][name] = children[name].pid
            wait_for(lambda: collector.latest('BATTERY_STATUS') and collector.latest('ESC_STATUS'),
                     20, 'battery and motor telemetry', children, collector)
            if args.mode == 'fixed':
                record_interval(collector, 'fixed', 4, children)
                report['uorb_battery'] = command('listener battery_status -n 1')
                report['uorb_motor'] = command('listener esc_status -n 1')
                for expected in ['id: 1', 'source: 1', 'cell_count: 4', 'temperature: 28.500']:
                    if expected not in report['uorb_battery']:
                        raise RuntimeError('uORB battery missing expected value: ' + expected)
                report['measurement'] = validate_fixed(collector.rows, args.instance + 1)
                if any(r['fields'].get('mavpackettype') == 'HEARTBEAT' and r['fields'].get('base_mode', 0) & 128 for r in collector.rows):
                    raise RuntimeError('Fixed test unexpectedly armed')
            else:
                wait_for(lambda: collector.latest('LOCAL_POSITION_NED'), 30, 'local position', children, collector)
                deadline = time.monotonic() + 30
                while 'Preflight check: OK' not in command('commander check'):
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Preflight checks did not pass; no force-arm attempted')
                    record_interval(collector, 'settling', 2, children)
                command('arion_motor_bridge stop')
                report['restart_stopped'] = command('arion_motor_bridge status')
                if 'not running' not in report['restart_stopped']:
                    raise RuntimeError('Motor bridge did not stop before restart')
                command('arion_motor_bridge start -t ' + topic + '/motor_state')
                record_interval(collector, 'preflight', 3, children)
                z0 = collector.latest('LOCAL_POSITION_NED')['z']
                collector.phase = 'takeoff'
                command('commander takeoff')
                wait_for(lambda: bool(collector.latest('HEARTBEAT').get('base_mode', 0) & 128)
                         and collector.latest('EXTENDED_SYS_STATE').get('landed_state') == 2
                         and z0 - collector.latest('LOCAL_POSITION_NED').get('z', z0) >= 2.5
                         and abs(collector.latest('LOCAL_POSITION_NED').get('vz', 100)) < 0.35,
                         40, 'normal takeoff and settled altitude', children, collector)
                record_interval(collector, 'hover', args.hover_seconds, children)
                report['uorb_battery'] = command('listener battery_status -n 1')
                report['uorb_motor'] = command('listener esc_status -n 1')
                collector.phase = 'land'
                command('commander land')
                wait_for(lambda: collector.latest('EXTENDED_SYS_STATE').get('landed_state') == 1,
                         40, 'on-ground landing', children, collector)
                wait_for(lambda: not collector.latest('HEARTBEAT').get('base_mode', 128) & 128,
                         10, 'automatic disarm after landing', children, collector)
                record_interval(collector, 'landed', 3, children)
                report['measurement'] = validate_flight(collector.rows, args.instance + 1)
            report['final_commander'] = command('commander status')
            if 'Disarmed' not in report['final_commander']:
                raise RuntimeError('Final commander is not disarmed')
            report['final_bridge'] = command('arion_motor_bridge status')
            command('arion_motor_bridge stop')
            for name in ['sender', 'observer']:
                cleanup_owned_process(report, name, children[name])
            os.write(master, b'shutdown\n')
            children['px4'].wait(timeout=15)
            acceptance = True
        except BaseException as exc:
            record_error(report, 'run', exc)
        finally:
            for name in ['sender', 'observer']:
                if name not in report['cleanup']:
                    cleanup_owned_process(report, name, children[name])
            if children['px4'] is not None and children['px4'].poll() is None and master is not None:
                try:
                    os.write(master, b'shutdown\n')
                    children['px4'].wait(timeout=12)
                except BaseException as exc:
                    record_error(report, 'px4 shutdown', exc)
            for name in ['px4', 'gazebo']:
                cleanup_owned_process(report, name, children[name])
            if collector is not None:
                try:
                    collector.close()
                except BaseException as exc:
                    record_error(report, 'collector shutdown', exc)
                report['total_records'] = len(collector.rows)
                report['message_counts'] = dict(Counter(r['fields'].get('mavpackettype') for r in collector.rows))
            if receiver is not None:
                receiver.close()
            for fd in [master, slave]:
                if fd is not None:
                    os.close(fd)
            for name, outcome in report['cleanup'].items():
                allowed = (0, 130) if name == 'sender' else (0,)
                if not outcome['started'] or outcome['returncode'] not in allowed or outcome['forced_kill'] or outcome['group_alive']:
                    record_error(report, name + ' cleanup', RuntimeError(str(outcome)))
            report['ipc_reclaimed'] = []
            for path in ipc_paths:
                if path.exists():
                    info = path.lstat()
                    if info.st_uid == os.getuid() and info.st_ctime_ns >= start_ns and not report['cleanup'].get('px4', {}).get('group_alive', True):
                        destination = folder / ('reclaimed-' + path.name)
                        path.rename(destination)
                        report['ipc_reclaimed'].append(str(destination))
                    else:
                        record_error(report, 'IPC cleanup', RuntimeError('Uncertain ownership: ' + str(path)))
            report['ipc_remaining'] = [str(p) for p in ipc_paths if p.exists()]
            report['finished_utc'] = datetime.now(timezone.utc).isoformat()
            report['passed'] = acceptance and not report.get('errors') and not report['ipc_remaining']
            (folder / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print('RUN_EVIDENCE', folder, flush=True)
    print(json.dumps({'passed': report['passed'], 'error': report.get('error'),
                      'measurement': report.get('measurement'), 'cleanup': report['cleanup']}, ensure_ascii=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    for arg in ['px4-source', 'px4-binary', 'plugin-build', 'decoder', 'artifacts', 'ipc-dir']:
        ap.add_argument('--' + arg, type=lambda x: Path(x).expanduser().resolve(), required=True)
    ap.add_argument('--mode', choices=['fixed', 'flight'], required=True)
    ap.add_argument('--fly', action='store_true')
    ap.add_argument('--instance', type=int, default=35)
    ap.add_argument('--hover-seconds', type=float, default=8.0)
    options = ap.parse_args()
    if not 3 <= options.hover_seconds <= 20:
        ap.error('hover-seconds must be 3..20')
    raise SystemExit(run(options))
