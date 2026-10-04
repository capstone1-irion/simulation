"""배터리 sender 집중 시험: 모델/서버 없이 합성 C++ seam과 loopback만 사용한다."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

SIMULATION = Path(__file__).resolve().parents[1]
ARTIFACTS = SIMULATION.parent / 'hermes/cmu-service/io'
MAVLINK = SIMULATION.parent / 'hermes/motor-fixed/build/px4/mavlink'


class SenderOptionsTests(unittest.TestCase):
    """실제 제품 translation unit을 기존 dependency로 빌드하며 설치/시뮬레이터 실행은 하지 않는다."""
    @classmethod
    def setUpClass(cls):
        """프로젝트 내 새 폴더에 합성 옵션 시험 binary를 만든다; 컴파일 원문을 보존한다."""
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        cls.root = Path(tempfile.mkdtemp(prefix='synthetic-sender-', dir=ARTIFACTS))
        cls.binary = cls.root / 'sender_options'
        source = cls.root / 'sender_options.cpp'
        source.write_text('#define main sender_program_main\n#include "' + str(SIMULATION/'tools/battery_sender.cpp') + '"\n#undef main\n'
                          'int main(int argc, char **argv) {\n'
                          ' try { auto opt=parse_options(argc, argv);\n'
                          ' std::cout << opt.duration_s << "\\n"; return 0; }\n'
                          ' catch (const std::exception &e) {std::cerr << e.what(); return 2;} }\n')
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport13'], text=True))
        command = ['g++', '-std=c++17', '-Wno-address-of-packed-member', '-I'+str(SIMULATION/'include'),
                   '-I'+str(MAVLINK), str(source), '-o', str(cls.binary), *flags]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        (cls.root/'build.log').write_text(result.stdout)
        if result.returncode:
            raise RuntimeError(f'sender fixture compile failed: {cls.root}/build.log')

    def invoke(self, *extra):
        """송신 없는 옵션 파서를 실행한다; ID·임시 목적지 예시는 해석만 하고 사용하지 않는다."""
        command = [str(self.binary), '--send', '--topic', '/synthetic/battery',
                   '--px4-port', '32123', '--system-id', '42', *extra]
        return subprocess.run(command, env={**os.environ, 'GZ_PARTITION': 'synthetic-options-only'},
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)

    def test_wall_freshness_drops_old_frames_and_accepts_reset_zero(self):
        """순수 steady-clock seam에서 6초 공백·오래된 새 generation·sim0 복구를 검사한다."""
        source = self.root / 'freshness.cpp'
        source.write_text('#define main sender_program_main\n#include "' + str(SIMULATION/'tools/battery_sender.cpp') + '"\n#undef main\n' + r'''
int main() {
    using namespace std::chrono;
    const Clock::time_point start{};
    DeliveryState delivery(start);
    Mailbox state;
    Sample sample;
    sample.sim_ns=1000000000;
    sample.received=start;
    accept_sample(state, sample);
    if (!delivery.poll(state, start).send) return 10;
    if (delivery.poll(state, start + seconds(1)).send) return 11;
    auto missing=delivery.poll(state, start + seconds(6));
    if (missing.send || !missing.missing_event || !delivery.missing) return 12;
    if (delivery.poll(state, start + seconds(7)).missing_event) return 13;
    sample.sim_ns=2000000000;
    sample.received=start + seconds(1);
    accept_sample(state, sample);
    if (delivery.poll(state, start + seconds(7)).send || !delivery.missing) return 14;
    sample.sim_ns=0;
    sample.received=start + seconds(8);
    accept_sample(state, sample);
    auto recovered=delivery.poll(state, start + seconds(8));
    if (!recovered.send || !recovered.recovered_event || delivery.missing) return 15;
    if (recovered.sample.sim_ns!=0 || state.resets!=1) return 16;
    accept_sample(state, sample);
    if (delivery.poll(state, start + seconds(8)).send || state.duplicates!=1) return 17;
    // 처리 시각이 아니라 실제 수신 시각부터 freshness를 계산한다.
    DeliveryState delayed(start);
    Mailbox delayed_state;
    sample.received=start;
    accept_sample(delayed_state, sample);
    if (delayed.poll(delayed_state, start + milliseconds(1100)).send) return 18;
    return 0;
}
''')
        binary = self.root/'freshness'
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport13'], text=True))
        compiled = subprocess.run(['g++', '-std=c++17', '-Wno-address-of-packed-member',
                        '-I'+str(SIMULATION/'include'), '-I'+str(MAVLINK), str(source), '-o', str(binary), *flags],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        (self.root/'freshness-build.log').write_text(compiled.stdout)
        self.assertEqual(compiled.returncode, 0, compiled.stdout)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_continuous_forwarder_pause_recovery_and_signals(self):
        """실제 loopback으로 5초 넘는 합성 공백 동안 재송신0·복구·SIGTERM/INT 종료를 확인한다."""
        import json
        import signal
        import socket
        import time
        source = self.root/'forward.cpp'
        source.write_text('#define main sender_program_main\n#include "' + str(SIMULATION/'tools/battery_sender.cpp') + '"\n#undef main\n#include <thread>\n' + r'''
int main(int argc, char **argv) {
    auto state=std::make_shared<Mailbox>();
    Options options;
    options.port=static_cast<uint16_t>(std::stoi(argv[1]));
    options.system_id=42;
    options.duration_s=0;
    options.wait_when_stale=true;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    UdpSocket socket(options.port);
    Sample sample;
    arion::decode_battery_frame(arion::kFixedBatteryValues.data(), 9, sample.values);
    sample.sim_ns=1000000000;
    sample.received=Clock::now();
    accept_sample(*state,sample);
    std::thread producer([state, sample]() mutable {
        const auto until=Clock::now()+std::chrono::milliseconds(5500);
        while (!stop_requested && Clock::now()<until) {std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        if (!stop_requested) {
            std::lock_guard<std::mutex> lock(state->mutex);
            sample.sim_ns=0;
            sample.received=Clock::now();
            accept_sample(*state,sample);
        }
    });
    int code=1;
    try {code=forward_frames(options,state,socket);} catch (const std::exception &error) {std::cerr<<error.what();}
    stop_requested=1;
    producer.join();
    return code;
}
''')
        binary = self.root/'forward'
        flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport13'], text=True))
        compiled = subprocess.run(['g++', '-std=c++17', '-Wno-address-of-packed-member',
                        '-I'+str(SIMULATION/'include'), '-I'+str(MAVLINK), str(source), '-o', str(binary), *flags],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
        (self.root/'forward-build.log').write_text(compiled.stdout)
        self.assertEqual(compiled.returncode, 0, compiled.stdout)
        for sig in (signal.SIGTERM, signal.SIGINT):
            with self.subTest(signal=sig), socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
                receiver.bind(('127.0.0.1', 0))
                receiver.settimeout(2)
                process = subprocess.Popen([str(binary), str(receiver.getsockname()[1])],
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    packet, _ = receiver.recvfrom(1024)
                    self.assertTrue(packet)
                    if sig == signal.SIGTERM:
                        receiver.settimeout(5.1)
                        with self.assertRaises(socket.timeout):
                            receiver.recvfrom(1024)
                        self.assertIsNone(process.poll(), 'wait mode died during >5s pause')
                        receiver.settimeout(1)
                        self.assertTrue(receiver.recvfrom(1024)[0])
                        receiver.settimeout(0.15)
                        with self.assertRaises(socket.timeout):
                            receiver.recvfrom(1024)
                    before = time.monotonic()
                    process.send_signal(sig)
                    stdout, stderr = process.communicate(timeout=2)
                    (self.root/f'forward-{sig}.stdout').write_text(stdout)
                    (self.root/f'forward-{sig}.stderr').write_text(stderr)
                    self.assertEqual(process.returncode, 0, stderr)
                    self.assertLess(time.monotonic()-before, 1)
                    events = [json.loads(line) for line in stdout.splitlines()]
                    if sig == signal.SIGTERM:
                        self.assertEqual([e['sim_ns'] for e in events if e['event']=='udp_sent'], [1000000000,0])
                        self.assertEqual(sum(e['event']=='missing' for e in events), 1)
                        self.assertEqual(sum(e['event']=='recovered' for e in events), 1)
                finally:
                    if process.poll() is None:
                        process.kill()
                    process.communicate(timeout=2)

    def test_duration_zero_and_explicit_wait_keep_old_defaults(self):
        """duration0/wait를 받되 예전 기본20초와 잘못된 정책 거부를 보존한다."""
        old = self.invoke()
        self.assertEqual(old.returncode, 0, old.stderr)
        self.assertEqual(old.stdout.strip(), '20')
        continuous = self.invoke('--duration', '0', '--stale-policy', 'wait')
        self.assertEqual(continuous.returncode, 0, continuous.stderr)
        self.assertEqual(continuous.stdout.strip(), '0')
        for arguments in (('--duration', '-1'), ('--stale-policy', 'oops'),
                          ('--stale-policy', 'wait', '--stale-policy', 'exit')):
            self.assertNotEqual(self.invoke(*arguments).returncode, 0)


if __name__ == '__main__':
    unittest.main()
