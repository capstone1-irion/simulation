"""Synthetic ELF bytes only: test preflight without launching PX4 or Gazebo."""
import hashlib
import importlib
import io
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'tools'))
check_runtime_files = importlib.import_module('sim_service.runtime').check_runtime_files

ARTIFACTS = REPO.parent / 'hermes/cmu-service/service-tests'


class RuntimeFilesTest(unittest.TestCase):
    def setUp(self):
        """격리된 합성 실행파일·필수 경로만 만들고 실행 명령은 사용하지 않는다."""
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=ARTIFACTS)
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.binary = root / 'bin/px4'
        self.config = {'px4_binary': str(self.binary), 'px4_source': str(root / 'source'),
                       'plugin_build': str(root / 'plugins'), 'decoder': str(root / 'decoder.py'),
                       'catalog': str(root / 'catalog.json'), 'ipc_dir': str(root / 'ipc-한글'),
                       'instance': 3}
        paths = [self.binary, root / 'decoder.py', root / 'catalog.json',
                 root / 'etc/init.d-posix/rcS', root / 'source/Tools/simulation/gz/worlds/default.sdf',
                 root / 'source/Tools/simulation/gz/models/x500/model.sdf']
        paths.extend(root / 'plugins' / name for name in
                     ('libArionCoupledPowerPlugin.so', 'arion_battery_sender', 'arion_power_control'))
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        self.binary.chmod(0o700)
        ipc = Path(self.config['ipc_dir'])
        self.prefixes = [os.fsencode(ipc / name) for name in ('px4_lock', 'px4-sock-')]
        self.payload = b'\x7fELF' + b'\0'.join(self.prefixes) + b'\0'
        cli = patch('sim_service.runtime.shutil.which', return_value='/synthetic/gz-not-executed')
        cli.start()
        self.addCleanup(cli.stop)

    def test_binary_reads_are_bounded(self):
        """파일 크기만큼 읽지 않고 제한된 읽기로 전체 SHA256을 보존해야 한다."""
        self.binary.write_bytes(self.payload + b'\0' * (3 * 1024 * 1024))
        expected = hashlib.sha256(self.binary.read_bytes()).hexdigest()
        real_open = Path.open
        test = self
        sizes = []

        class Reader(io.BufferedReader):
            def read(self, size: int | None = -1):
                if size is None or size <= 0:
                    test.fail('unbounded binary read')
                test.assertLessEqual(size, 1024 * 1024)
                sizes.append(size)
                return super().read(size)

        def open_checked(path, *args, **kwargs):
            if path == self.binary:
                return Reader(io.FileIO(path, 'rb'))
            return real_open(path, *args, **kwargs)

        with patch.object(Path, 'open', open_checked):
            self.assertEqual(check_runtime_files(self.config), expected)
        self.assertGreater(len(sizes), 1)

    def test_ipc_markers_across_chunk_boundaries(self):
        """금지/필수 IPC 접두사의 모든 경계 분할과 파일 끝을 검사한다."""
        boundary = 4 + 1024 * 1024
        for marker in [b'/tmp/px4_lock', b'/tmp/px4-sock-', *self.prefixes]:
            forbidden = marker.startswith(b'/tmp/')
            prefix = self.payload if forbidden else b'\x7fELF' + next(p for p in self.prefixes if p != marker) + b'\0'
            for split in range(1, len(marker)):
                with self.subTest(marker=marker, split=split):
                    payload = prefix + b'\0' * (boundary - split - len(prefix)) + marker
                    self.binary.write_bytes(payload)
                    if forbidden:
                        with self.assertRaisesRegex(ValueError, 'project-local SITL overlay ELF'):
                            check_runtime_files(self.config)
                    else:
                        self.assertEqual(check_runtime_files(self.config), hashlib.sha256(payload).hexdigest())

    def test_invalid_elf_and_missing_prefix_are_rejected(self):
        """빈 파일·짧은/잘못된 magic·필수 접두사 누락은 이전과 같은 이유로 거부한다."""
        for payload in (b'', b'\x7f', b'\x7fE', b'\x7fEL', b'NOPE' + self.payload):
            with self.subTest(payload=payload):
                self.binary.write_bytes(payload)
                with self.assertRaisesRegex(ValueError, 'project-local SITL overlay ELF'):
                    check_runtime_files(self.config)
        for marker in self.prefixes:
            self.binary.write_bytes(b'\x7fELF' + marker)
            with self.assertRaisesRegex(ValueError, 'IPC prefix does not match'):
                check_runtime_files(self.config)

    def test_unknown_ipc_owner_is_preserved(self):
        """파일 검사 최적화가 기존 instance 소유권 거부를 제거해서는 안 된다."""
        self.binary.write_bytes(self.payload)
        ipc = Path(self.config['ipc_dir']) / 'px4_lock-3'
        ipc.parent.mkdir()
        ipc.write_bytes(b'synthetic owner')
        with self.assertRaisesRegex(RuntimeError, 'unknown owner'):
            check_runtime_files(self.config)
        self.assertEqual(ipc.read_bytes(), b'synthetic owner')


if __name__ == '__main__':
    unittest.main()
