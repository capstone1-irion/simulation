"""역할: 이 실행이 시작한 프로세스·콘솔·파일만 소유하고 종료한다.
입력: 실행 argv, 환경 사전, run 내부 로그 경로. 출력: 콘솔 응답·종료 결과.
주요 호출자: simulation.py와 battery_link.py. 모든 대기는 벽시계 기준이다.
"""
from __future__ import annotations

import errno
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import threading
import time


class Child:
    """하나의 독립 프로세스 그룹과 로그를 관리한다. 다른 실행의 PID는 건드리지 않는다."""

    def __init__(self, argv, log_path, *, env, cwd, console=False, pipe_output=False):
        if console and pipe_output:
            raise ValueError('console과 pipe_output은 함께 사용할 수 없습니다')
        self.process = None
        self.master = None
        self.reader = None
        self.error = None
        self.buffer = ''
        self.offset = 0
        self.condition = threading.Condition()
        self.command_lock = threading.Lock()
        self.outcome = None
        self.log = Path(log_path).open('ab', buffering=0)
        slave = None
        try:
            if console:
                self.master, slave = pty.openpty()
                self.process = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=slave,
                                                env=env, cwd=cwd, start_new_session=True)
            else:
                self.process = subprocess.Popen(argv, stdin=subprocess.DEVNULL,
                                                stdout=subprocess.PIPE if pipe_output else self.log,
                                                stderr=self.log, env=env, cwd=cwd,
                                                start_new_session=True)
            self.pgid = self.process.pid
            if console:
                self.reader = threading.Thread(target=self._read, name='px4-console', daemon=True)
                self.reader.start()
        except BaseException:
            if self.process is not None:
                self.stop()
            elif self.master is not None:
                os.close(self.master)
                self.master = None
            self.log.close()
            raise
        finally:
            if slave is not None:
                os.close(slave)

    def _read(self):
        """PTY를 계속 비워 PX4가 출력 때문에 멈추지 않게 하고 마지막 응답만 메모리에 둔다."""
        descriptor = self.master
        try:
            while True:
                try:
                    data = os.read(descriptor, 65536)
                except OSError as exc:
                    if exc.errno in (errno.EIO, errno.EBADF):
                        break
                    raise
                if not data:
                    break
                self.log.write(data)
                text = data.decode('utf-8', errors='replace')
                with self.condition:
                    self.offset += len(text)
                    self.buffer = (self.buffer + text)[-262144:]
                    self.condition.notify_all()
        except BaseException as exc:
            self.error = repr(exc)
            with self.condition:
                self.condition.notify_all()

    def tail(self, since=0):
        """콘솔 누적 문자 위치 이후의 제한된 응답을 반환한다. ANSI 장식은 제거한다."""
        with self.condition:
            start = max(0, since - (self.offset - len(self.buffer)))
            return re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', self.buffer[start:])

    def wait_text(self, text, *, since=0, timeout=100, regex=False):
        """문자열 또는 명시한 정규식의 완성을 120초 이하 벽시계 제한으로 기다린다."""
        if not 0 < timeout <= 120:
            raise ValueError('console timeout은 0~120초 범위여야 합니다')
        pattern = re.compile(text if regex else re.escape(text), re.S | re.M)
        end = time.monotonic() + timeout
        with self.condition:
            while True:
                output = self.tail(since)
                if self.error:
                    raise RuntimeError('콘솔 기록 실패: ' + self.error)
                if pattern.search(output):
                    return output
                if self.process.poll() is not None:
                    raise RuntimeError('콘솔이 먼저 종료됨: ' + output[-2000:])
                remaining = end - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('콘솔 응답 대기 초과: ' + output[-2000:])
                self.condition.wait(min(remaining, .2))

    def send_line(self, text):
        """소유한 PX4 콘솔에 한 줄을 쓴다. 외부 장비용 전송 함수가 아니다."""
        if self.master is None or '\n' in text or '\r' in text:
            raise ValueError('열린 콘솔과 단일 명령이 필요합니다')
        data = (text + '\n').encode('utf-8')
        while data:
            data = data[os.write(self.master, data):]

    def command(self, text, timeout=10):
        """완성된 명령 echo와 그 뒤 새 줄의 프롬프트 사이만 반환한다. 성공 여부는 별도 확인한다."""
        if not 0 < timeout <= 120:
            raise ValueError('console timeout은 0~120초 범위여야 합니다')
        # pinned PX4 run_pxh는 입력 글자마다 '\rpxh> '를 다시 그린다.
        # Enter의 개행과 명령 처리 후 프롬프트를 함께 봐야 입력 도중 반환하지 않는다.
        pattern = r'pxh> ' + re.escape(text) + r'\r?\n(.*?)^pxh> '
        with self.command_lock:
            with self.condition:
                since = self.offset
            self.send_line(text)
            output = self.wait_text(pattern, since=since, timeout=timeout, regex=True)
            return re.search(pattern, output, re.S | re.M).group(1)

    def alive(self):
        """직접 자식을 회수한 뒤 그룹의 남은 자손 유무도 확인한다."""
        self.process.poll()
        try:
            os.killpg(self.pgid, 0)
            return True
        except ProcessLookupError:
            return False

    def _wait_group(self, seconds):
        """부모만 종료됐다고 정리가 끝난 것으로 처리하지 않는다."""
        end = time.monotonic() + seconds
        while self.alive() and time.monotonic() < end:
            time.sleep(.05)
        return not self.alive()

    def stop(self, grace=5):
        """SIGINT→SIGTERM→SIGKILL 순서로 소유 그룹만 종료하고 강제 종료를 숨기지 않는다."""
        if self.outcome is not None:
            return self.outcome
        result = {'pid': self.process.pid, 'returncode': None,
                  'signals': [], 'forced': False, 'group_alive': True, 'errors': []}
        for sig, seconds in ((signal.SIGINT, grace), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            if not self.alive():
                break
            result['signals'].append(sig.name)
            result['forced'] |= sig != signal.SIGINT
            try:
                os.killpg(self.pgid, sig)
                self._wait_group(seconds)
            except ProcessLookupError:
                pass
            except BaseException as exc:
                result['errors'].append(repr(exc))
        result['returncode'] = self.process.poll()
        result['group_alive'] = self.alive()
        if self.master is not None:
            os.close(self.master)
            self.master = None
        if self.reader is not None:
            self.reader.join(timeout=2)
            if self.reader.is_alive():
                result['errors'].append('콘솔 reader 종료 확인 실패')
        if self.process.stdout is not None:
            self.process.stdout.close()
        if self.error:
            result['errors'].append(self.error)
        self.log.close()
        self.outcome = result
        return result
