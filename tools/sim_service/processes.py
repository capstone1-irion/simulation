"""운영기가 직접 시작한 세션만 관리한다. tests/harness 또는 전역 kill 명령에 의존하지 않는다."""
import errno
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import threading
import time


class RotatingLog:
    """메모리를 누적하지 않고 파일당 byte 상한으로 원문을 분할한다; 과거 파일은 삭제하지 않는다."""
    def __init__(self, path, limit=16 * 1024 * 1024):
        """새 로그만 생성한다. limit은 파일당 byte이며 전체 디스크 보관 상한은 아니다."""
        if not isinstance(limit, int) or limit < 1:
            raise ValueError('Log size must be a positive integer')
        self.path, self.limit = Path(path), limit
        self.lock, self.paths, self.handle, self.size = threading.RLock(), [], None, 0
        self._open()

    def _open(self):
        """다음 번호의 파일을 배타적으로 열어 기존 수집 로그 덮어쓰기를 방지한다."""
        target = self.path if not self.paths else self.path.with_name(self.path.name + f'.{len(self.paths):06d}')
        self.handle = target.open('xb', buffering=0)
        self.paths.append(target)
        self.size = 0

    def write(self, data):
        """원문 bytes를 순서대로 기록한다. 분할 파일을 이어 붙이면 원문이 복원된다."""
        if isinstance(data, str):
            data = data.encode('utf-8')
        with self.lock:
            view = memoryview(data)
            while view:
                if self.handle is None:
                    raise RuntimeError('Log is closed')
                if self.size == self.limit:
                    self.handle.close()
                    self._open()
                n = min(len(view), self.limit - self.size)
                self.handle.write(view[:n])
                self.size += n
                view = view[n:]

    def close(self):
        """최종 파일을 닫는다. 여러 번 종료해도 다른 파일을 건드리지 않는다."""
        with self.lock:
            if self.handle is not None:
                self.handle.close()
                self.handle = None


def group_alive(pgid):
    """기록한 POSIX 세션 그룹의 존재 여부만 검사한다; 임의 프로세스 검색/종료는 하지 않는다."""
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_group(process, timeout=8):
    """소유한 그룹에 SIGINT 후 제한 시간 내 종료를 기다리고 강제 종료 여부를 숨기지 않는다."""
    result = {'pid': process.pid, 'returncode': process.poll(), 'forced_kill': False,
              'sigint_sent': False, 'group_alive': False}
    if group_alive(process.pid):
        result['sigint_sent'] = True
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    until = time.monotonic() + timeout
    try:
        process.wait(timeout=max(0.01, until - time.monotonic()))
    except subprocess.TimeoutExpired:
        pass
    while group_alive(process.pid) and time.monotonic() < until:
        time.sleep(.05)
    if group_alive(process.pid):
        result['forced_kill'] = True
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        until = time.monotonic() + 3
        while group_alive(process.pid) and time.monotonic() < until:
            time.sleep(.05)
    result.update(returncode=process.poll(), group_alive=group_alive(process.pid))
    return result


class PipeProcess:
    """stdout/stderr를 계속 비우며 소유 PID와 회전 로그를 함께 관리한다."""
    def __init__(self, argv, log_path, *, env, cwd, log_limit=16 * 1024 * 1024):
        """shell 없이 독립 세션으로 실행하며 Popen 실패 시 열린 로그를 즉시 닫는다."""
        self.log = RotatingLog(log_path, log_limit)
        self.error = None
        try:
            self.process = subprocess.Popen(argv, env=env, cwd=cwd, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            self.log.close()
            raise
        self.thread = threading.Thread(target=self._drain, name='drain-' + str(self.process.pid), daemon=True)
        self.thread.start()

    def _drain(self):
        """자식 출력을 읽어 파일로 보낸다. I/O 실패는 상태 검사에 노출한다."""
        try:
            while True:
                block = os.read(self.process.stdout.fileno(), 8192)
                if not block:
                    break
                self.log.write(block)
        except Exception as error:
            self.error = str(error)

    def poll(self):
        """자식의 실제 종료코드를 읽는다; None만 실행 중을 의미한다."""
        return self.process.poll()

    def stop(self, timeout=8):
        """소유 세션을 끝내고 pipe/thread/log를 닫아 종료 증거를 반환한다."""
        result = stop_group(self.process, timeout)
        self.thread.join(timeout=3)
        self.process.stdout.close()
        self.log.close()
        if self.thread.is_alive():
            result['error'] = 'output thread did not stop'
        if self.error:
            result['error'] = self.error
        return result


class PtyConsole:
    """실제 PX4 콘솔용 PTY. 상시 읽기와 명령 직렬화로 버퍼 포화/부분 prompt 오인을 막는다."""
    def __init__(self, argv, log_path, *, env, cwd, log_limit=16 * 1024 * 1024):
        """PTY 양끝을 즉시 소유하며 실행 실패 시 FD와 로그를 모두 닫는다."""
        self.log = RotatingLog(log_path, log_limit)
        self.fd = None
        slave = None
        self.condition, self.command_lock = threading.Condition(), threading.Lock()
        self.buffer, self.offset, self.error = '', 0, None
        self.done = threading.Event()
        try:
            self.fd, slave = pty.openpty()
            self.process = subprocess.Popen(argv, env=env, cwd=cwd, stdin=slave,
                                            stdout=slave, stderr=slave, start_new_session=True)
            os.close(slave)
            slave = None
        except BaseException:
            if slave is not None:
                os.close(slave)
            if self.fd is not None:
                os.close(self.fd)
            self.log.close()
            raise
        self.thread = threading.Thread(target=self._drain, name='px4-console-reader', daemon=True)
        self.thread.start()

    def _drain(self):
        """PTY를 계속 파일로 기록하고 최근 256 KiB 텍스트만 명령 응답 판정에 보관한다."""
        try:
            while not self.done.is_set():
                ready, _, _ = select.select([self.fd], [], [], .1)
                if not ready:
                    continue
                try:
                    block = os.read(self.fd, 8192)
                except OSError as error:
                    if error.errno == errno.EIO:
                        break
                    raise
                if not block:
                    break
                self.log.write(block)
                with self.condition:
                    self.buffer += block.decode('utf-8', errors='replace')
                    drop = max(0, len(self.buffer) - 262144)
                    self.buffer = self.buffer[drop:]
                    self.offset += drop
                    self.condition.notify_all()
        except Exception as error:
            self.error = str(error)
        finally:
            with self.condition:
                self.condition.notify_all()

    def _wait(self, start, timeout, command=None):
        """완전한 명령 echo 뒤의 prompt만 완료로 인정하며 모든 대기는 wall-clock으로 제한한다."""
        until = time.monotonic() + timeout
        with self.condition:
            while time.monotonic() < until:
                if self.error:
                    raise RuntimeError('Console reader: ' + self.error)
                if start < self.offset:
                    raise RuntimeError('Console response exceeded bounded buffer')
                text = self.buffer[start - self.offset:]
                after = 0
                echoed = None if command is None else re.search(re.escape(command) + r'\r?\n', text)
                if command is None or echoed is not None:
                    after = 0 if echoed is None else echoed.end()
                    if 'pxh>' in text[after:]:
                        return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)
                if self.process.poll() is not None:
                    raise RuntimeError('PX4 console exited: ' + text[-2000:])
                self.condition.wait(timeout=min(.1, max(0, until - time.monotonic())))
        raise TimeoutError('Console command did not finish: ' + str(command))

    def wait_ready(self, timeout=100):
        """최초 pxh prompt를 최대 timeout초 기다린다; 이 함수는 비행을 시작하지 않는다."""
        return self._wait(self.offset, timeout)

    def send_line(self, text):
        """소유 콘솔에 한 줄을 쓴다. 줄바꿈 삽입은 금지하며 종료처럼 prompt 없는 명령에도 쓴다."""
        if not isinstance(text, str) or '\n' in text or '\r' in text or len(text) > 1024:
            raise ValueError('Console command must be one bounded line')
        os.write(self.fd, (text + '\n').encode())

    def command(self, text, timeout=15):
        """한 명령씩 전송하고 그 명령의 완료 응답을 반환한다; 외부 사용자의 임의 콘솔 API는 없다."""
        with self.command_lock:
            with self.condition:
                start = self.offset + len(self.buffer)
            self.send_line(text)
            return self._wait(start, timeout, command=text)

    def poll(self):
        """PX4의 실제 종료코드를 읽는다."""
        return self.process.poll()

    def stop(self, timeout=8):
        """소유 그룹과 읽기 스레드를 종료하고 PTY FD/로그를 닫는다."""
        result = stop_group(self.process, timeout)
        self.done.set()
        self.thread.join(timeout=3)
        os.close(self.fd)
        self.log.close()
        if self.thread.is_alive():
            result['error'] = 'console thread did not stop'
        if self.error:
            result['error'] = self.error
        return result
