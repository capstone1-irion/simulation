"""동일 사용자 전용 UNIX socket 제어 API. MAVLink 수집기는 이 송수신 경로와 분리된다."""
import json
import os
from pathlib import Path
import socket
import socketserver
import threading

MAX_REQUEST = 16384
MAX_REPLY = 1048576


def request(path, payload, timeout=115):
    """최대 16KiB 요청 한 건을 로컬 서비스에 보내고 제한 시간 안의 JSON 응답만 반환한다."""
    raw = json.dumps(payload, allow_nan=False).encode() + b'\n'
    if len(raw) > MAX_REQUEST:
        raise ValueError('Control request too large')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as channel:
        channel.settimeout(timeout)
        channel.connect(str(path))
        channel.sendall(raw)
        response = bytearray()
        while b'\n' not in response:
            block = channel.recv(min(65536, MAX_REPLY - len(response) + 1))
            if not block:
                raise ConnectionError('Control server closed before replying')
            response.extend(block)
            if len(response) > MAX_REPLY:
                raise ValueError('Control reply too large')
    value = json.loads(response.split(b'\n', 1)[0])
    if not isinstance(value, dict):
        raise ValueError('Invalid control reply')
    return value


class _Handler(socketserver.StreamRequestHandler):
    """한 연결에서 한 요청만 처리한다. 잘못된 입력을 PX4 console 명령으로 실행하지 않는다."""
    def handle(self):
        """짧고 엄격한 JSON 입력을 dispatch하고 오류를 구조화하여 반환한다."""
        self.connection.settimeout(5)
        try:
            raw = self.rfile.readline(MAX_REQUEST + 1)
            if not raw.endswith(b'\n') or len(raw) > MAX_REQUEST:
                raise ValueError('Incomplete or oversized control request')
            value = json.loads(raw)
            if not isinstance(value, dict) or not isinstance(value.get('action'), str):
                raise ValueError('Request must be an object with action')
            result = self.server.dispatch(value)
        except Exception as error:
            result = {'ok': False, 'error': type(error).__name__ + ': ' + str(error)}
        try:
            self.wfile.write(json.dumps(result, ensure_ascii=False, allow_nan=False).encode() + b'\n')
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass


class _Server(socketserver.ThreadingUnixStreamServer):
    """상태 읽기는 비행 명령 대기 중에도 처리하고 실제 동작 직렬화는 Runtime이 맡는다."""
    daemon_threads = True
    allow_reuse_address = False


class ControlServer:
    """배타적으로 만든 제어 socket만 종료 때 제거한다; 타 서비스 socket은 덮어쓰지 않는다."""
    def __init__(self, path, dispatch):
        """아직 socket을 만들지 않고 경로와 요청 처리기만 저장한다."""
        self.path, self.dispatch = Path(path), dispatch
        self.server = self.thread = None
        self.identity = None

    def start(self):
        """중복 경로를 거부한 뒤 mode0600 socket을 만들고 소유 스레드를 시작한다."""
        if self.path.exists() or self.path.is_symlink():
            raise RuntimeError('Control socket already exists: ' + str(self.path))
        if len(os.fsencode(self.path)) >= 108:
            raise ValueError('Control socket path too long')
        self.server = _Server(str(self.path), _Handler)
        try:
            os.chmod(self.path, 0o600)
            st = self.path.stat()
            self.identity = (st.st_dev, st.st_ino)
            self.server.dispatch = self.dispatch
            self.thread = threading.Thread(target=self.server.serve_forever,
                                           kwargs={'poll_interval': .1}, daemon=True,
                                           name='arion-control-api')
            self.thread.start()
        except BaseException:
            self.server.server_close()
            self.path.unlink(missing_ok=True)
            raise
        return self

    def close(self):
        """제어 접수를 끝내고 생성 당시 inode가 같은 socket만 지운다."""
        if self.server is None:
            return
        if self.thread is not None:
            self.server.shutdown()
            self.thread.join(timeout=3)
        self.server.server_close()
        if self.path.exists():
            st = self.path.stat()
            if self.identity == (st.st_dev, st.st_ino):
                self.path.unlink()
        self.server = None
