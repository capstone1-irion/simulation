#!/usr/bin/env python3
"""Build a pinned Linux PX4 SITL overlay without writing to its checkout/cache.

Only copied battery receiver/stream and IPC sources change. Always compile the
CURRENT external motor bridge, then reuse immutable cached production objects.
Never run PX4/Gazebo, configure CMake, install packages, or invoke a build target.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time

PINNED_HEAD = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
MAIN = "platforms/posix/src/px4/common/main.cpp"
SOCKET = "platforms/posix/src/px4/common/px4_daemon/sock_protocol.cpp"
RECEIVER = "src/modules/mavlink/mavlink_receiver.cpp"
MESSAGES = "src/modules/mavlink/mavlink_messages.cpp"
STREAM = "src/modules/mavlink/streams/BATTERY_STATUS.hpp"
BRIDGE = "px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp"
PATCH = "patches/px4-v1.17.0/0002-sitl-external-battery.patch"


# 목적: 파일을 청크 단위로 읽어 내용 식별용 SHA-256을 계산한다.
# 입출력: 파일 경로 → 16진수 해시 문자열(읽기 청크는 byte 단위).
# 변경/제약: 설명만 추가, 동작 유지. 전체 파일을 메모리에 올리지 않으며 읽기 오류는 호출자에게 전파한다.
def digest(path):
    with Path(path).open("rb") as stream:
        h = hashlib.sha256()
        # 목적: 반복자에 다음 읽기 청크를 공급하는 지연 호출 람다이다.
        # 입출력: 인자 없음 → 최대 1 MiB의 bytes, 파일 끝에서는 b"".
        # 변경/제약: 설명만 추가, 동작 유지. 열린 stream을 캡처하며 빈 바이트를 반복 종료 표식으로 사용한다.
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


# 목적: 보호 대상 또는 산출물의 비교용 파일 식별 정보를 만든다.
# 입출력: 경로 → 심볼릭 링크 대상 문자열 또는 SHA-256·크기 dict(크기 byte).
# 변경/제약: 설명만 추가, 동작 유지. 링크 자체는 해석한 대상 내용이 아니라 저장된 링크 문자열로 비교한다.
def identity(path):
    path = Path(path)
    if path.is_symlink():
        return {"symlink": os.readlink(path)}
    return {"sha256": digest(path), "size": path.stat().st_size}


# 목적: 경로가 기준 디렉터리 자신 또는 그 하위인지 판별한다.
# 입출력: Path 두 개 → 포함 여부 bool(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 경로 구성만 비교하므로 실제 경로 해석·정규화는 호출자가 해야 한다.
def inside(path, parent):
    return path == parent or parent in path.parents


# 목적: 복사한 PX4 소스의 고정 lock/socket 문자열을 지정 IPC 경로로 치환한다.
# 입출력: 소스 문자열·종류·IPC 디렉터리 → 치환된 소스 문자열(소켓 경로 한도는 byte).
# 변경/제약: 설명만 추가, 동작 유지. 절대 경로·/tmp 밖·경로 길이·원본 문자열 1개를 요구하며 파일은 쓰지 않는다.
def relocate_ipc(text, kind, ipc_dir):
    ipc_dir = Path(ipc_dir)
    if not ipc_dir.is_absolute() or inside(ipc_dir, Path("/tmp")):
        raise ValueError("IPC must be an absolute, project-local directory, not /tmp")
    if len(os.fsencode(str(ipc_dir / "px4-sock--2147483648"))) >= 108:
        raise ValueError("IPC socket path exceeds Linux sockaddr_un.sun_path")
    old, name = {"lock": ('"/tmp/px4_lock"', "px4_lock"),
                 "socket": ('"/tmp/px4-sock-"', "px4-sock-")}[kind]
    if text.count(old) != 1:
        raise ValueError(f"expected exactly one pinned {kind} path literal")
    return text.replace(old, json.dumps(str(ipc_dir / name), ensure_ascii=False))


# 목적: 캐시의 SITL 컴파일 인자를 복사 소스·별도 오브젝트·출력 디렉터리에 맞춰 재작성한다.
# 입출력: compile_commands 항목과 source/obj/output 경로 → 실행 인자 목록(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. SITL 정의를 요구하고 의존성 출력 옵션을 제거하며 이 함수는 컴파일하지 않는다.
def rewrite_compile(entry, source, obj, output):
    args = list(entry.get("arguments") or shlex.split(entry["command"]))
    if "-DCONFIG_ARCH_BOARD_PX4_SITL" not in args:
        raise ValueError("refusing non-SITL compilation command")
    rewritten = [args[0]]
    index = 1
    while index < len(args):
        arg = args[index]
        if arg in ("-o", "-MF", "-MT", "-MQ"):
            index += 2
            continue
        if arg in ("-MD", "-MMD", "-MP"):
            index += 1
            continue
        if arg == entry["file"]:
            rewritten.append(str(source))
        elif arg.startswith("-DPX4_BINARY_DIR="):
            rewritten.append('-DPX4_BINARY_DIR=' + json.dumps(str(output)))
        else:
            rewritten.append(arg)
        index += 1
    # Quoted local includes in copied .cpp files must still find their siblings.
    rewritten.extend(["-I" + str(Path(entry["file"]).parent), "-o", str(obj)])
    if str(source) not in rewritten:
        raise ValueError("compile command did not name the expected source")
    return rewritten


# 목적: ninja 명령 출력에서 bin/px4를 만드는 단일 링크 명령의 인자를 추출한다.
# 입출력: 명령 출력 문자열 → 링크 인자 목록, 일치 개수·금지 토큰 오류 시 ValueError(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 고정 캐시 형식의 && 구분을 전제로 하며 일반 셸 파서나 명령 실행기가 아니다.
def extract_link_command(text):
    matches = []
    for line in text.splitlines():
        if "-o bin/px4" not in line:
            continue
        for segment in line.split("&&"):
            args = shlex.split(segment)
            if "-o" in args and args[args.index("-o") + 1] == "bin/px4":
                matches.append(args)
    if len(matches) != 1:
        raise ValueError("expected one cached bin/px4 link command")
    if any(arg in (";", "|", ">", "<") for arg in matches[0]):
        raise ValueError("shell operators in cached link command")
    return matches[0]


# 목적: 링크 입력 중 교체 오브젝트·아카이브와 최종 ELF·맵 출력 경로를 재지정한다.
# 입출력: 링크 인자·교체 dict·출력/캐시 Path → 새 인자 목록(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 나머지 상대 .o/.a는 캐시 기준 절대 경로로 바꾸며 링크 자체는 수행하지 않는다.
def rewrite_link(args, replacements, output, base):
    rewritten = []
    index = 0
    while index < len(args):
        arg = args[index]
        if arg == "-o":
            rewritten.extend(["-o", str(output / "bin/px4")])
            index += 2
            continue
        if arg in replacements:
            rewritten.append(str(replacements[arg]))
        elif arg.endswith((".o", ".a")) and not Path(arg).is_absolute():
            rewritten.append(str(base / arg))
        else:
            rewritten.append(arg)
        index += 1
    rewritten.append("-Wl,-Map," + str(output / "link.map"))
    return rewritten


# 목적: 캐시의 etc·명령 별칭을 복사하고 overlay의 모듈 실행 링크를 구성한다.
# 입출력: 캐시/출력 Path → 런타임 파일 배치, 정상 완료 시 None(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 모듈 링크 대상·별칭 대응·rcS 존재를 검사할 뿐 PX4를 실행하지 않는다.
def package_runtime(base, output):
    shutil.copytree(base / "etc", output / "etc", symlinks=False)
    shutil.copy2(base / "bin/px4-alias.sh", output / "bin/px4-alias.sh")
    for entry in sorted((base / "bin").iterdir()):
        if entry.name.startswith("px4-") and entry.is_symlink():
            if entry.resolve() != (base / "bin/px4").resolve():
                raise ValueError(f"unexpected module symlink: {entry}")
            (output / "bin" / entry.name).symlink_to("px4")
    for name in re.findall(r"='(px4-[\w-]+) --instance", (output / "bin/px4-alias.sh").read_text()):
        if not (output / "bin" / name).is_symlink():
            raise ValueError(f"alias lacks module symlink: {name}")
    if not (output / "etc/init.d-posix/rcS").is_file():
        raise ValueError("missing normal rcS")


# 목적: 선택 저장소에서 호출자가 지정한 Git 조회 결과를 문자열로 받는다.
# 입출력: 저장소 경로·Git 인자 → 양끝 공백을 제거한 stdout(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. 선택적 잠금을 끄지만 읽기 전용 인자를 선택할 책임은 호출자에게 있다.
def git_read(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args],
                                   env=dict(os.environ, GIT_OPTIONAL_LOCKS="0"), text=True).strip()


# 목적: 빌드 전후 동일성을 비교할 원본·캐시·제품 입력·Git 관리 파일의 경로를 모은다.
# 입출력: PX4/캐시/제품 저장소 Path → 중복 제거·정렬된 Path 목록(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. PX4는 추적된 서브모듈까지 포함하며 OS 수준 쓰기 방지를 설정하지는 않는다.
def protected_paths(px4, base, repo):
    # Recurse through pinned submodules but not build outputs inside the checkout.
    tracked = subprocess.check_output(["git", "-C", str(px4), "ls-files", "--recurse-submodules", "-z"],
                                      env=dict(os.environ, GIT_OPTIONAL_LOCKS="0")).decode().split("\0")
    paths = {px4 / name for name in tracked if name}
    paths.update(p for p in base.rglob("*") if p.is_file() or p.is_symlink())
    paths.add(repo / BRIDGE)
    paths.add(repo / PATCH)
    for root in (px4, repo):
        for item in ("index", "HEAD", "config"):
            paths.add(Path(git_read(root, "rev-parse", "--path-format=absolute", "--git-path", item)))
    return sorted(paths)


# 목적: 고정 PX4·캐시를 읽어 복사 소스와 현재 외부 bridge로 분리된 SITL overlay를 만든다.
# 입출력: 경로 옵션 namespace → overlay·ELF·로그·manifest, 정상 완료 시 0(명령 소요 시간 s).
# 변경/제약: 설명만 추가, 동작 유지. 보호 경로·HEAD를 확인하며 CMake 설정·패키지 설치·PX4/Gazebo 실행은 하지 않는다.
def build(args):
    px4, base, repo, output, ipc = (Path(getattr(args, key)).expanduser().resolve()
                                  for key in ("px4_source", "base_build", "repo", "output", "ipc_dir"))
    for destination in (output, ipc):
        for protected in (px4, base):
            if inside(destination, protected) or inside(protected, destination):
                raise ValueError(f"destination overlaps protected tree: {destination}")
    if inside(output, repo):
        raise ValueError("output must be outside the product repository")
    relocate_ipc('"/tmp/px4-sock-"', "socket", ipc)
    if git_read(px4, "rev-parse", "HEAD") != PINNED_HEAD:
        raise ValueError("PX4 HEAD does not match the audited pinned version")
    for path in (base / "build.ninja", base / "compile_commands.json", base / "bin/px4-alias.sh", repo / BRIDGE, repo / PATCH):
        if not path.is_file():
            raise ValueError(f"missing build prerequisite: {path}")
    for name in ("overlay", "obj", "archives", "bin", "etc", "logs", "tmp", "manifest.json"):
        if (output / name).exists() or (output / name).is_symlink():
            raise ValueError(f"refusing to overwrite previous build: {output / name}")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("overlay", "obj", "archives", "bin", "logs", "tmp"):
        (output / name).mkdir()
    env = dict(os.environ, TMPDIR=str(output / "tmp"), GIT_OPTIONAL_LOCKS="0", CCACHE_DISABLE="1")
    manifest = {"scope": "Linux SITL-only copied overlay; not real-flight firmware", "passed": False,
                "runtime_verified": False, "px4_head": PINNED_HEAD, "base_build": str(base),
                "repo": str(repo), "output": str(output), "px4_source": str(px4),
                "ipc": {"lock_prefix": str(ipc / "px4_lock-"), "socket_prefix": str(ipc / "px4-sock-"),
                        "directory": str(ipc), "files_created_by_builder": []}, "commands": []}
    before = {}

    # 목적: 현재 빌드 기록을 출력 디렉터리의 manifest.json에 저장한다.
    # 입출력: 캡처한 output·manifest → JSON 파일 갱신, 반환 None(기록된 명령 시간 s).
    # 변경/제약: 설명만 추가, 동작 유지. 직접 덮어쓰므로 중단 시 원자적 저장을 보장하지 않는다.
    def save():
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    # 목적: 외부 명령의 합친 stdout/stderr를 로그에 남기고 완료 결과를 manifest에 추가한다.
    # 입출력: 로그 이름·명령 인자·작업 경로 → 로그 문자열, 기록에는 종료 코드와 소요 시간 s가 포함된다.
    # 변경/제약: 설명만 추가, 동작 유지. 명령 제한은 240 s이며 비정상 종료는 예외, 타임아웃은 완료 기록 전에 전파된다.
    def run(label, command, cwd=base):
        log = output / "logs" / (label + ".log")
        started = time.monotonic()
        with log.open("w") as stream:
            stream.write("cwd: " + str(cwd) + "\n$ " + shlex.join([str(x) for x in command]) + "\n")
            stream.flush()
            result = subprocess.run(command, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                    timeout=240)
        record = {"label": label, "argv": [str(x) for x in command], "cwd": str(cwd),
                  "returncode": result.returncode, "seconds": time.monotonic() - started, "log": str(log)}
        manifest["commands"].append(record)
        save()
        print(f"{label}: exit {result.returncode} ({record['seconds']:.3f}s)", flush=True)
        if result.returncode:
            raise RuntimeError(f"{label} failed; see {log}")
        return log.read_text()

    try:
        before = {str(path): identity(path) for path in protected_paths(px4, base, repo)}
        (output / "protected-before.json").write_text(json.dumps(before, indent=2) + "\n")
        manifest["git_before"] = {str(root): {"head": git_read(root, "rev-parse", "HEAD"),
                                            "branch": git_read(root, "rev-parse", "--abbrev-ref", "HEAD")}
                                  for root in (px4, repo)}
        entries = json.loads((base / "compile_commands.json").read_text())
        wanted = [(px4 / MAIN, "main", None), (px4 / SOCKET, "socket", "libpx4_daemon.a"),
                  (px4 / RECEIVER, "receiver", "libmodules__mavlink.a"),
                  (px4 / MESSAGES, "messages", "libmodules__mavlink.a"),
                  (repo / BRIDGE, "bridge", "libmodules__arion_motor_bridge.a")]
        selected = []
        for original, label, archive in wanted:
            matches = [entry for entry in entries if Path(entry["file"]).resolve() == original]
            if len(matches) != 1 or Path(matches[0]["directory"]).resolve() != base:
                raise ValueError(f"cached compile entry/path does not match: {original}")
            entry = matches[0]
            if original == repo / BRIDGE:
                copied = output / "overlay/arion" / BRIDGE
            else:
                copied = output / "overlay" / original.relative_to(px4)
            copied.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, copied)
            if label in ("main", "socket"):
                copied.write_text(relocate_ipc(copied.read_text(), "lock" if label == "main" else "socket", ipc))
            selected.append((entry, copied, label, archive))
        stream = output / "overlay" / STREAM
        stream.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(px4 / STREAM, stream)
        # Validate patch scope before allowing patch to open paths in our copy.
        patch_text = (repo / PATCH).read_text()
        destinations = re.findall(r"^\+\+\+ b/(.+)$", patch_text, re.M)
        sources = re.findall(r"^--- a/(.+)$", patch_text, re.M)
        if set(destinations) != {RECEIVER, STREAM} or sources != destinations or len(destinations) != 2:
            raise ValueError("patch has unexpected source/destination scope")
        run("battery-patch", ["patch", "--batch", "--fuzz=0", "-p1", "-i", str(repo / PATCH)], output / "overlay")
        ninja_log = run("cached-link-command", ["ninja", "-t", "commands", "bin/px4"])
        link = extract_link_command(ninja_log)
        replacements, archives = {}, {}
        manifest["recompiled"] = []
        for entry, copied, label, archive_name in selected:
            obj = output / "obj" / label / Path(entry["output"]).name
            obj.parent.mkdir()
            command = rewrite_compile(entry, copied, obj, output)
            run("compile-" + label, command)
            replacements[entry["output"]] = str(obj)
            manifest["recompiled"].append({"original": entry["file"], "copy": str(copied),
                                             "source_sha256": digest(copied), "object": str(obj),
                                             "object_sha256": digest(obj), "archive": archive_name})
            if archive_name:
                candidates = {arg for arg in link if Path(arg).name == archive_name}
                if len(candidates) != 1:
                    raise ValueError(f"archive absent or ambiguous: {archive_name}")
                archives.setdefault(candidates.pop(), []).append(obj)
        for relative, objects in archives.items():
            source_archive = base / relative
            copied_archive = output / "archives" / source_archive.name
            members = subprocess.check_output(["ar", "t", str(source_archive)], text=True).splitlines()
            if any(members.count(obj.name) != 1 for obj in objects):
                raise ValueError(f"replacement archive members not unique in {source_archive}")
            shutil.copy2(source_archive, copied_archive)
            run("archive-" + source_archive.stem, ["ar", "rcsD", str(copied_archive), *map(str, objects)])
            after_members = subprocess.check_output(["ar", "t", str(copied_archive)], text=True).splitlines()
            if members != after_members:
                raise ValueError("archive member order/count changed")
            replacements[relative] = str(copied_archive)
        (output / "link-replacements.json").write_text(json.dumps(replacements, indent=2) + "\n")
        run("link", rewrite_link(link, replacements, output, base))
        package_runtime(base, output)
        binary = output / "bin/px4"
        with binary.open("rb") as stream:
            if stream.read(4) != b"\x7fELF":
                raise ValueError("link output is not an ELF executable")
        binary_bytes = binary.read_bytes()
        for prefix in (b"/tmp/px4_lock", b"/tmp/px4-sock-"):
            if prefix in binary_bytes:
                raise ValueError(f"old IPC prefix survived link: {prefix!r}")
        for prefix in manifest["ipc"]["lock_prefix"], manifest["ipc"]["socket_prefix"]:
            # lock adds its hyphen at runtime; socket includes it in the literal.
            literal = prefix[:-1] if "px4_lock" in prefix else prefix
            if os.fsencode(literal) not in binary_bytes:
                raise ValueError("new IPC prefix missing from ELF")
        run("elf-header", ["readelf", "-h", str(binary)])
        run("dynamic-dependencies", ["ldd", str(binary)])
        if "not found" in (output / "logs/dynamic-dependencies.log").read_text():
            raise ValueError("unresolved ELF dynamic dependency")
        ipc.mkdir(parents=True, exist_ok=True)
        manifest["binary"] = {"path": str(binary), **identity(binary)}
        manifest["packaging"] = {"alias_sha256": digest(output / "bin/px4-alias.sh"),
                                  "module_symlink_count": sum(p.is_symlink() for p in (output / "bin").iterdir()),
                                  "rcS_sha256": digest(output / "etc/init.d-posix/rcS")}
        manifest["artifacts"] = {str(p.relative_to(output)): identity(p)
                                 for name in ("overlay", "obj", "archives", "bin", "etc")
                                 for p in sorted((output / name).rglob("*")) if p.is_file() or p.is_symlink()}
        manifest["passed"] = True
    except BaseException as error:
        manifest["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if before:
            after = {name: identity(Path(name)) for name in before}
            changed = [name for name in before if before[name] != after[name]]
            manifest["protected"] = {"entry_count": len(before), "changed": changed,
                                       "unchanged": not changed,
                                       "before_manifest": str(output / "protected-before.json")}
            if changed:
                manifest["passed"] = False
        save()
    if not manifest["passed"]:
        raise RuntimeError("protected inputs changed during build")
    print(str(output / "bin/px4"))
    return 0


# 목적: overlay 빌더의 필수 경로 CLI 옵션을 받아 build를 호출한다.
# 입출력: 프로세스 인자 → build 반환 코드 또는 처리한 오류의 stderr·종료 코드 1(물리 단위 없음).
# 변경/제약: 설명만 추가, 동작 유지. argparse 종료와 명시한 예외 밖의 오류는 이 처리 경로에 포함되지 않는다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("px4-source", "base-build", "output", "ipc-dir", "repo"):
        parser.add_argument("--" + flag, required=True, type=Path)
    args = parser.parse_args()
    try:
        return build(args)
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"overlay build failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
