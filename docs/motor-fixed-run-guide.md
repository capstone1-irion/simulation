# 모터 고정값 연동 실행 가이드 — Ubuntu

## 1. 프로그램 역할

| 프로그램/파일 | 역할 |
| --- | --- |
| PX4 SITL | PC에서 실행되는 비행제어기. 모터 구동 명령과 MAVLink 출력 담당 |
| Gazebo Harmonic | x500의 기체 물리·센서 시뮬레이션 |
| `ArionFixedMotorPlugin` | 모터 4개의 **고정 시험값**만 별도 Gazebo Transport 토픽에 발행 |
| `arion_motor_bridge` | 그 상태를 PX4 `esc_status`로 연결하는 외부 모듈 |
| 최소 PX4 패치 | 기본 ESC 상태 발행만 전환. 기본값은 기존 동작 유지 |
| pymavlink + 생성 디코더 | 해당 PX4 버전의 MAVLink 패킷을 Python에서 해석 |
| `run_motor_fixed_e2e.py` | 독립적인 headless Gazebo/PX4를 실행하고, Disarmed 상태에서 실제 UDP 결과를 비교·저장 |

이 문서는 기존 PX4/Gazebo 설치가 끝난 Ubuntu 환경에서 실행하는 방법이다. 별도 Hermes나 ROS 2가 필요 없다. 설치가 없는 PC는 팀의 PX4 설치 가이드를 먼저 따른다. 아래 명령은 새 프로그램 설치나 원격 업로드를 하지 않는다.

예시 작업 폴더는 `~/px4-workspace/simulation`이다. 이 팀 저장소 안에 `PX4-Autopilot/`과 `.venv/`가 준비되어 있다고 가정한다. 다른 설치 위치라면 `PX4_DIR`과 Python 경로를 바꾼다. 이 예시 때문에 기존 환경을 옮길 필요는 없다.

## 2. 경로와 가상환경

**Ubuntu 터미널**에서:

```bash
cd ~/px4-workspace/simulation
source .venv/bin/activate
export SIM_DIR="$PWD"
export PX4_DIR="$SIM_DIR/PX4-Autopilot"
export RUN_DIR="$SIM_DIR/experiments/motor-fixed"
mkdir -p "$RUN_DIR"
```

- `.venv`는 기존 pymavlink와 PX4 Python 빌드 의존성을 사용한다.
- `build/`에는 컴파일 결과, `experiments/`에는 실행 로그가 생긴다. 소스와 분리되어 Git 업로드 대상이 아니다.

## 3. 상태 발행 전환 패치

선택한 PX4는 **v1.17.0 / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`**이다. 다른 버전이나 수정된 원본에 강제로 적용하지 않는다.

```bash
git -C "$PX4_DIR" apply --check \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch"
git -C "$PX4_DIR" apply \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch"
```

`--check`는 적용 가능 여부만 검사한다. 다음 명령이 원본의 `GZMixingInterfaceESC.cpp`와 `module.yaml`을 바꾼다. 이미 적용했으면 다시 적용하지 않는다. 기존 변경 때문에 검사에 실패하면 중단하고 차이를 확인한다.

## 4. Gazebo Plugin 빌드

```bash
cmake -S "$SIM_DIR" -B "$SIM_DIR/build/motor-plugin" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$SIM_DIR/build/motor-plugin" -j 2
ctest --test-dir "$SIM_DIR/build/motor-plugin" --output-on-failure
```

`libArionFixedMotorPlugin.so`와 입력 decoder 테스트가 만들어진다. 시스템 Gazebo 파일을 설치·교체하지 않는다.

Gazebo에서 실제 발행되는 값과 시뮬레이션 시각 기준 주기까지 확인하려면:

```bash
ARION_TEST_ROOT="$RUN_DIR" \
ARION_PLUGIN_BUILD="$SIM_DIR/build/motor-plugin" \
python -m unittest discover -s tests -p test_gazebo_motor.py -v
```

이 시험은 별도 Gazebo 서버만 사용하며 PX4나 비행 명령을 실행하지 않는다. probe 값이 맞아도 서버의 비정상 종료·강제 종료·남은 소유 그룹이 있으면 실패한다. launcher가 먼저 끝나도 같은 소유 그룹의 자식 작업을 정리하며, 결과는 각 실행의 `cleanup.json`과 `cleanup-errors.json`에 남긴다.

## 5. 외부 Bridge를 포함한 PX4 빌드

```bash
cmake -S "$PX4_DIR" -B "$SIM_DIR/build/px4-arion" -G Ninja \
  -DCONFIG=px4_sitl_default \
  -DEXTERNAL_MODULES_LOCATION="$SIM_DIR/px4" \
  -DPYTHON_EXECUTABLE="$SIM_DIR/.venv/bin/python" \
  -DPython3_EXECUTABLE="$SIM_DIR/.venv/bin/python"
cmake --build "$SIM_DIR/build/px4-arion" --target px4 -j 2
```

`EXTERNAL_MODULES_LOCATION`은 `px4/src/CMakeLists.txt`의 모듈 목록을 PX4 빌드에 추가한다. Bridge 소스를 PX4 원본 폴더에 복사하지 않는다. 기존 기본 PX4 빌드 결과와 로그도 덮어쓰지 않는 별도 build 디렉터리다.

## 6. PX4 정의와 맞는 Python 디코더 생성

```bash
mavgen.py --lang=Python3 --wire-protocol=2.0 \
  --output="$RUN_DIR/px4_messages.py" \
  "$PX4_DIR/src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml"
```

이는 새로운 센서를 만드는 명령이 아니다. 설치한 PX4의 메시지 정의를 읽어 Python 해석 파일을 만든다. 디코더는 실행 가능한 Python 코드이므로 본인이 생성한 신뢰 가능한 파일만 사용한다. 설치된 기본 `common` 디코더 파일을 덮어쓰지 않는다.

## 7. 전체 고정값 전달 시험

```bash
python tests/run_motor_fixed_e2e.py \
  --px4-source "$PX4_DIR" \
  --px4-binary "$SIM_DIR/build/px4-arion/bin/px4" \
  --plugin-build "$SIM_DIR/build/motor-plugin" \
  --decoder "$RUN_DIR/px4_messages.py" \
  --artifacts "$RUN_DIR"
```

실행 도구의 동작:

| 단계 | 확인 내용 |
| --- | --- |
| 격리 실행 | 별도 Gazebo partition, PX4 instance, 새로운 rootfs, OS 배정 loopback UDP 수신 포트 사용 |
| 기본 상태 | command-derived ESC 전압·전류가 기본 값 0으로 전달되는지 확인 |
| 사용자 상태 | 기본 상태 발행을 끄고 Bridge를 시작한 후 각 모터의 고정 RPM/V/A가 일치하는지 확인 |
| PX4 내부 | uORB `esc_status`의 연결 수와 RPM을 읽음 |
| 입력 부재 | 존재하지 않는 상태 토픽으로 Bridge를 재시작하고 ESC 값이 새 시각으로 재발행되거나 기본 명령 값으로 대체되지 않는지 확인 |
| 복귀 | Bridge 정지 후 기본 ESC 상태 발행 재개 확인 |
| 종료 | 본 시험이 시작한 PX4/Gazebo 프로세스만 종료 |

Python UDP 소켓은 **수신만** 한다. PX4 console에서는 소유한 시뮬레이터의 텔레메트리 설정·Bridge 시작/정지만 실행한다. Heartbeat 전송, MAVLink 제어 메시지, Arm/이륙/착륙 명령은 보내지 않는다. 현재 켜져 있는 QGC나 다른 시뮬레이터의 종료 명령도 보내지 않는다.

각 실행은 덮어쓰지 않는 새 `px4-motor-fixed-*` 폴더를 만든다.

| 파일 | 의미 |
| --- | --- |
| `report.json` | 실제 비교 결과, sender ID, phase별 메시지 개수·원본 시각 간격, uORB/Bridge 출력 |
| `mavlink_raw.jsonl` | 실제 수신·디코딩한 필드와 송신자, 수신 시각. wire-byte pcap은 아님 |
| `px4.log`, `gazebo.log` | 이번 실행의 console/server 로그 |
| `rootfs/` | 이번 실행만의 PX4 파라미터·실행 파일 위치 |

`report.json`의 `passed=true`와 `cleanup`의 프로세스 종료 상태를 확인한다. 성공 판정은 값 비교와 종료 검사가 모두 끝난 뒤 저장하며, 종료 예외·비정상 종료·강제 종료·남은 프로세스 그룹이 있으면 실패다. `E2E_EVIDENCE`는 저장 위치이며 그 문구만으로 성공을 의미하지 않는다. 고정값 표와 해석 제한은 `docs/motor-fixed-contract.md`를 따른다.

정리 도중 Ctrl+C 등의 중단도 오류로 기록하고 같은 자원 정리를 제한된 횟수로 재시도한 뒤 다른 소유 그룹·수신 소켓·PTY descriptor 정리를 계속한다. 같은 프로세스 정리가 반복 중단되면 그 시험의 소유 그룹만 강제 종료하며, 이 경우에도 성공으로 판정하지 않는다. PX4 생성에 실패했을 때도 PTY master/slave를 모두 정리한다. `errors`에는 원래 오류와 정리 중 오류가 함께 남고, `descriptor_cleanup`에는 마지막 descriptor 정리 결과가 남는다.

안전·값 비교 검사가 제거되지 않도록 **일반 Python 실행만 허용**한다. `python -O`, `python -OO`, `PYTHONOPTIMIZE=1/2` 실행은 시뮬레이터 시작 전에 거부한다.

### 수정한 모터 경로의 집중 회귀시험

모터 관련 단위 시험만 명시적으로 선택한다. 전체 `test_*.py` 검색은 배터리·overlay·서비스 시험까지 선택하므로 사용하지 않는다.

```bash
ARION_TEST_ARTIFACT_ROOT="$RUN_DIR" \
ARION_TEST_ROOT="$RUN_DIR" \
PYTHONPATH="$SIM_DIR/tests${PYTHONPATH:+:$PYTHONPATH}" \
python -m unittest -v \
  test_bridge_lifecycle test_bridge_worker test_bridge_shutdown \
  test_e2e_safety.CleanupTest test_e2e_safety.OptimizationTest \
  test_cleanup_regressions
```

실제 Disarmed PX4/Gazebo 실행이 필요한 변경일 때만 다음 시험을 별도로 실행한다. 단위 시험 명령은 이 실통합 시험을 자동으로 시작하지 않는다.

```bash
ARION_TEST_ARTIFACT_ROOT="$RUN_DIR" \
ARION_TEST_ROOT="$RUN_DIR" \
ARION_PLUGIN_BUILD="$SIM_DIR/build/motor-plugin" \
ARION_PX4_BINARY="$SIM_DIR/build/px4-arion/bin/px4" \
ARION_DECODER="$RUN_DIR/px4_messages.py" \
ARION_RUN_E2E_REGRESSION=1 \
PYTHONPATH="$SIM_DIR/tests${PYTHONPATH:+:$PYTHONPATH}" \
python -m unittest -v \
  test_gazebo_motor test_px4_console test_e2e_safety.RealShutdownRegressionTest
```

배터리/overlay는 `docs/sitl-overlay-build.md`의 별도 명령과 필수 `ARION_OVERLAY_TEST_ROOT`를 따른다. CMU/운영 서비스는 각각 `docs/cmu-evtol-normal-profile-guide.md`, `docs/cmu-service-guide.md`를 따른다.

두 번째 명령에는 실제 Disarmed 시뮬레이터에서 값 비교가 끝난 뒤 종료 쓰기·프로세스 정리 중 실패를 주입하는 회귀시험도 포함된다. 그 실행의 `report.json`은 의도대로 `passed=false`가 되어야 해당 회귀시험이 통과한다. 콜백·워커 수명 시험은 AddressSanitizer/UndefinedBehaviorSanitizer를 사용한 별도 C++ 단위 시험이다. 실제 Bridge C++·pthread·pinned ModuleBase/POSIX task 구현으로 지연 진입, 초기화/생성 실패, native join 실패·동시 stop·세대 분리를 검사한다. 초기화된 시뮬레이션 시계를 멈추는 단위 seam과 실제 native wrapper 종료를 지연하는 시험도 있다. 일반 `usleep`이나 시험 쪽 join을 제품 종료의 근거로 삼지 않으며, 실패 뒤 fixture 안전 정리는 판정 이후 별도로 기록한다. PX4 서비스·Transport는 fixture이므로 실제 시뮬레이터 pause나 텔레메트리의 증거가 아니다. 로컬 Python 프로세스로 만드는 종료·descriptor 단위 fixture도 실제 Gazebo 값 시험과 구분한다. 정상 전달 성공 여부는 위 전체 고정값 전달 시험을 따로 실행하여 확인한다. Bridge 단위 부산물은 `ARION_TEST_ARTIFACT_ROOT`를 우선 사용하며, 다른 기존 시험까지 같은 폴더에 저장하려면 위처럼 `ARION_TEST_ROOT`도 함께 지정한다.

## 8. 직접 PX4 console에서 전환할 때

위 자동 시험은 이 전환을 내부에서 수행한다. 수동 시뮬레이터를 이미 시작한 경우에는 **PX4의 `pxh>` console**에 입력한다. Ubuntu shell 명령이 아니다.

```text
param set SIM_GZ_ESC_ST 0
arion_motor_bridge start -t /arion/custom_x500_0/motor_state
arion_motor_bridge status
listener esc_status -n 1
```

모델 instance가 0이 아니면 실제 모델 이름으로 토픽을 바꾼다. Bridge에 데이터가 없어도 기존 모터 명령을 상태로 자동 대체하지 않는다. 수신 프레임과 `esc_status` timestamp는 기존 시뮬레이션 시간 기준이며, 입력 plugin의 20 Hz 계약을 wall-clock 발행으로 바꾸지 않는다. worker는 새 프레임 또는 종료 요청의 predicate/condition-variable로 깨어나므로, 입력 부재·초기화된 시뮬레이션 시계 정지에서도 종료 신호를 처리할 수 있다.

`start` 성공은 동기 초기화·구독 설정과 native task 생성 성공을 뜻한다. 첫 frame이나 worker 진입 완료를 뜻하지 않으며, 즉시 `stop`도 미리 생성한 owner에 종료 요청을 남긴다. `stop`은 **monotonic wall-clock의 단일 약 5초 예산** 안에서 command lock 조정·협력 종료·PX4 native wrapper의 실제 종료와 pthread join·등록 정리를 기다린다. worker와 stop은 이를 위해 `px4_usleep`이나 시뮬레이션 시계 진행을 기다리지 않는다. 현재 구현은 monotonic native timed-lock/join API가 제공되는 Linux POSIX 환경을 대상으로 하며, 다른 플랫폼에서는 worker 생성 전에 시작을 거부한다.

성공한 `stop`은 C++ owner뿐 아니라 native pthread 회수까지 끝났음을 뜻한다. timeout·native join 오류이면 실패를 반환하고, 실행 중인 worker가 접근할 owner 또는 아직 회수가 확인되지 않은 세대의 native handle/등록을 보존하여 새 시작을 거부한다. 강제 task 삭제·cancel은 하지 않는다. 지연 worker가 나중에 끝나도 **`stop`을 다시 실행하여 native 회수와 등록 정리를 성공시킨 뒤에만** 재시작한다. owner가 없어져도 native 정리가 남았으면 `status`는 `stopping; native cleanup pending`을 출력하며 `not running`으로 표현하지 않는다. 회수된 stack mapping은 libc cache에 남아 재사용될 수 있으므로 mapping이 남았다는 사실만으로 자원 누수를 판정하지 않는다. 정리가 계속 실패하면 소유한 시뮬레이터를 종료하고 원인을 확인한다. 다음 명령으로 복귀한다.

```text
arion_motor_bridge stop
arion_motor_bridge status
```

`not running`을 확인했을 때만:

```text
param set SIM_GZ_ESC_ST 1
```

완전히 원본 소스로 되돌릴 때는 관련 시뮬레이터를 먼저 정지하고:

```bash
git -C "$PX4_DIR" apply --reverse --check \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch"
git -C "$PX4_DIR" apply --reverse \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch"
```

그 뒤 원본 PX4를 다시 빌드한다. 사용자 변경이 생겨 reverse 검사에 실패하면 강제 reset하지 않는다.

## 현재 검증과 후속 범위

- 검증 완료 범위는 별도 실행 기록에 명시한다. 빌드 성공만으로 실제 전달이나 비행 성공이라고 판단하지 않는다.
- 현재 fixture의 온도·fault·power 등은 미구현 값이며 AI 입력으로 사용하지 않는다.
- 배터리 연동, 부하 의존 모터/배터리 상태, 정상/이상 시나리오, 커스텀 모델 비행, QGC UI 확인은 별도 체크리스트 항목이다.
