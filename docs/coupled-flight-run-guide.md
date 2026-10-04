# 모터–배터리 연동 SITL 실행 안내

## 각 프로그램의 역할

| 프로그램/파일 | 역할 |
| --- | --- |
| Gazebo + stock x500 | 기체 물리와 센서·회전자 상태 시뮬레이션 |
| `ArionCoupledPowerPlugin` | 관측 RPM → 모터 전류 근사 → 팩 전류·전압·SOC 계산. 추진력·명령은 변경하지 않음 |
| `arion_motor_bridge` | 상태 토픽을 PX4 `esc_status`로 전달 |
| `arion_battery_sender` | 배터리 토픽을 표준 BATTERY_STATUS로 포장해 별도 PX4 입력 포트에 송신 |
| 별도 PX4 SITL overlay | 외부 배터리 변환/프로젝트 IPC 보정. 원본 PX4 및 기존 빌드 보존 |
| `run_coupled_flight.py` | 소유 시뮬레이터의 console-only 이륙·호버·착륙과 정리. 실제 기체 제어용이 아님 |
| 내부 receive-only collector | PX4의 표준 MAVLink를 수신·JSONL 저장. 비행 명령 송신과 분리 |
| `arion_power_observer` | Plugin 원본 값 대조용 읽기 전용 구독. AI 입력 경로가 아님 |

ROS 2나 Hermes는 실행에 필요하지 않다. QGC GUI를 열지 않는 headless 시험이며, GUI/QGC 연동을 확인하는 절차는 아니다.

## 1. 준비된 환경과 경로

기존 Ubuntu PX4 v1.17.0/Gazebo Harmonic 환경, CMake/Ninja/C++17 및 프로젝트 Python 가상환경을 사용한다. `motor-fixed-run-guide.md`의 **외부 arion_motor_bridge를 포함한 기존 빌드**가 먼저 있어야 한다. 이 안내는 패키지 설치나 새 PX4 초기 구성 안내가 아니다.

아래는 팀 공통 예시 경로다. `BASE`는 실제 기존 PX4 빌드 위치로 맞춘다. 예시는 bash에서 순서대로 실행한다.

```bash
WORK="$HOME/px4-workspace"
SIM="$WORK/simulation"
PX4="$SIM/PX4-Autopilot"
BASE="$SIM/build/px4-arion"
OUT="$WORK/artifacts/coupled-flight"
PY="$SIM/.venv/bin/python"
mkdir -p "$OUT/tmp" "$WORK/ipc"
```

PX4 checkout은 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`이고 기존 빌드의 compile_commands 경로가 유효해야 한다. 원본에 이번 배터리 patch를 직접 적용하지 않는다.

## 2. 상태 Plugin과 송신/관측 도구 빌드

새 build 폴더에 생성한다. 기존 모터 모델·빌드를 덮어쓰지 않는다.

```bash
TMPDIR="$OUT/tmp" cmake -S "$SIM" -B "$OUT/plugins" -G Ninja   -DCMAKE_BUILD_TYPE=RelWithDebInfo   -DARION_BUILD_BATTERY_COMPONENTS=ON   -DARION_BUILD_COUPLED_POWER=ON   -DARION_MAVLINK_INCLUDE_DIR="$BASE/mavlink"
TMPDIR="$OUT/tmp" cmake --build "$OUT/plugins" --target   ArionFixedMotorPlugin ArionFixedBatteryPlugin ArionCoupledPowerPlugin   arion_battery_sender arion_power_observer -j 2
```

## 3. 분리된 PX4 실행파일과 matching decoder 만들기

기존 완료 빌드의 의존성을 읽어 새 실행파일만 링크한다. overlay 출력 폴더는 새 경로여야 한다. 재실행을 위해 매번 다시 빌드할 필요는 없다.

```bash
"$PY" -B "$SIM/tools/build_sitl_overlay.py"   --repo "$SIM" --px4-source "$PX4" --base-build "$BASE"   --output "$OUT/px4" --ipc-dir "$WORK/ipc"
"$SIM/.venv/bin/mavgen.py" --lang=Python3 --wire-protocol=2.0   --output="$OUT/px4_messages.py"   "$PX4/src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml"
```

`$OUT/px4/bin/px4`·aliases·etc를 함께 유지한다. 실행파일에는 IPC 경로가 들어가므로 나중에 다른 `--ipc-dir`로 바꾸려면 새 overlay를 만든다. 세부사항은 `sitl-overlay-build.md`를 따른다.

## 4. 고정값 전달 또는 비행

공통 인자를 준비한다. 수집 포트는 OS가 빈 loopback 포트로 선택한다. 서로 다른 실행에는 고유한 artifacts 하위 폴더와 Gazebo partition/HOME/rootfs가 만들어진다.

```bash
args=(
  --px4-source "$PX4"
  --px4-binary "$OUT/px4/bin/px4"
  --plugin-build "$OUT/plugins"
  --decoder "$OUT/px4_messages.py"
  --artifacts "$OUT/runs"
  --ipc-dir "$WORK/ipc"
  --instance 35
)
# 처음 만든 환경에서 고정 배터리 왕복만 확인할 때(Arm 없음):
"$PY" -B "$SIM/tools/run_coupled_flight.py" "${args[@]}" --mode fixed
# 모터 부하와 배터리가 연동된 기체의 이륙/3m 호버8초/착륙:
"$PY" -B "$SIM/tools/run_coupled_flight.py" "${args[@]}"   --mode flight --fly --hover-seconds 8
```

정상 종료 시 `RUN_EVIDENCE <폴더>`와 `passed: true`가 출력된다. 필요 없이 성공한 고정 시험이나 전체 회귀를 반복하지 않는다. 실패 시 해당 폴더 `report.json`의 첫 오류, `px4.log`·`gazebo.log`를 보고 원인만 수정한다. 사전검사 실패는 강제 Arm으로 우회하지 않는다. 실패 산출물을 지우거나 성공 기록으로 덮어쓰지 않는다.

실행기가 고정하는 중요한 설정:

- stock 배터리 발행을 부팅 전 `SIM_BAT_ENABLE=0`, `BAT1_SOURCE=1`로 격리.
- stock ESC 상태와 custom Bridge를 겹쳐 발행하지 않음. 기존 actuator/flight-control 경로 유지.
- standalone Gazebo는 프로젝트 `config/coupled-headless.config`의 IMU·barometer·GPS·magnetometer 시스템을 사용. 기본 server config만 쓰면 센서가 없을 수 있음.
- 비행 모델 `custom_x500_power`는 `allow_command_fallback=false`: joint 상태가 없으면 상태 발행을 중단하고 모델 명령값으로 대신하지 않음.
- 이 시험은 RC/GCS 없이 **소유 PX4 콘솔**로 비행하므로 `COM_RC_IN_MODE=4`, 부팅 후 `NAV_DLL_ACT=0`. 실기체의 통신 단절 안전설정으로 복사하지 않음.
- collector는 receive-only이며 source system36/component1의 별도 출력만 판정. input/output MAVLink forwarding은 꺼짐.
- 다른 PX4/Gazebo 프로세스를 일괄 종료하지 않으며 소유 PID 그룹과 정확한 IPC만 정리.

## 5. 결과 사용

`mavlink_raw.jsonl`이 PX4 수신 원문이다. `producer.jsonl`/`sender.jsonl`은 경로 대조용이며 AI 입력을 대체하지 않는다. `report.json`에는 각 phase·비행 상태·배터리 부하 반응·정리 결과가 남는다.

2026-10-04 KST 로컬 실제 실행에서 고정 배터리 왕복과 커스텀 비행이 성공했다. 해당 환경의 측정값/근거는 작업폴더 `배터리_모터_연동_및_커스텀_비행_완료_보고.md`에 있다. 위 공통 경로는 팀원용 재현 예시이며 다른 컴퓨터의 설치·비행을 검증했다는 뜻은 아니다.

모델은 4S/5Ah/팩저항0.04Ω/초기SOC95%의 **가정한 정상 reference**다. 전류의 벤치 조건, 고정 온도, 단방향 결합의 한계는 `coupled-power-model.md`, 실제 수신률/미상값/식별·시각 계약은 `coupled-flight-data-contract.md`를 먼저 읽는다. 실기체 검증·이상 주입·AI 학습은 별도 작업이다.
