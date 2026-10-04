# 고정 배터리 Plugin·외부 송신부 — Ubuntu 빌드 안내

> **후속 완료 상태(2026-10-04 KST):** 이 문서의 아래 '미실행/보류' 설명은 초기 코드·빌드 단계의 기록이다. 이후 사용자 요청으로 원본을 보존한 별도 SITL overlay에서 고정 배터리 왕복과 모터–배터리 연동 비행까지 실제 완료했다. 최신 실행·설정은 `coupled-flight-run-guide.md`, 데이터 계약은 `coupled-flight-data-contract.md`를 따른다. 원본 PX4에 배터리 패치를 직접 적용하라는 뜻은 아니다.

## 1. 프로그램 역할과 현재 범위

| 구성요소 | 역할 |
| --- | --- |
| `ArionFixedBatteryPlugin` | Gazebo 모델에서 고정 시험값을 생성해 별도 배터리 토픽에 발행 |
| `arion_battery_sender` | 해당 토픽의 새 프레임을 MAVLink `BATTERY_STATUS`로 변환해 지정한 로컬 PX4 입력 포트로 전송 |
| PX4 | 외부 입력을 uORB `battery_status`에 반영하고 별도 MAVLink 링크로 재송신할 대상. 이번 작업에서는 수정·실행하지 않음 |
| Python 수집기 | 향후 PX4가 재송신한 패킷을 수신·대조할 대상. 송신부 stdout이나 Gazebo 직접 수신으로 대체하지 않음 |

**2026-10-04 현재: Plugin·송신부 구현 및 빌드까지 완료. 실제 Gazebo 발행, UDP 송신, PX4 내부 반영과 재송신은 미실행이다.** 실행 보류 결정에 따라 기존 PX4 소스·실행파일·모터 코드·`custom_x500` 모델을 변경하지 않았다. 공유 `CMakeLists.txt`에는 기본 OFF인 배터리 빌드 옵션만 추가했다.

이 문서는 기존 Ubuntu/Gazebo 개발환경을 사용하는 팀원을 위한 안내다. 추가 설치·PX4 재빌드·원격 업로드를 하지 않으며 별도 에이전트 프로그램이 필요 없다.

## 2. 필요한 환경과 빌드

- C++17 컴파일러, CMake, Ninja.
- Gazebo Harmonic: `gz-sim8`, `gz-transport13`, `gz-plugin2` 개발 라이브러리.
- 팀의 pinned PX4 **v1.17.0 / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`** 빌드에서 생성된 MAVLink C 헤더. `common/mavlink.h`가 들어 있는 디렉터리를 지정한다.
- 아래 `~/px4-workspace/simulation`은 예시다. 실제 설치 위치를 옮기지 말고 경로 변수만 맞춘다.

**Ubuntu 터미널 — 빌드만 수행:**

```bash
cd ~/px4-workspace/simulation
export SIM_DIR="$PWD"
export PX4_DIR="$SIM_DIR/PX4-Autopilot"
export MAVLINK_INCLUDE_DIR="$PX4_DIR/build/px4_sitl_default/mavlink"
export BATTERY_BUILD="$SIM_DIR/build/battery-fixed"
mkdir -p "$BATTERY_BUILD/tmp"

TMPDIR="$BATTERY_BUILD/tmp" cmake -S "$SIM_DIR" -B "$BATTERY_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DARION_BUILD_BATTERY_COMPONENTS=ON \
  -DARION_MAVLINK_INCLUDE_DIR="$MAVLINK_INCLUDE_DIR"

TMPDIR="$BATTERY_BUILD/tmp" cmake --build "$BATTERY_BUILD" \
  --target ArionFixedBatteryPlugin arion_battery_sender battery_contract_compile_test \
  -j 2
```

다른 별도 PX4 빌드를 사용했다면 `MAVLINK_INCLUDE_DIR`을 그 빌드의 `mavlink/`로 바꾼다. CMake가 헤더 누락을 알리면 거기서 중단한다. 이 프로젝트가 자동으로 헤더를 다운로드하거나 PX4를 재빌드하지는 않는다.

| 빌드 대상 | 결과 |
| --- | --- |
| `ArionFixedBatteryPlugin` | `libArionFixedBatteryPlugin.so` |
| `arion_battery_sender` | 외부 송신 실행파일 |
| `battery_contract_compile_test` | `static_assert`로 값 변환·잘못된 입력·발행 시계를 검사하는 object target. 시험 프로그램을 실행하지 않음 |

세 대상만 지정하므로 기존 모터 시험이나 PX4/Gazebo를 기동하지 않는다. 컴파일러가 계약 검사를 통과해도 실제 전송 성공을 뜻하지 않는다.

## 3. 고정값 계약

소스: `include/arion/BatteryFrame.hpp`, `include/arion/BatteryPublishClock.hpp`.

- 토픽: `/arion/<Gazebo의 실제 모델 이름>/battery_state`.
- 메시지: `gz.msgs.Float_V`, 정확히 9개 값.
- header의 `schema` 항목: `arion.battery_state.v1`, 정확히 한 번.
- header stamp: 해당 프레임의 Gazebo 시뮬레이션 시간. 수신 wall-clock이나 UNIX 시각이 아니다.
- 발행 스케줄: **시뮬레이션 시간 기준 200 ms 간격(5 Hz 목표)**. pause 중 발행하지 않고 시간이 뒤로 돌아가면 새 시점에서 다시 시작한다. 실제 관측 주파수는 아직 측정하지 않았다.

| 배열 순서(0부터) | 의미 | 고정 시험값 | 송신부의 MAVLink 값 |
| --- | --- | --- | --- |
| 0 | 배터리 ID | 0 | `id=0` |
| 1~4 | 셀 1~4 전압 | 4.01 / 4.02 / 4.03 / 4.04 V | 4010 / 4020 / 4030 / 4040 mV |
| 5 | 방전 전류 | 7.25 A | `current_battery=725` cA |
| 6 | 잔량 비율 | 0.73 | `battery_remaining=73` % |
| 7 | 온도 | 28.5 °C | `temperature=2850` cdegC |
| 8 | 소비 전하량 | 123 mAh | `current_consumed=123` mAh |

셀 전압 합계는 **16.10 V**다. 각 값은 식별·단위 대조용으로 정한 fixture이며 공개 데이터의 팩 환산값이나 X500 실측값이 아니다. 전류 적분, SOC 변화, 용량·내부저항·열 모델을 구현하지 않았고 모터 부하에도 반응하지 않는다. LiPo 타입 표시는 fixture의 선언값이지 실제 배터리 화학종을 확인한 결과가 아니다.

송신부는 단일 배터리 ID 0·셀 4개·알려진 유한값·방전 전류만 받는다. NaN/Inf, 잘못된 길이/ID/schema/시각, 표현 범위 밖의 값은 거부한다. 단위 변환은 가장 가까운 정수 wire 단위로 반올림한다. 현재 계약은 미상 센서값이나 충전/회생 전류 입력을 지원하지 않는다.

사용하지 않는 기본 전압 슬롯은 `UINT16_MAX`, 확장 전압은 0, 소비 에너지는 -1, 잔여 시간은 0, charge-state는 UNDEFINED, mode는 UNKNOWN, fault bitmask는 0으로 넣는다. 이는 미지원 필드를 구분하기 위한 것이며 **fault 0을 정상 진단 완료로 해석하면 안 된다**. 필드 단위·미상값의 근거는 pinned `common.xml`의 `BATTERY_STATUS` 정의와 공식 MAVLink 메시지 문서다.

## 4. 송신부 동작과 실행 보류

소스: `tools/battery_sender.cpp`.

- 대상 주소는 **127.0.0.1만** 사용한다. 포트와 `system-id`는 반드시 명시한다.
- `--send`와 비어 있지 않은 `GZ_PARTITION`이 없으면 시작을 거부한다.
- source component는 `MAV_COMP_ID_BATTERY`(180)다. PX4 autopilot component로 가장하지 않는다.
- 새 유효 프레임만 전달한다. 같은 시각의 중복은 버리고 입력이 빨라지면 최신 프레임 하나만 유지한다. 입력 시각 역행은 새 시뮬레이션 구간으로 집계한다.
- 수신 후 1초보다 오래 대기한 프레임은 보내지 않는다. 유효한 새 입력이 5 wall-clock초 없으면 오류로 종료한다. 마지막 값을 무기한 재전송하지 않는다.
- 송신 루프는 기본 20 wall-clock초, `--duration`으로 1~120초를 지정한다. 이는 기동·Gazebo 라이브러리 종료까지 포함한 전체 프로세스의 실측 시간 보장이 아니다.
- `BATTERY_STATUS` 이외의 Heartbeat·제어·Arm·비행·파라미터 변경 메시지를 보내지 않는다. PX4와 Gazebo를 자동 기동하지 않는다.
- stdout의 JSONL은 로컬 UDP send 기록이며 **PX4 수신 확인/ACK/재송신 수집 파일이 아니다**. 원본 시뮬레이션 시각은 이 기록에 남지만 `BATTERY_STATUS` payload에는 별도 sample timestamp가 없다.
- callback은 shared mailbox만 참조하고 실제 socket 송신은 메인 루프에서 한다. 종료 시 자원을 회수하도록 구현했으나 실제 구독·종료 수명 시험은 아직 하지 않았다.

CLI 형식은 다음과 같다. **현재 실행 승인을 의미하지 않는 사용 형식**이며 값은 실제 대상 확인 후 지정해야 한다.

```text
arion_battery_sender --send \
  --topic /arion/MODEL/battery_state \
  --px4-port PORT --system-id ID --duration 20
```

별도 `tests/battery_fixture.sdf`는 향후 Gazebo-only 시험용 모델이며 현재 실행하지 않았다. 실제 `custom_x500`에는 배터리 Plugin을 자동 추가하지 않았다. 이후 승인된 모델 복사본의 `<model>` 안에 다음 항목을 넣고 `GZ_SIM_SYSTEM_PLUGIN_PATH`에 배터리 빌드 디렉터리를 추가하는 방식이다.

```xml
<plugin filename="ArionFixedBatteryPlugin" name="arion::FixedBatteryState"/>
```

후속 실행에서는 독립 Gazebo partition과 소유 프로세스 정리를 준비하고, 전체 실행을 외부 `timeout`으로 제한한다. 연결 시도는 120초 넘게 방치하지 않는다. 정확한 PX4 instance·IPC 사용 범위를 먼저 확정한다.

## 5. PX4 연결 전에 남아 있는 문제

현재 보존한 pinned PX4의 소스에는 다음 제약이 있다. 이 작업에서 우회·수정하거나 실제 실행으로 재현하지 않았다.

1. `mavlink_receiver.cpp`의 `handle_message_battery_status`는 수신 온도를 uORB 섭씨로 변환하지 않고 복사한다. `streams/BATTERY_STATUS.hpp`는 유한 온도를 다시 100배한다. **송신부가 잘못된 MAVLink 단위를 보내 보상하지 않는다.**
2. 입력 배터리 ID/source 및 미상값 변환도 별도 처리가 필요하다. 따라서 현 상태에서 위 송신부를 연결해도 올바른 온도·ID·미상값 end-to-end 전달을 보장하지 않는다.
3. 기본 battery simulator 격리는 새 실행 시작 전 `BAT1_SOURCE=1`, `SIM_BAT_ENABLE=0`을 검토해야 한다. `SIM_BAT_DRAIN=0`은 발행 중지 스위치가 아니므로 사용하지 않는다. 실제 파라미터는 이번에 바꾸지 않았다.
4. 외부 입력용 링크와 Python 수집용 링크를 분리하고, 원래 송신부 component 180 패킷/포워딩 패킷을 PX4 autopilot의 재송신과 구별해야 한다. PX4와 같은 system ID라는 사실만으로 PX4 출력임이 증명되지 않는다.

근거 위치(모두 팀 pinned PX4 트리 기준):

- `src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`, `minimal.xml`.
- `src/modules/mavlink/mavlink_receiver.cpp::handle_message_battery_status`.
- `src/modules/mavlink/streams/BATTERY_STATUS.hpp`.
- `src/modules/simulation/battery_simulator/BatterySimulator.cpp`, `battery_simulator_params.c`.
- `src/lib/battery/battery.cpp`, `ROMFS/px4fmu_common/init.d-posix/rcS`.
- 공식 메시지 정의: `https://mavlink.io/en/messages/common.html#BATTERY_STATUS`.
- Gazebo 8 Plugin API: `https://gazebosim.org/api/sim/8/createsystemplugins.html`.

## 6. 완료와 미완료의 구분

**확인한 것:** 새 Plugin 및 송신부 컴파일·링크, 컴파일 시 고정값 변환/입력 거부/시뮬레이션 시계 계약 검사.

**하지 않은 것:** 시험 실행파일 실행, Gazebo/PX4 기동, UDP 송신, Plugin 로딩·토픽 관측, MAVLink 패킷 실제 encode/decode 대조, uORB 반영, PX4 재송신·Python 수집, 비행·부하 연동, 전체 회귀·독립 리뷰, Git 통합.

따라서 B-01/B-02의 **코드 준비**를 기록하되 B-01~B-04의 실제 경로 완료 표시는 보류한다. 다음 실시간 단계에는 PX4 변환 문제 처리 방침과 실행 범위에 대한 별도 결정이 필요하다.
