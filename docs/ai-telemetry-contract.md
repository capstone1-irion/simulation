# AI가 사용할 MAVLink 관측값 — 현재 CMU 서비스 계약

## 1. 지금 완료된 경계

이 문서는 **현재 `simulation` 저장소가 PX4 출력에서 실제로 수집하는 값**을 정의한다. AI 모델의 최종 입력 벡터·리샘플링·정규화·추론 연결까지 구현됐다는 뜻은 아니다. 아래 관측 중 무엇을 모델에 넣을지는 AI 쪽 전처리 계약으로 별도 확정한다.

| 경로 | 현재 역할 |
|---|---|
| Gazebo joint → coupled Plugin → 모터 bridge → PX4 → MAVLink UDP | 모터 RPM·전압·전류 관측 전달 |
| coupled Plugin → battery sender → PX4 기존 배터리 handler + overlay → MAVLink UDP | 배터리 전압·전류·소비량·SOC 전달 |
| PX4 → 공용 수집기 | 디코딩 원문 기록·최신값 제공. 수집기는 송신·비행 명령을 하지 않음 |
| 수집된 관측 → AI 전처리 → 모델 → 결과 | 이 문서가 명세를 제공하는 후속 연동 경계. 이미 연결된 AI 추론으로 표현하지 않음 |

규격의 필드 정의와 단위는 공식 MAVLink 문서를 참고하되, 이 실행의 기준은 **PX4 v1.17.0의 고정 XML과 그 XML에서 생성한 디코더**다. 웹 development 정의나 다른 설치의 `common` 디코더를 임의로 바꾸지 않는다.[1][2][3]

구현 근거는 `Runtime._start_px4`, `Runtime.telemetry`, `MavlinkRecorder`, `CoupledPowerModel`, 실제 PX4 stream과 배터리 overlay다. 파일 링크는 10절에 있다.

## 2. AI용으로 설정·기록하는 관측 메시지 6종

| 메시지 | ID | 서비스의 설정 목표 | AI/전처리에 제공할 수 있는 관측 |
|---|---:|---:|---|
| `ESC_STATUS` | 291 | 20 Hz | 네 모터의 보고 RPM·전압·전류 |
| `BATTERY_STATUS` | 147 | 5 Hz | 한 배터리 팩의 셀 전압·팩 전류·소비량·SOC 등 |
| `ATTITUDE` | 30 | 10 Hz | 자세와 각속도 |
| `LOCAL_POSITION_NED` | 32 | 10 Hz | 로컬 위치와 속도 |
| `EXTENDED_SYS_STATE` | 245 | 5 Hz | 지상/공중/이착륙 상태 |
| `HEARTBEAT` | 0 | 2 Hz | 무장 여부·모드·기체/자율비행 제어기 종류·상태 |

**Hz는 `Runtime._start_px4`가 요청하는 송신 목표이며, 센서 생성률·실제 수신률의 보장이 아니다.** 메시지별로 독립적으로 도착하며, 이 6종이 한 묶음의 동기화된 벡터로 송신되는 것도 아니다.

2026-10-04의 기존 로컬 SITL run `cmu-7cbb5818504d-7ngfgotm`의 원문 전체를 다시 읽어 확인했다. 문서 작성 중 시뮬레이터를 새로 실행한 것은 아니다.

| 메시지 | 실제 저장 건수 |
|---|---:|
| `ESC_STATUS` | 983 |
| `BATTERY_STATUS` | 255 |
| `ATTITUDE` | 510 |
| `LOCAL_POSITION_NED` | 510 |
| `EXTENDED_SYS_STATE` | 255 |
| `HEARTBEAT` | 102 |
| **합계** | **2,615** |

전량 source system 36/component 1이었고 unknown/bad/error는 0이었다. **36은 해당 실행의 ID**다. 다른 설정에서는 `telemetry/metadata.json`의 실제 source 필터를 확인한다.

이것이 UDP에 다른 메시지가 전혀 없었다는 뜻은 아니다. 같은 종료 summary의 `seen_by_type`에는 `CURRENT_EVENT_SEQUENCE`, `EVENT`, `MISSION_CURRENT`, `PARAM_VALUE`, `COMMAND_LONG`도 있었다. 수집기는 이 타입들을 집계하지만 raw 관측으로 저장하거나 서비스의 6종 최신값에 넣지 않는다. 특히 설정 메시지까지 무차별적으로 AI 특징에 합치지 않는다. 해당 메시지 payload는 이 raw 로그에 보존되지 않았으므로 그 안의 개별 파라미터 값은 여기서 확인했다고 주장하지 않는다.

수집기의 `SELECTED`는 위 6종 외에 `ESC_INFO`, `SERVO_OUTPUT_RAW`도 받을 수 있지만, **현재 CMU 서비스는 이 두 stream을 명시적으로 설정하지 않으며 위 실제 run에는 둘 다 없었다.** 저장 지원 목록 8종을 실제 송신 8종으로 설명하지 않는다. 특히 모터 명령/PWM·ESC 온도/오류를 이미 AI에 전달 중이라고 가정하지 않는다.

## 3. 핵심 모터 관측: ESC_STATUS

| 필드 | wire 타입·단위 | 현재 구현 의미 |
|---|---|---|
| `index` | uint8 | 첫 ESC의 번호. 현재 네 모터는 bank 시작 0 |
| `time_usec` | uint64, µs | 이 bridge가 PX4에서 받은 시각을 붙인 uORB timestamp. 원래 Gazebo 생성 timestamp를 그대로 전달한 값이 아님 |
| `rpm` | int32 배열 4개, rpm | joint 관측으로 복원한 회전수에 보고용 이상 배율을 적용한 값 |
| `voltage` | float 배열 4개, V | 각 슬롯에 동일한 모델 팩 단자 전압을 제공. 서로 독립적인 네 전압 센서 실측이 아님 |
| `current` | float 배열 4개, A | joint RPM의 16V bench lookup 전류에 각 모터의 전류 배율을 적용한 모델값 |

배열 크기·단위와 `index`의 의미는 MAVLink의 ESC_STATUS 정의에 따른다. 다만 실제 생산값의 물리적 의미는 위 프로젝트 구현에 한정한다.[1]

- CLI 모터 1/2/3/4는 배열 슬롯 0/1/2/3이다. 이 bank 안의 번호 매핑은 현재 기체 계약이며 다른 기체의 물리 위치까지 보장하지 않는다.
- 정상 모델은 joint 상태를 이용하지만, **실기체의 전류/RPM 센서 측정값은 아니다.** 전류 lookup은 이상 배율을 적용하기 전 joint RPM을 사용한다.
- RPM 보고를 0.85배로 해도 joint·추진력·PX4 구동 명령을 0.85배로 만드는 것이 아니다.
- 모터 전류 배율은 배터리 부하 계산에도 반영한다. 현재 모델의 팩 전류는 네 모터 전류 합에 보조 부하 0.5 A를 더한다.
- 서로 다른 시점의 ESC와 배터리 메시지를 즉시 더해 일치 여부를 판정하면 안 된다. 송신 주기·양자화·수신 지연이 다르다.

## 4. 핵심 배터리 관측: BATTERY_STATUS

### 4-A. 사용할 수 있는 주요 값

| 필드 | 원문 단위·값 | 해석·변환 | 현재 구현의 한계 |
|---|---|---|---|
| `id` | uint8, 현재 0 | 한 배터리 식별 | 다중 배터리 일반 지원 아님 |
| `voltages` | uint16 배열 10개, mV | 현재 앞 네 칸을 각각 1000으로 나누면 V | 네 직렬 셀 그룹의 동일 모델값. 병렬 두 셀 각각의 측정값이 아님 |
| 팩 전압 | 별도 필드 없음 | **현재 계약에서는** 유효한 앞 네 셀 전압의 합 ÷ 1000 | 일반 MAVLink 기체에는 첫 칸에 팩 전체 전압을 넣는 경우도 있으므로 무조건 4셀로 해석하면 안 됨 |
| `current_battery` | int16, cA | 100으로 나누면 팩 A | 모터 부하 합 기반 모델. `-1`은 미제공 |
| `current_consumed` | int32, mAh | 소비 전하량. 단위 변환 없음 | **정격 용량·현재 완충 용량이 아님**. reset/profile 변경 시 초기화 |
| `battery_remaining` | int8, % | 현재 모델의 SOC 잔량 | **SOH가 아님**. `-1`은 미제공 |
| `temperature` | int16, cdegC | 100으로 나누면 °C | **현재 항상 2500 → 25°C인 고정값**. 열 모델·열 이상 특징으로 사용하지 않음 |

전압·전류·소비량·온도의 표준 단위와 sentinel은 MAVLink BATTERY_STATUS 표에 따른다. 현재의 4셀 구성·고정 온도·모델 의미는 `battery_frame`, 변환 함수, PX4 overlay 및 원문 로그로 확인했다.[1]

### 4-B. 원문에는 있지만 학습 신호로 오해하면 안 되는 값

| 필드 | 현재 run에서 관측한 값 | 의미 |
|---|---|---|
| `battery_function` | 1 | `MAV_BATTERY_FUNCTION_ALL`로 출력하는 구성값 |
| `type` | 1 | PX4 stream이 `MAV_BATTERY_TYPE_LIPO`로 채우는 출력 분류. 원자료 셀 화학의 실측 판정이 아님 |
| `voltages`의 나머지 여섯 칸 | 65535 | 미사용/미제공. 합계에 포함하지 않음 |
| `voltages_ext` | 네 칸 모두 0 | 확장 셀 슬롯 미지원. 네 개의 0V 셀이라는 뜻이 아님 |
| `energy_consumed` | -1 | 에너지 소비 추정 미제공 |
| `time_remaining` | 0 | 잔여시간 추정 미제공 |
| `charge_state` | 이 run에서는 1 | 잔량 경고에서 유도되는 상태. SOH/주입 정답 아님; 모든 상황에서 고정 1이라고 가정하지 않음 |
| `mode` | 0 | 출력 모드 미정 |
| `fault_bitmask` | 0 | 현재 receiver가 주입 고장을 이 flag에 연결하지 않음. 0을 정상 정답으로 사용하지 않음 |

지원하지 않는 값과 실제 0을 구별해야 한다. `voltages_ext`의 미지원 0은 기본 `voltages`의 미지원 65535와 다르다.[1]

### 4-C. 실제 수신 예시

아래는 위 run의 `mavlink_raw.jsonl` **1042번째 레코드**에서 가져온 실제 `BATTERY_STATUS.fields`다. 만들어 낸 예시가 아니며 실행 시마다 값은 달라진다.

```json
{
  "mavpackettype": "BATTERY_STATUS",
  "id": 0,
  "battery_function": 1,
  "type": 1,
  "temperature": 2500,
  "voltages": [3815, 3815, 3815, 3815, 65535, 65535, 65535, 65535, 65535, 65535],
  "current_battery": 2369,
  "current_consumed": 39,
  "energy_consumed": -1,
  "battery_remaining": 94,
  "time_remaining": 0,
  "charge_state": 1,
  "voltages_ext": [0, 0, 0, 0],
  "mode": 0,
  "fault_bitmask": 0
}
```

이 레코드의 변환 결과는 **팩 15.260 V, 23.69 A, SOC 94%, 소비 39 mAh, 고정 25°C**다. 온도는 값이 존재하더라도 유효한 열 학습 신호가 아니다. 이 값들을 다른 시각의 모터 샘플과 하나의 동시 측정으로 합치지 않는다.

## 5. 비행 상태와 문맥 관측

아래 필드는 정상 부하 변화/이착륙 구간을 구별하는 문맥 후보다. 전부를 모델 입력으로 반드시 넣어야 한다는 뜻은 아니다.

| 메시지 | 전체 데이터 필드 | 의미·단위 |
|---|---|---|
| `ATTITUDE` | `time_boot_ms`, `roll`, `pitch`, `yaw`, `rollspeed`, `pitchspeed`, `yawspeed` | 부팅 기준 ms, 자세 rad, 각속도 rad/s |
| `LOCAL_POSITION_NED` | `time_boot_ms`, `x`, `y`, `z`, `vx`, `vy`, `vz` | 부팅 기준 ms, 위치 m, 속도 m/s. North/East/Down 좌표이므로 z는 아래가 양수 |
| `EXTENDED_SYS_STATE` | `vtol_state`, `landed_state` | VTOL 상태·지상/공중/이착륙 enum. 현재 x500의 VTOL 값 0을 고장으로 해석하지 않음 |
| `HEARTBEAT` | `type`, `autopilot`, `base_mode`, `custom_mode`, `system_status`, `mavlink_version` | 기체/제어기 종류, 무장 bit와 모드·상태. PX4 `custom_mode`는 PX4 규칙으로 해석 |

자세·위치·시스템 상태 필드의 기본 의미는 common 정의, heartbeat는 minimal 정의를 따른다.[1][3]

- 지면 기준 고도는 같은 epoch의 기준 z에서 현재 z를 빼서 얻는다. `z` 자체가 무조건 지면고도는 아니다.
- 무장은 `base_mode`의 128 bit로 확인한다. `landed_state`는 1 지상, 2 공중, 3 이륙, 4 착륙이다. unknown을 임의로 지상으로 바꾸지 않는다.
- `HEARTBEAT.mavlink_version=3`이라는 필드를 보고 MAVLink wire protocol v3라고 부르지 않는다.
- 각 `fields`에는 디코더가 붙인 `mavpackettype` 문자열도 있다. 이것은 센서 측정 필드가 아니다.

## 6. 데이터에 붙는 식별·시간·품질 정보

원문 JSONL은 MAVLink payload만이 아니라 수집기가 만든 envelope를 포함한다.

| envelope 항목 | 용도 |
|---|---|
| `message_name`, `message_id` | 메시지 종류 구별 |
| `source_system`, `source_component` | 발행 기체/컴포넌트 구분. 인증 수단 아님 |
| `packet_sequence` | 패킷의 8비트 순환 번호. 타입마다 연속인 샘플 번호나 센서 시각이 아님 |
| `peer` | 로컬 UDP 송신 IP/포트 |
| `receive_utc_ns` | 수집기가 처리한 UTC 시각 |
| `receive_monotonic_ns`, `elapsed_monotonic_ns` | 같은 수집 run의 수신 순서·경과시간 |
| `fields` | 디코더 원문 필드. 저장 시 단위 변환·보간·정답 추가 없음 |
| `quality` | 생산 경로·sentinel에 대한 별도 주석. 자동 고장 판정·정답·센서 유효성 인증이 아님 |

**원본 sim-time, PX4 시각, PC 수신 시각은 서로 다르다.** 특히 BATTERY_STATUS에는 본 계약의 생성 timestamp가 없으므로 수신 시각을 센서 생성 시각으로 바꾸어 부르지 않는다. `world-reset` 뒤 PX4 시각은 새 epoch로 구분하고, `reset`/`profile` 뒤 소비량·SOC의 불연속도 보존한다.

원문 보존 규칙:

- `NaN`, `+Inf`, `-Inf`는 각각 `{"$float":"nan"}`, `{"$float":"inf"}`, `{"$float":"-inf"}`로 직렬화한다. 0으로 채우지 않는다.
- 유한 sentinel은 숫자 그대로 둔다. 전처리 단계에서 필드별 유효성 mask로 구분한다.
- source 필터는 최신값과 모델 품질 주석에 적용된다. raw에는 다른 source의 선택 메시지도 남을 수 있으므로 AI용 원문을 읽을 때도 metadata의 system/component로 선별한다.
- `recorder.healthy=true`만으로 배터리·ESC가 둘 다 있다는 뜻은 아니다. 타입별 수신 나이와 내용, 새 source 시각을 따로 확인한다.
- pause 중에 최근 수신이 있더라도 새로운 센서/모델 계산의 증거가 아니다. 타임스탬프 반복·공백을 보존한다.

## 7. AI 입력에서 제외해야 하는 simulator 정답

| 값 | 실제 위치/상태 |
|---|---|
| 상대 SOH, RPT/profile ID | catalog, `selected_profile`, run metadata·정답 이벤트 |
| 실제 모델 용량 Ah·내부저항 Ω·OCV 곡선 | catalog·모델 설정. BATTERY_STATUS 기본 관측 필드 아님 |
| 이상 대상 모터·전류/RPM 배율·주입 명령 시각 | `groundtruth.events.jsonl*`, `power` 운영 상태 |
| `accepted_revision`, `applied_revision`, reset generation | 모델 제어/적용 확인용. AI 센서 특징 아님 |
| run 식별·설정·실행 경로 | 데이터 관리·분할/추적용. 모델이 외워야 하는 정상/이상 특징 아님 |

따라서 **`status` 전체 JSON을 모델 입력으로 넣지 않는다.** 정답은 학습/평가 구간을 구성할 때 별도로 연결하되 관측 특징으로 흘려보내지 않는다. 주입 요청 시각·적용 ACK·첫 관측 변화 시각도 같은 시점이 아니다.

현재 관측에서 먼저 검토할 특징은 모터별 RPM/A, 팩 V/A, 소비 mAh·SOC와 필요한 비행 문맥이다. 중복 전압 채널을 어떻게 처리할지, missing mask·인과적 시간 정렬·window·정규화는 **아직 이 서비스가 대신 구현한 부분이 아니다**. 고정 온도, 미지원 flag, profile ID를 성능 좋은 특징처럼 넣지 않는다.

모터 온도, 진동/원시 IMU 가속도, 개별 추력, PWM/구동 명령도 현재의 이 6종 관측 계약에 포함하지 않는다. 자세 각속도와 원시 가속도는 다른 값이다.

## 8. 사용자가 직접 확인하는 위치

서비스를 시작한 뒤 두 방법을 구분한다. 시작·비행·종료 절차는 [전체 테스트 가이드](full-test-guide.md)와 [서비스 사용 가이드](cmu-service-guide.md)를 따른다.

```bash
python tools/arion_sim.py watch
python tools/arion_sim.py watch --json
```

- 첫 명령은 사람이 읽기 좋은 V/A/SOC/모터 표시다. Ctrl+C로 표시만 끝낸 후 두 번째 명령을 실행한다.
- 두 번째는 조회 시점의 최신값 JSONL이며 **매 MAVLink 수신 메시지마다 출력하는 stream이 아니다**. 기본 조회 간격은 1초, 같은 값이 반복되거나 중간 수신값이 생략될 수 있다.
- 원문 학습 데이터는 `status`의 `run_dir` 아래 `telemetry/mavlink_raw*.jsonl`을 번호 순서로 읽는다. 종료 후 `telemetry/summary.json`과 건수를 대조한다.
- 같은 UDP 수신 포트에 별도 AI/collector를 하나 더 bind하지 않는다. 현재 수집기는 독점 소켓이다. 실시간 추론 연동에는 기존 수신기의 명시적인 전달 계층 또는 별도 PX4 출력 링크 설계가 필요하다; 이 문서가 그 기능을 추가하지는 않는다.

## 9. 실기체로 옮길 때 유지할 것과 다시 확인할 것

표준 MAVLink 필드명·단위는 재사용할 수 있지만, 실제 기체에 배터리 전류 센서·ESC RPM/전류/전압 telemetry가 있는지는 별도 확인해야 한다. 메시지 정의에 필드가 있다는 사실만으로 센서 값이 채워지지 않는다. 실기체에서는 `arion_cmu_v1`이라는 simulator 품질 프로필을 무조건 적용하지 않는다.

현재 값은 단방향 모사 모델의 관측이며, 실기체 보정·고장 현실성·AI 성능 검증 완료가 아니다. 정상 고부하와 이상 주입을 구별하는 평가, run 단위 데이터 분할, 가용 센서와 동일한 전처리 계약이 후속 과제다.

## 10. 코드·검증 근거

- 활성 stream/조회: [`tools/sim_service/runtime.py`](../tools/sim_service/runtime.py)의 `_start_px4`, `telemetry`.
- 수신 원문·source·품질·시간: [`tools/mavlink_collector/collector.py`](../tools/mavlink_collector/collector.py), [수집기 안내](../tools/mavlink_collector/README.md).
- 모터/배터리 모델: [`include/arion/CoupledPowerModel.hpp`](../include/arion/CoupledPowerModel.hpp), [`BatteryFrame.hpp`](../include/arion/BatteryFrame.hpp).
- 모터 수신/발행: [`ArionMotorBridge.cpp`](../px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp).
- 배터리 입력/왕복 변환: [`battery_sender.cpp`](../tools/battery_sender.cpp), [`0002-sitl-external-battery.patch`](../patches/px4-v1.17.0/0002-sitl-external-battery.patch).
- 별도 vendor checkout: `PX4-Autopilot/src/modules/mavlink/streams/{ESC_STATUS,BATTERY_STATUS,ATTITUDE,LOCAL_POSITION_NED,EXTENDED_SYS_STATE,HEARTBEAT}.hpp`와 `src/modules/mavlink/mavlink/message_definitions/v1.0/`.
- 기존 실제 run의 raw SHA-256: `d7abc2852e20c3b7c51100d9734ed40296544581b07220d524a8a8186bc40c22`. 이것은 위 관측 예시의 근거이며 다른 PC가 재현해야 하는 파일 해시는 아니다. 원문 로그·개인 설정 전체를 저장소에 업로드하지 않았다.

## Sources

[1] https://mavlink.io/en/messages/common.html — MAVLink Common: battery, ESC, attitude, position and state
[2] https://mavlink.io/en/messages/development.html — MAVLink development dialect; pinned XML takes precedence
[3] https://mavlink.io/en/messages/minimal.html — MAVLink Minimal: HEARTBEAT
