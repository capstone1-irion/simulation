# AI가 사용할 MAVLink 관측값 — 현재 CMU 서비스 계약

## 1. 지금 완료된 경계

이 문서는 **현재 `simulation` 저장소가 PX4 출력에서 수집하도록 구현한 관측·제어 문맥**을 정의한다. 기존 관측 6종과 추가 제어 3종의 송신 설정·원문 저장·최신값 조회를 연결했다. 합성 패킷 시험 외에 **2026-10-04 실제 로컬 SITL의 목표 3m 이륙·호버·착륙에서 9종 수신과 정상 종료를 확인**했다. 실기체·모든 비행 모드·필드별 시간 정렬·개별 모터 물리 대응을 모두 검증한 것은 아니며, AI 최종 입력 벡터·리샘플링·정규화·추론 연결도 후속 작업이다.

| 경로 | 현재 역할 |
|---|---|
| Gazebo joint → coupled Plugin → 모터 bridge → PX4 → MAVLink UDP | 모터 RPM·전압·전류 관측 전달 |
| coupled Plugin → battery sender → PX4 기존 배터리 handler + overlay → MAVLink UDP | 배터리 전압·전류·소비량·SOC 전달 |
| PX4 기존 제어 목표·actuator output → 기존 MAVLink stream → 공용 수집기 | 제어 문맥 기록. 제어기·추진 경로를 변경하거나 새 조종 명령을 보내지 않음 |
| PX4 → 공용 수집기 | 디코딩 원문 기록·최신값 제공. 수집기는 송신·비행 명령을 하지 않음 |
| 수집된 관측 → AI 전처리 → 모델 → 결과 | 이 문서가 명세를 제공하는 후속 연동 경계. 이미 연결된 AI 추론으로 표현하지 않음 |

규격의 필드 정의와 단위는 공식 MAVLink 문서를 참고하되, 이 실행의 기준은 **PX4 v1.17.0의 고정 XML과 그 XML에서 생성한 디코더**다. 웹 development 정의나 다른 설치의 `common` 디코더를 임의로 바꾸지 않는다.[1][2][3]

구현 근거는 `Runtime._start_px4`, `Runtime.telemetry`, `MavlinkRecorder`, `CoupledPowerModel`, 실제 PX4 stream과 배터리 overlay다. 파일 링크는 10절에 있다.

## 2. 수집 설정 9종 — 기존 관측 6종과 추가 제어 3종

| 메시지 | ID | 서비스의 설정 목표 | AI/전처리에 제공할 수 있는 관측 |
|---|---:|---:|---|
| `ESC_STATUS` | 291 | 20 Hz | 네 모터의 보고 RPM·전압·전류 |
| `BATTERY_STATUS` | 147 | 5 Hz | 한 배터리 팩의 셀 전압·팩 전류·소비량·SOC 등 |
| `ATTITUDE` | 30 | 10 Hz | 자세와 각속도 |
| `LOCAL_POSITION_NED` | 32 | 10 Hz | 로컬 위치와 속도 |
| `EXTENDED_SYS_STATE` | 245 | 5 Hz | 지상/공중/이착륙 상태 |
| `HEARTBEAT` | 0 | 2 Hz | 무장 여부·모드·기체/자율비행 제어기 종류·상태 |
| `POSITION_TARGET_LOCAL_NED` | 85 | 10 Hz | PX4의 목표 위치·속도·가속도·yaw. 실제 위치/속도가 아님 |
| `ATTITUDE_TARGET` | 83 | 10 Hz | 목표 자세·각속도·정규화된 전체 추력 명령. 실측 추력이 아님 |
| `ACTUATOR_OUTPUT_STATUS` | 375 | 20 Hz | 출력 채널별 구동 명령 원문. 단위·모터 대응은 생산 경로별 확인 필요 |

**Hz는 `Runtime._start_px4`가 요청하는 송신 목표이며, 센서 생성률·실제 수신률의 보장이 아니다.** 설정과 조회는 `TELEMETRY_RATES` 한 목록을 사용한다. 메시지별로 독립적으로 도착하며, 9종이 한 묶음의 동기화된 벡터로 송신되는 것도 아니다. 제어 메시지는 모드/발행 토픽에 따라 없을 수 있어 기존 지상 시작 필수 조건에는 추가하지 않았다. 빈 최신값은 `{}`이며 0 명령으로 채우지 않는다.

아래는 **제어 3종 추가 이전**인 2026-10-04 로컬 SITL run `cmu-7cbb5818504d-7ngfgotm`의 실제 기록이다. 이 과거 표를 9종 실수신 근거로 사용하지 않는다. 추가 후 실행은 이어지는 2-A절에 별도로 기록한다.

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

이것이 UDP에 다른 메시지가 전혀 없었다는 뜻은 아니다. 같은 종료 summary의 `seen_by_type`에는 `CURRENT_EVENT_SEQUENCE`, `EVENT`, `MISSION_CURRENT`, `PARAM_VALUE`, `COMMAND_LONG`도 있었다. 수집기는 이 타입들을 집계하지만 raw 관측이나 서비스 최신값에는 넣지 않는다. 해당 payload는 이 raw 로그에 보존되지 않았으므로 개별 파라미터·관제 명령 내용을 확인했다고 주장하지 않는다. 이번 추가도 GCS가 보낸 명령을 가로채는 방식이 아니라 PX4가 출력하는 제어 문맥을 받는 방식이다.

수집기의 `SELECTED`는 위 9종 외에 `ESC_INFO`, `SERVO_OUTPUT_RAW`도 받을 수 있어 **저장 지원은 11종**이다. CMU 서비스는 이 두 추가 stream을 명시적으로 설정하지 않으며 아래 새 실행에도 없었다. **지원 11종 / 서비스 설정 9종 / 추가 전 실수신 6종 / 추가 후 실수신 9종**을 구분한다. 기존 서비스를 실행 중이라면 코드를 바꾸는 것만으로 stream이 추가되지 않는다. 먼저 정상 착륙·종료한 뒤 새 서비스로 시작한다.

### 2-A. 제어 추가 후 실제 SITL 수신 확인

2026-10-04 run `cmu-121f29ede7e1-nhu_2dto`에서 기존 로컬 SITL binary로 정상 preflight → 목표 3m 이륙 → 호버 → 착륙·자동 Disarm → 종료를 수행했다. 실행·정리 합계 **33.405초**이며 비행 시간 자체가 아니다. 고장 주입·프로파일 전환·world reset·실기체 접속·새 빌드는 하지 않았다.

| 메시지 | 실제 저장 건수 |
|---|---:|
| `ESC_STATUS` | 552 |
| `BATTERY_STATUS` | 143 |
| `ATTITUDE` | 287 |
| `LOCAL_POSITION_NED` | 287 |
| `EXTENDED_SYS_STATE` | 143 |
| `HEARTBEAT` | 57 |
| `POSITION_TARGET_LOCAL_NED` | 287 |
| `ATTITUDE_TARGET` | 287 |
| `ACTUATOR_OUTPUT_STATUS` | 574 |
| **합계** | **2,617** |

- `telemetry/mavlink_raw.jsonl`의 전 행을 읽어 종료 summary의 타입별/전체 저장 수와 대조했다. source system 36/component 1, unknown/bad/recorder error 0이었다. 원문 파일 SHA-256은 `784e5d5382d380c3d7256e33939f9e5903dcf54d65dfdc89ddd76784309cb849`다.
- 호버 중 `status` 24회와 `watch --json` 7회의 모든 메시지 payload가 실제 저장 원문에 존재했다. 새 3종의 source 시각이 진행하고, 호버의 목표 위치·자세·추력과 앞 네 actuator 출력값이 존재함을 확인했다. 수신기·서비스·PX4·sender·Gazebo를 정상 종료했으며 강제 kill·남은 소유 프로세스/IPC가 없었다.
- 이 실행의 `active`는 **16**이었다. ULog의 `SIM_GZ_EC_FUNC1..4=101..104`, `5..8=0`으로 설정상 Motor1~4 할당을 읽었다. 이는 아래 고정 소스의 rad/s 명령 해석과 함께 사용할 근거지만 개별 모터를 따로 구동해 물리 대응을 확인한 실험은 아니다.
- 원문 위치 목표의 `type_mask`는 0과 63이 관측됐다. 무시된 필드의 NaN을 정상 0으로 바꾸지 않는다. ULog와 MAVLink actuator 표본의 정확한 timestamp 교집합은 없어 일대일 값 대조·시간 정렬 완료를 주장하지 않는다.
- 첫 검증 스크립트 실행은 `watch --json`을 `status`와 같은 중첩 구조로 잘못 읽어 실패했다. 제품 코드를 바꾸지 않고 검증 경로만 수정한 새 실행이 위 통과 run이다. 합성 회귀 17시험도 다시 통과했으며, 합성 근거와 이 실제 수신 근거를 구분한다.

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

아래는 제어 추가 이전 run `cmu-7cbb5818504d-7ngfgotm`의 `mavlink_raw.jsonl` **1042번째 레코드**에서 가져온 실제 `BATTERY_STATUS.fields`다. 만들어 낸 예시가 아니며 실행 시마다 값은 달라진다.

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

### 5-B. 새 제어 문맥의 필드와 해석

MAVLink의 `POSITION_TARGET_LOCAL_NED`와 `ATTITUDE_TARGET`은 autopilot의 제어 목표를 보고하는 메시지다. 외부 제어기가 보내는 `SET_POSITION_TARGET_LOCAL_NED`, `SET_ATTITUDE_TARGET`과 방향·역할이 다르다.[1]

| 메시지 | 필드 | 의미·원문 단위 |
|---|---|---|
| `POSITION_TARGET_LOCAL_NED` | `time_boot_ms`, `coordinate_frame`, `type_mask` | PX4 부팅 기준 ms, 좌표계, 사용하지 않는 축/필드 마스크 |
| 같은 메시지 | `x`, `y`, `z`; `vx`, `vy`, `vz` | 목표 위치 m, 목표 속도 m/s. 현재 pinned stream은 LOCAL_NED로 출력 |
| 같은 메시지 | `afx`, `afy`, `afz`; `yaw`, `yaw_rate` | 현재 pinned stream의 목표 가속도 m/s², yaw rad, yaw rate rad/s. 범용 규격에서는 FORCE_SET 마스크에 따라 힘 N일 수 있으므로 원문 마스크 보존 |
| `ATTITUDE_TARGET` | `time_boot_ms`, `type_mask`, `q` | 부팅 기준 ms, 무시 마스크, 목표 quaternion 4요소(w,x,y,z) |
| 같은 메시지 | `body_roll_rate`, `body_pitch_rate`, `body_yaw_rate` | body 각속도 목표 rad/s |
| 같은 메시지 | `thrust` | 정규화된 전체 추력 명령. 일반 범위 0..1, 역추력 가능 기체는 -1..1. N 단위 측정 추력이 아님 |
| `ACTUATOR_OUTPUT_STATUS` | `time_usec`, `active`, `actuator` | 부팅 기준 µs, 활성 출력 정보, float 32요소 출력 배열. 각 원문 숫자의 단위는 해당 드라이버 경로를 확인 |

위 메시지/필드의 규격은 common 정의에 따른다.[1] 다음은 **고정 PX4 소스와 실제 수집 경로에서 반드시 지킬 해석 경계**다.

- `POSITION_TARGET_LOCAL_NED`는 `vehicle_local_position_setpoint`에서 온다. 비유한 필드는 pinned stream이 무시 마스크를 세우므로 마스크와 비유한값을 함께 확인한다. 무시된 목표를 0 목표로 바꾸지 않는다. 관제 UI의 원래 요청값과 내부 제어 목표가 항상 같다고 가정하지 않는다.
- `ATTITUDE_TARGET`은 `vehicle_attitude_setpoint`와 `vehicle_rates_setpoint`를 함께 읽는다. 메시지 시각은 자세 목표에서 가져오며 rate-only 갱신 경로도 있다. **한 메시지의 모든 필드가 그 시각에 새로 계산됐다는 보장은 없다.** pinned stream은 `type_mask`를 설정하지 않아 0으로 남는다. quaternion 전부 0, 비유한값, 오래되거나 0인 source timestamp 등을 전처리에서 구별해야 한다. 같은 timestamp여도 rate가 달라질 수 있어 시각만 보고 메시지 전체를 버리지 않는다. 이 stream의 `thrust`는 자세 목표 `thrust_body` 벡터의 norm으로, 방향/부호가 사라진 크기다.
- `ACTUATOR_OUTPUT_STATUS`는 `actuator_outputs`의 **instance 0**을 복사한다. pinned stream의 `active`는 `noutputs`를 대입한 개수이며 모터 고장 flag나 bitmask가 아니다. GZ ESC mixer는 구성/회전 중 모터 수가 아니라 출력 용량을 발행한다. **네 모터라고 `active=4`라고 가정하지 않는다.** 또 복사 길이가 첫 송신의 `noutputs`로 고정되는 구현이 있어 이후 `active` 값만으로 모든 슬롯이 갱신됐다고 보증할 수 없다. 미사용 tail의 0과 정지한 모터의 0도 구분해야 한다.
- 출력값을 모든 환경에서 PWM µs·정규화 추력·측정 RPM으로 이름 붙이지 않는다. 소스로 추적한 GZ ESC 경로는 정수 출력값을 Gazebo angular velocity 명령(rad/s)으로 직접 전달한다. stock airframe 4001은 처음 네 채널에 Motor1~4를 배정하지만, 이것만으로 실행 중 instance 0의 소유자와 custom 모델/실기체의 채널 대응을 검증한 것은 아니다. 실수신 단계에서 출력 드라이버/instance·단위·유효 채널·모터 대응을 확인한 뒤 파생 특징을 만든다.
- 새 `quality.fields`는 목표/출력 명령이 측정값과 다름을 표시한다. 마스크 적용·좌표 변환·RPM 변환·유효성 인증·시간 동기화는 수행하지 않는다. 원문은 그대로 보존한다.
- 2-A절에서 새 3종의 실제 수신·원문 저장·호버 source clock 갱신을 확인했다. AI 특징 채택 전에는 사용할 모드별 필드 유효성·인과적 시간 정렬·ESC와의 개별 모터 대응을 별도로 확정한다. 합성 UDP·mock 프로세스 시험을 실제 비행 근거로 대신하지 않는다.

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

## 7. AI 입력 경계와 정상 전용 학습

### 7-A. simulator 정답은 특징에서 제외

| 값 | 실제 위치/상태 |
|---|---|
| 상대 SOH, RPT/profile ID | catalog, `selected_profile`, run metadata·정답 이벤트 |
| 실제 모델 용량 Ah·내부저항 Ω·OCV 곡선 | catalog·모델 설정. BATTERY_STATUS 기본 관측 필드 아님 |
| 이상 대상 모터·전류/RPM 배율·주입 명령 시각 | `groundtruth.events.jsonl*`, `power` 운영 상태 |
| `accepted_revision`, `applied_revision`, reset generation | 모델 제어/적용 확인용. AI 센서 특징 아님 |
| run 식별·설정·실행 경로 | 데이터 관리·분할/추적용. 모델이 외워야 하는 정상/이상 특징 아님 |

따라서 **`status` 전체 JSON을 모델 입력으로 넣지 않는다.** 정답은 학습/평가 구간을 구성할 때 별도로 연결하되 관측 특징으로 흘려보내지 않는다. 주입 요청 시각·적용 ACK·첫 관측 변화 시각도 같은 시점이 아니다.

현재 관측에서 먼저 검토할 특징은 모터별 RPM/A, 팩 V/A, 소비 mAh·SOC와 필요한 비행 문맥이다. 새 제어 목표·출력은 가용성과 의미를 확인한 뒤 부하 문맥으로 결합한다. 중복 전압 채널, missing mask·인과적 시간 정렬·window·정규화는 **아직 이 서비스가 대신 구현한 부분이 아니다**. 고정 온도, 미지원 flag, profile ID를 성능 좋은 특징처럼 넣지 않는다.

모터 온도, 진동/원시 IMU 가속도, 개별 **측정 추력**은 현재 계약에 없다. 새 구동 출력도 이러한 측정을 대신하지 않는다. 자세 각속도와 원시 가속도는 다른 값이다.

### 7-B. 정상 데이터만으로 학습하는 방침

이 프로젝트의 학습 방침은 **다양한 정상 비행만으로 정상 패턴을 학습하고, 그 패턴에서 벗어나는 관측을 이상 후보로 탐지**하는 것이다. 정상 학습 집합으로 새로운 관측의 이탈을 찾는 방식은 novelty detection에 해당한다. 정상/비정상 라벨을 모두 학습하는 이진 분류와 구분한다.[4]

| 단계 | 사용할 데이터 | 역할 |
|---|---|---|
| 모델·전처리 학습 | 정상만 | 정상 패턴과 정규화 통계 학습 |
| 검증·임계값 보정 | 학습에 쓰지 않은 정상 run | 정상 오탐 허용 수준에 맞춰 threshold/지속시간 정책 선택 |
| 최종 평가 | 앞 단계에서 사용하지 않은 정상 + 비정상 run | 오탐, 미탐/탐지율, 탐지 지연, 대상 부품 식별 성능을 별도로 평가 |

이 구분은 프로젝트의 실험 설계 방침이며 이번 변경이 학습기·데이터 분할기·추론기를 구현했다는 뜻은 아니다. **비정상을 학습에서 제외하는 것과 평가에서도 제외하는 것은 다르다.** 비정상 평가가 없으면 정상 오탐은 관찰할 수 있어도 고장을 얼마나 놓치는지는 확인할 수 없다.

운영/학습 계약:

1. **정상 범위를 먼저 고정한다.** 초기 정상 수집 기준으로 `cmu_vah01_rpt000`과 모터 이상 없음을 권장한다. 이것은 모델의 상대 기준이지 실물 SOH100 인증이 아니다. 다른 열화 프로파일을 다양성 명목으로 모두 정상 학습에 넣지 않는다. 허용 열화/저전압 등 경계는 AI팀이 목표 이상 정의와 함께 확정한다. 정상적인 SOC 감소는 SOH 열화와 다르다.
2. 정상 이륙·호버·이동·회전·가감속·착륙, 허용 SOC/부하 범위를 포함하는 데이터가 필요하다. **현재 CLI의 자동 비행은 이륙·호버·착륙까지이며 옆 이동·임무 자동화는 이번에 추가하지 않았다.** 바람/탑재량 등은 모델 지원과 유효 신호를 확인한 범위에서만 확장한다. 같은 결정론적 경로의 반복만으로 다양성을 확보했다고 하지 않는다.
3. **run/비행 단위로 먼저 분할한 뒤 window를 만든다.** 같은 비행의 인접·겹치는 window나 reset 전후를 train/test로 흩뿌리지 않는다. 같은 실물 셀/기체의 출처도 추적하고, 최종 일반화 주장을 위해 경로·조건이 다른 정상 평가 run을 따로 둔다. 서로 연관된 표본을 독립 표본처럼 나누지 않도록 하는 그룹/시계열 분할 원칙을 적용한다.[6]
4. 정규화·결측 처리의 학습 파라미터·특징 선택은 정상 train에서만 fit하고, 검증/평가에는 같은 변환을 적용한다. 평가 데이터를 모델/전처리 선택에 사용하면 누출이다.[5]
5. 제어 이력과 관측 응답을 인과적으로 정렬한다. 미래 보간값이나 미래 명령을 사용하지 않으며 물리 응답 지연, 유효 mask, 공백/epoch를 보존한다. 제어기 보상으로 일부 목표 오차가 작아져도 고장 없음으로 확정하지 않는다.
6. 동일한 정상 train·검증·최종 평가 분할로 서로 다른 정상 패턴 모델을 비교한다. 제어 문맥 포함/제외도 같은 평가 조건으로 비교한다. 정상 검증으로 선택한 임계값을 잠근 뒤 최종 평가한다. 비정상 평가 결과로 임계값·모델을 다시 조정했다면 그 집합은 더 이상 손대지 않은 최종 test가 아니다.
7. [전체 테스트 가이드](full-test-guide.md)의 profile 전환·fault 주입 run은 정상 학습용 run이 아니다. 정상 학습 데이터를 별도 run으로 수집하고, 주입 정답/프로파일 ID를 특징으로 넣지 않는다. 이상 점수만으로 특정 모터나 원인을 자동 식별한다고 약속하지 않는다.

## 8. 사용자가 직접 확인하는 위치

서비스를 시작한 뒤 두 방법을 구분한다. 시작·비행·종료 절차는 [전체 테스트 가이드](full-test-guide.md)와 [서비스 사용 가이드](cmu-service-guide.md)를 따른다.

```bash
python tools/arion_sim.py watch
python tools/arion_sim.py watch --json
```

- 첫 명령은 사람이 읽기 좋은 V/A/SOC/모터 표시다. Ctrl+C로 표시만 끝낸 후 두 번째 명령을 실행한다.
- 두 번째는 조회 시점의 최신값 JSONL이며 **매 MAVLink 수신 메시지마다 출력하는 stream이 아니다**. 기본 조회 간격은 1초, 같은 값이 반복되거나 중간 수신값이 생략될 수 있다.
- 추가 3종은 `watch --json`/`status`의 `messages` 또는 `telemetry.messages`에서 확인한다. 일반 `watch`의 V/A/SOC/RPM 표시 형식은 바꾸지 않았다. 없는 제어 메시지는 `{}`이며 시작 성공만으로 제어 수신 완료로 판단하지 않는다.
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
- 제어의 고정 송신 구현: 같은 vendor checkout의 `streams/{POSITION_TARGET_LOCAL_NED,ATTITUDE_TARGET,ACTUATOR_OUTPUT_STATUS}.hpp`. 이번 추가에서 PX4 원본·플러그인·기존 추진/제어 경로는 수정하지 않았다.
- 집중 시험: [`test_continuous_collector.py`](../tools/mavlink_collector/test_continuous_collector.py)의 합성 11종 원문/회전/품질, [`test_sim_service.py`](../tests/test_sim_service.py)의 mock stream 설정·제어 최신값 API/비유한값/만료. 실제 SITL 수신과는 구분한다.
- 기존 실제 run의 raw SHA-256: `d7abc2852e20c3b7c51100d9734ed40296544581b07220d524a8a8186bc40c22`. 이것은 위 관측 예시의 근거이며 다른 PC가 재현해야 하는 파일 해시는 아니다. 원문 로그·개인 설정 전체를 저장소에 업로드하지 않았다.

Sources:
[1] https://mavlink.io/en/messages/common.html
[2] https://mavlink.io/en/messages/development.html
[3] https://mavlink.io/en/messages/minimal.html
[4] https://scikit-learn.org/stable/modules/outlier_detection.html
[5] https://scikit-learn.org/stable/common_pitfalls.html
[6] https://scikit-learn.org/stable/modules/cross_validation.html — Cross-validation: grouped and time-dependent data
