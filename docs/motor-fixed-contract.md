# 모터 고정값 전달 계약 v1

이 계약은 **통신 검증용 고정 fixture**다. 실측 ESC, 정상 상태 모델, 이상/열화 모델 또는 AI 학습 데이터가 아니다. 비행 중에도 같은 값을 내보내므로 실제 회전·소비전류를 나타내지 않는다.

## 구성과 변경 범위

| 요소 | 선택한 구성 |
| --- | --- |
| PX4 | v1.17.0, `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Gazebo | Harmonic, gz-sim8 / gz-transport13 / gz-msgs10 |
| 기체 | `custom_x500`: 해당 PX4의 x500 SDF에 상태 fixture plugin만 추가 |
| 제어 | 기존 x500 공력·모터 모델·`command/motor_speed` 경로 유지 |
| 상태 생성 | `plugins/motor/FixedMotorState.cpp`, `ArionFixedMotorPlugin` |
| PX4 수신 | out-of-tree `px4/src/modules/arion_motor_bridge` |
| 원본 최소 패치 | `gz_bridge`의 기본 ESC 상태 callback 발행 전환과 그 파라미터 정의만 추가 |
| 외부 출력 | 기존 uORB `esc_status` instance 0 → 기존 MAVLink `ESC_STATUS` |

## Gazebo Transport 입력

- 타입: `gz.msgs.Float_V`.
- 토픽: `/arion/<실제로 생성된 모델 이름>/motor_state`.
- PX4 instance 0이면 `/arion/custom_x500_0/motor_state`; 다른 instance에는 해당 숫자가 붙는다.
- `header.data`에 `key="schema"`, `value="arion.motor_state.v1"`가 정확히 하나 필요하다.
- `data`는 **정확히 12개**: RPM 4개, 전압 4개, 전류 4개 순서.
- `header.stamp`는 Gazebo 시뮬레이션 시각(sec/nsec). Bridge는 이 시각을 PX4/MAVLink 시각으로 복사하거나 UTC와 정렬하지 않는다.
- 발행 목표: 시뮬레이션 시각 기준 20 Hz. Pause에서는 새 상태를 생성하지 않는다. 실제 수신 주기는 별도로 측정한다.

| 논리 모터 | SDF joint / motorNumber | 제어 할당 함수 | uORB / MAVLink slot | RPM | V | A |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `rotor_0_joint` / 0 | 101 | 0 | 1101 | 15.1 | 1.1 |
| 2 | `rotor_1_joint` / 1 | 102 | 1 | 2202 | 15.2 | 2.2 |
| 3 | `rotor_2_joint` / 2 | 103 | 2 | 3303 | 15.3 | 3.3 |
| 4 | `rotor_3_joint` / 3 | 104 | 3 | 4404 | 15.4 | 4.4 |

이 전압 차이는 번호/배열 위치 검증을 위한 fixture이며 같은 배터리 버스의 정상 전압 차이를 모델링한 것이 아니다. `ESC_STATUS.index=0`인 첫 batch의 네 slot을 사용한다. Bridge는 시작 시 x500 할당 함수 101~104를 확인한다. 실행 중 할당을 변경하지 말고, 변경이 필요하면 Bridge를 먼저 정지한다.

## PX4와 MAVLink의 값·시간

| 항목 | 처리 |
| --- | --- |
| RPM | 입력은 이미 RPM 단위. rad/s 또는 eRPM 변환을 하지 않는다. `esc_rpm`에 가장 가까운 정수로 반올림 |
| 전압 | V를 그대로 `esc_voltage` 및 MAVLink `voltage[]`에 전달 |
| 전류 | A를 그대로 `esc_current` 및 MAVLink `current[]`에 전달; 음수 전류를 일괄 결측 처리하지 않음 |
| 식별 | `esc_count=4`, `esc_address=1..4`, actuator function 101..104; 연결 수 밖의 slot은 유효 모터로 취급하지 않음 |
| 시각 | `esc_status.timestamp` / `ESC_STATUS.time_usec`는 Bridge 수신 당시 PX4 `hrt_absolute_time()`의 µs 시각 |
| online | 최근 수신한 유효 frame에서 네 채널 존재를 표시; 이후의 마지막 flags를 영구 연결 증거로 해석하지 않음 |
| armed | PX4 `actuator_armed`를 읽는 proxy. 개별 ESC에서 측정한 arming 상태가 아님 |
| 온도·connection type·fault·power | 이 최소 계약에서 미구현. 구조체의 기본 0은 측정값/정상 진단이 아니다. `ESC_INFO.temperature=0`을 온도로 사용하지 않음 |

RPM은 int32 범위이고 모든 입력 값은 유한해야 한다. 길이 오류, NaN/Inf, 범위 밖 RPM 또는 schema 오류는 frame 전체를 거부한다. 유효 frame마다 한 번만 발행하며, 입력 중단/거부 시 마지막 값을 새 시각으로 계속 발행하지 않는다. 수집측은 메시지 부재·시각 노후화를 결측으로 다뤄야 한다. 물리적 타당성 검증, 시간 동기화, AI window/정규화 정책은 후속 T-03 대상이다.

## 단일 상태 발행원

- `SIM_GZ_ESC_ST=1`은 **기본값**이며 기존 command-derived ESC 상태를 발행한다. Bridge 시작은 이때 거부된다.
- `SIM_GZ_ESC_ST=0`은 기존 **상태 callback만** 비활성화한다. 구동 명령 publication과 비행제어는 그대로다.
- 0으로 설정한 뒤 Bridge를 시작한다. Bridge가 없거나 입력이 없으면 ESC 상태는 미갱신이다. 자동 command-echo fallback은 하지 않는다.
- 원래 경로로 복귀할 때 Bridge를 정지한 뒤 1로 설정한다.
- 전환 구간은 학습/평가 데이터에서 제외한다. 시험 도구는 전환과 안정 구간을 별도 phase로 기록한다. 모든 전환의 원자성이나 무손실 handover를 보장하지 않는다.
- 배터리 경로, 동적 RPM/전류 모델, 온도 모델, 비행 검증은 이 계약의 완료를 의미하지 않는다.
