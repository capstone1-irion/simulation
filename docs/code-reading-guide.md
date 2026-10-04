# 코드 학습 가이드 — 읽는 순서, 호출 추적, 외부 문서

## 읽기 전 약속과 도착점
- 첫 회독은 실행하지 않고 코드를 따라가는 과정이다. 아래 명령 문자열은 추적할 입력 예시다. 실제 운영·비행은 별도 [서비스 사용 가이드](cmu-service-guide.md)의 순서를 따르며, 설치나 원본 PX4 패치 적용을 독해 연습으로 반복하지 않는다.
- 모든 소스 경로는 `simulation/` 저장소 루트 기준이다. 편집기의 파일 열기·기호 찾기·참조 찾기만으로 따라갈 수 있으며 특정 계정이나 에이전트가 필요 없다.
- 먼저 [`README.md`](../README.md) → [`docs/cmu-service-guide.md`](../docs/cmu-service-guide.md)의 1·2·4·5·6·7절을 읽는다. 준비/실행 명령이 있는 절은 지금 따라 하지 않는다.
- 목표는 “어떤 입력을 누가 바꾸고, 어느 단위·시계로 전달하며, 실패를 어디서 드러내는가”를 설명하는 것이다. 전체 PX4를 숙독하거나 완전한 이해를 보장하는 과정은 아니다.
- 아래 단계마다 **입력 / 출력 / 상태 변경 / 실패 조건 / 테스트 근거**를 다섯 칸으로 적는다. 첫 회독에서는 정상 경로, 두 번째에는 오류·종료 경로를 따라간다.
- 줄 번호는 변경될 수 있으므로 파일 경로와 기호 이름으로 찾는다. `PX4-Autopilot/` 경로는 별도로 준비된 로컬 vendor checkout이며, 이 저장소에 PX4 전체를 업로드했다는 뜻이 아니다.


## 먼저 볼 한 장짜리 순서

**처음부터 PX4 전체, 가장 긴 C++ 파일, 스레드 종료 코드부터 읽지 않는다.** 결과를 보는 입구를 잡고, 값의 출처를 거슬러 올라간 다음 오류 경로로 돌아온다.

| 회독 | 순서 | 이번에 얻을 것 | 다음으로 넘어가는 기준 |
|---|---|---|---|
| 1: 전체 흐름 | 0 → 1 → 2 → 3 | CLI, 서비스, 수집기의 역할·프로세스·통신 경계 | `motor-fault`와 `watch`가 서로 다른 경로인 이유를 설명 |
| 2: 값의 출처 | 4 → 5 → 6 → 7 → 8 | joint→모터 전류→배터리→PX4→MAVLink, 실측/추정/가정 | 전류 한 값의 단위와 변환 위치를 끝까지 추적 |
| 3: 제대로 끝나는가 | 9 → 11 → 10 | 수명, 오류, reset, 시험의 의미 | 정상 경로와 실패 경로를 모두 손으로 설명 |

첫날은 0~2단계만 끝내도 된다. 각 표의 보존 fixture를 전부 정독할 필요는 없다. **현재 사용하는 길을 먼저 익히고, 그 뒤 고정값 시험과 비교**한다.

Python 문법이 막히면 공식 Tutorial의 4장 제어 흐름, 5장 자료구조, 6장 모듈, 8장 예외, 9장 클래스 중 해당 부분만 읽고 코드로 돌아온다.[1][2]
C++는 이 프로젝트에 등장하는 `struct`, 참조/`const`, `std::array`, `std::optional`, 생성자/소멸자를 먼저 식별한다. 포인터·소유권·동시성은 9단계에서 실제 사용 위치와 함께 익힌다. C++ Core Guidelines의 RAII/스마트 포인터/동시성 절은 그때 보충 자료로 쓴다.[12]

### 매 함수에서 적을 다섯 칸

| 칸 | 적을 내용 |
|---|---|
| 입력 | 누가 호출하는가? 값의 타입·단위·유효 범위는? |
| 출력 | 무엇을 반환하거나 발행하는가? 다음 소비자는? |
| 상태 | 어떤 멤버·파일·큐·소켓을 바꾸는가? |
| 시간·실패 | sim-time인가 wall-clock인가? pause/timeout/입력 없음이면? |
| 근거 | 어느 시험의 어떤 assertion이 이 약속을 확인하는가? |

이후의 공식 문서 번호는 맨 아래 **Sources**의 실제 링크다. 12절에는 언제, 어느 절만 읽을지 모아 두었다. 웹 설명보다 이 프로젝트의 고정 버전 소스와 설정을 우선한다.

## 0. 활성 경로와 보존 자료부터 구분하기
| 분류 | 읽을 위치 | 혼동하지 않을 점 |
|---|---|---|
| 현재 운영 | [`tools/arion_sim.py`](../tools/arion_sim.py), `tools/sim_service/`, [`plugins/power/CoupledPowerState.cpp`](../plugins/power/CoupledPowerState.cpp) | `custom_x500_cmu` 모델을 run별로 생성한다. 고정 시험 모델을 그대로 실행하는 구조가 아니다. |
| 현재 배터리 | [`profiles/battery/cmu_runtime/catalog.json`](../profiles/battery/cmu_runtime/catalog.json) | 같은 VAH01 셀의 RPT 000/002/006/013 네 단계. CSV는 SOC 조회표이며 원본 시간 재생이 아니다. |
| 고정 전달 fixture | [`plugins/motor/FixedMotorState.cpp`](../plugins/motor/FixedMotorState.cpp), [`plugins/battery/FixedBatteryState.cpp`](../plugins/battery/FixedBatteryState.cpp) | `arion::FixedMotorState`, `arion::FixedBatteryState`는 식별 가능한 상수로 통로를 확인한다. 정상 물리 모델이 아니다. |
| 초기 모델 | [`models/custom_x500/model.sdf`](../models/custom_x500/model.sdf), [`models/custom_x500_battery_fixture/model.sdf`](../models/custom_x500_battery_fixture/model.sdf) | 전자는 고정 모터, 후자는 거기에 고정 배터리를 더한다. 현재 CMU 모델과 구분한다. |
| 초기 부하 연동 | [`models/custom_x500_power/model.sdf`](../models/custom_x500_power/model.sdf) | 같은 coupled Plugin이지만 CMU catalog가 없다. 순수 모델의 `assumed_linear_4s` 기본 경로로 이어진다. |
| 초기 오프라인 번들 | [`tools/battery_profiles/cmu_evtol.py`](../tools/battery_profiles/cmu_evtol.py), `profiles/battery/cmu_evtol_normal_soh100/` | `ProfileReplay.sample`과 초기 정상 6개 프로파일은 별도 참고·재생 도구다. 현재 서비스의 부하 적분기가 아니다. |
| 별도 검증 도구 | [`tools/run_coupled_flight.py`](../tools/run_coupled_flight.py), [`tests/run_motor_fixed_e2e.py`](../tests/run_motor_fixed_e2e.py) | 전자의 `Collector`는 harness 전용이다. 현재 공용 `MavlinkRecorder`와 같은 구현이라고 읽지 않는다. |
- NASA는 [`docs/nasa-retirement.md`](../docs/nasa-retirement.md)에서 제외 범위만 확인한다. 현재 서비스 선택지나 설치 의존성으로 되살리지 않는다.
- 읽기 완료 질문: “활성 CMU 설정은 어떤 함수가 만드는가? 폴더에 남아 있다는 이유만으로 실행 경로라고 판단하면 왜 안 되는가?”

## 1. 공개 CLI: 사용자의 말이 데이터가 되는 곳
- [`tools/arion_sim.py`](../tools/arion_sim.py): `parse_args` → `main` → `watch` 순서로 읽는다. `initialize`, `background_start`의 자원 생성 세부는 9단계로 미룬다.
- `parse_args`는 명명된 하위 명령만 노출한다. `main`은 `action`, `motor`, `current_scale`, `rpm_scale` 등을 JSON payload로 옮긴다.
- `start`/`serve`는 `Runtime.run_forever`, 나머지 운영 동작은 `sim_service.control_api.request`로 갈라진다. CLI와 상주 서비스는 별도 역할이다.
- `watch`는 `telemetry` 요청을 반복한다. 매 수신 메시지를 저장하는 함수가 아니므로 화면 한 줄을 새 센서 샘플 한 개로 세지 않는다.
- 일반 표시의 배터리 V/A 변환과 `watch --json`의 원래 MAVLink 단위를 비교한다. `status` 전체에는 정답 설정도 포함된다.
- 읽기 팁: `main`의 분기 옆에 “설정 저장 / 프로세스 시작 / 제어 요청 / 표시”를 붙이면 긴 함수가 짧은 네 경로로 보인다.
- 체크포인트: “`motor-fault`는 collector에 명령하는가? `watch`를 끝내는 것과 서비스를 끝내는 것은 같은가?”
- **외부 문서 시점:** `parse_args`에서 막히면 argparse의 Sub-commands 절을 읽고, 바로 `main`으로 돌아온다. 전체 PX4 설치 문서부터 읽지 않는다.[18]

## 2. 서비스 제어와 실제 실행 구성
- [`tools/sim_service/control_api.py`](../tools/sim_service/control_api.py): `request` → `_Handler.handle` → `ControlServer.start`를 읽는다. AF_UNIX 연결의 한 줄 JSON이 `Runtime.dispatch`에 전달된다.
- [`tools/sim_service/runtime.py`](../tools/sim_service/runtime.py): `validate_action` → `Runtime.dispatch` → `Runtime.power` → `Runtime.start` → `Runtime._start_px4` 순서로 이동한다.
- `validate_action`은 허용 키·유한 수치·범위를 검사한다. `dispatch`의 `action_lock`은 상태 변경을 직렬화하고 pause/Disarmed 조건을 검사한다.
- `Runtime.command`는 소유 PX4 PTY, `Runtime.power`는 `arion_power_control`, `Runtime.world`는 Gazebo WorldControl을 사용한다. 이 세 제어 경로는 MAVLink 수집기와 별개다.
- `_start_px4`가 PX4 출력 stream을 설정한다. 수집기가 메시지를 요청하거나 heartbeat를 보내는 구조가 아니다.
- `Runtime.event`와 `run.metadata.json`은 제어·프로파일 정답용, `telemetry/mavlink_raw*.jsonl`은 관측용이다. 운영 `status`를 통째로 AI 특징에 넣지 않는다.
- [`tools/sim_service/model_config.py`](../tools/sim_service/model_config.py): `validate_catalog` → `write_model` → `write_world`. catalog 배열을 SDF에 넣고 `allow_command_fallback=false`를 명시한다.
- `write_world`는 기체를 초기 world에 include한다. `PX4_GZ_MODEL_NAME`으로 기존 기체에 붙이는 것과 PX4가 새 기체를 spawn하는 것을 구별한다.
- [`config/coupled-headless.config`](../config/coupled-headless.config)는 headless 서버의 물리·IMU·기압·GPS·자력계 시스템 구성이다. GUI 유무와 비행 센서 제공 여부는 다른 문제다.
- 체크포인트: “누가 누구를 시작하며, 관측/콘솔/Plugin 제어는 각각 어느 통신 수단을 사용하는가?”
- **외부 문서 시점:** 요청 handler가 어떻게 호출되는지 막히면 socketserver의 Request Handler Objects를, 상태 발행과 제어 요청이 헷갈리면 Gazebo Transport의 Messages/Services를 비교한다.[19][6][20]

## 3. 수신 전용 collector: AI가 실제로 읽을 경계
- [`tools/mavlink_collector/README.md`](../tools/mavlink_collector/README.md)의 원문·최신값·품질 절 → [`tools/mavlink_collector/collector.py`](../tools/mavlink_collector/collector.py)를 연다.
- 핵심 순서: `MavlinkRecorder.start` → `_run` → `_record` → `_write_record` → `latest` → `snapshot` → `close`.
- `_run`은 `recvfrom`과 디코더 `MAVLink(None)`를 사용한다. 송신 함수나 비행 명령이 없다. UDP 데이터그램마다 파서를 만들며 잘린 조각은 `BAD_DATA`로 남긴다.
- `_record`는 선택 메시지와 `UNKNOWN`/`BAD_DATA`를 기록한다. “모든 MAVLink 타입을 원문 저장”하는 것은 아니다. 대상은 `SELECTED`에서 확인한다.
- `_matches`는 **최신값과 모델 주석**을 제한한다. 다른 source의 선택 메시지도 raw에는 남으며 그 품질은 `standard`다. ID는 인증 수단이 아니다.
- `fields`는 디코딩 원문이고 `quality_annotations`는 별도 해석이다. `safe_json`은 NaN/Inf를 태그로 남기지, 0으로 보정하지 않는다.
- 서비스는 `arion_cmu_v1`을 명시한다. collector CLI 기본은 `standard`, 라이브러리 생성자 기본은 `arion_cmu_v1`이므로 호출부까지 확인한다.
- `latest(..., max_age_s=3.0)`은 **수신 나이**를 검사한다. `snapshot()['healthy']`는 배터리·ESC 모두의 유효성 인증이 아니다.
- `_write_record`의 매 줄 flush, 레코드 경계 회전, 배타적 파일 생성을 읽는다. 파일별 상한은 전체 디스크 보관 상한이나 매번 fsync를 뜻하지 않는다.
- `seen_total`, `saved_total`, `datagrams_received`를 다른 카운터로 적는다. raw JSONL은 PCAP도, ULog 변환 결과도 아니다.
- 체크포인트: “한 패킷의 peer/source/sequence/fields/수신 시각을 어디에 저장하는가? 필터 밖 source는 삭제되는가?”
- **외부 문서 시점:** 원문을 읽는 지금 MAVLink의 HEARTBEAT, ESC_STATUS, BATTERY_STATUS 필드 표를 대조한다. development dialect의 현재 설명을 읽더라도 실제 decoder는 고정 PX4 XML에서 생성한 것을 기준으로 한다. 스키마에 필드가 있다는 것과 이 환경이 유효한 값을 생산한다는 것은 다르다.[3][4]

## 4. C++ 계약과 순수 모델: 먼저 단위, 다음 수식
- [`include/arion/MotorFrame.hpp`](../include/arion/MotorFrame.hpp): `arion::MotorState`, `MotorFrame`, `decode_motor_frame`. 12개 값은 **RPM 4개 → V 4개 → A 4개**다. 모터별 세 값 반복 배열이 아니다.
- [`include/arion/BatteryFrame.hpp`](../include/arion/BatteryFrame.hpp): `BatteryWireValues`, `battery_wire_integer`, `decode_battery_frame`. 입력은 ID0·셀V4·팩A·SOC비율·°C·소비mAh의 9개 값이다.
- 배터리 정수 wire 변환은 V→mV, A→cA, 비율→%, °C→cdegC다. `kFixedBatteryValues`는 같은 계약을 쓰는 fixture일 뿐 활성 모델 상수가 아니다.
- 비유한값·범위 밖·필드 수 오류를 거부하고 output을 부분 변경하지 않는 방식을 두 decoder에서 비교한다. 음수 전류 일반론과 이 방전 전용 생산 계약을 혼동하지 않는다.
- [`include/arion/CoupledPowerModel.hpp`](../include/arion/CoupledPowerModel.hpp): `BatteryProfile` → `PowerMotorInput` → `CoupledPowerSample` → `CoupledPowerModel::update` → `update_battery` → `motor_frame`/`battery_frame`.
- joint 입력은 부호 있는 **느리게 표현된 rad/s**다. `update`는 절댓값·slowdown 10·`kRpmPerRadS`로 rotor RPM을 복원한다.
- `current_for_rpm`은 16V bench 표 보간, 표 아래는 가정한 이차식, 위는 상한이다. 전류를 계산하는 RPM은 주입 후 `rpm`이 아니라 `observed_rpm`이다.
- `motor_current_a[i] = current_for_rpm(observed_rpm[i]) * current_scales_[i]`; 보고 RPM에는 별도의 `rpm_scales_[i]`를 적용한다.
- `update_battery`는 모터 전류 합+보조 0.5A, `I*dt/3.6`의 mAh 적분, 용량 대비 SOC, 셀 OCV 보간, `4*OCV-I*R`의 팩 전압을 계산한다.
- 25°C는 고정 placeholder, 0.004V 팩 하한은 wire 안전용이다. 열 모델·실제 차단 전압·전압 저하의 추진력 되먹임으로 해석하지 않는다.
- `update`의 첫 관측·동일 시각·pause·되감기 분기를 수식보다 먼저 표시한다. wall-clock 대기 시간을 배터리 적분에 넣지 않는다.
- `PowerPublishClock::due`와 [`include/arion/BatteryPublishClock.hpp`](../include/arion/BatteryPublishClock.hpp)의 `BatteryPublishClock::due`는 motor 50ms/battery 200ms의 sim-time 스케줄이다. 실제 수신 Hz와 동일하다고 단정하지 않는다.
- 체크포인트: “보고 RPM만 낮추면 왜 전류 lookup 입력은 그대로인가? SOC와 상대 SOH는 각각 무엇인가?”
- 외부 문서는 C++ `array`/`optional`/참조/`constexpr`, 단위 환산·전하 적분·선형 보간·등가회로를 필요한 함수와 함께 읽는다.

## 5. Gazebo Plugin: 누가 언제 모델을 갱신하는가
- [`plugins/power/CoupledPowerState.cpp`](../plugins/power/CoupledPowerState.cpp): `power_config_from_sdf` → `CoupledPowerState::Configure` → `PreUpdate` → `PostUpdate` → `publish` → `Reset`.
- `Configure`는 SDF 검증과 motor/battery topic·power_control service 등록을 한다. build target `ArionCoupledPowerPlugin`과 C++ 클래스 `arion::CoupledPowerState`는 이름이 다르다.
- `PreUpdate`의 `EnableVelocityCheck`는 **관측 component 요청**이다. joint 속도 명령을 쓰는 함수가 아니다.
- `PostUpdate`의 순서는 pending 제어 적용 → joint 읽기 → 순수 모델 `update` → 상태 snapshot 갱신 → 발행 판단이다. 유효한 네 모터가 없으면 발행을 보류한다.
- `publish`는 `gz::msgs::Float_V`에 schema·sim stamp·`generation`·`applied_revision` 등 진단 metadata를 붙인다. 이 metadata 전체가 PX4 MAVLink로 전달된다고 가정하지 않는다.
- [`include/arion/PowerControl.hpp`](../include/arion/PowerControl.hpp): `parse_power_command` → `PowerControlMailbox::handle` → `apply_pending` → `refresh` → `power_status_json`.
- `power_control_callback`은 공유 mailbox만 값으로 캡처한다. callback은 검증·큐잉/상태 읽기만 하고 실제 모델 변경은 `PostUpdate`에서 한다.
- `accepted_revision`은 접수, `applied_revision`은 적용이다. pause에서는 일반 pending 명령이 보류되고 world reset은 이전 큐를 취소할 수 있다.
- `cancelled_through_revision`, `last_cancelled_from_revision`, `generation`(여기서는 `reset_count`), `last_sample_ns`, `sample_valid`를 함께 읽는다. ACK만 보고 새 측정이 생겼다고 말하지 않는다.
- 체크포인트: “callback에서 모델을 직접 바꾸지 않는 이유와 reset 직후 freshness를 무효화하는 이유를 설명할 수 있는가?”
- **외부 문서 시점:** Gazebo Sim 8의 Create System Plugins에서 Configure/PreUpdate/PostUpdate를, Using Components에서 ECM을 읽는다. 다음으로 Transport 13의 Messages/Services를 붙인다. 센서 system 목록은 Server Configuration과 실제 [`config/coupled-headless.config`](../config/coupled-headless.config)를 비교한다.[5][7][8]

## 6. 모터 PX4 브리지: 배열이 uORB가 되는 곳
- [`px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp`](../px4/src/modules/arion_motor_bridge/ArionMotorBridge.cpp): 우선 `ArionMotorBridge::init` → `on_message` → `run` → `stock_disabled`만 따라간다.
- `on_message`는 schema와 `decode_motor_frame`을 확인하고 공유 최신 슬롯을 채운다. `run`이 uORB `esc_status`를 발행한다. 콜백 자체는 uORB를 발행하지 않는다.
- 미소비 최신값은 덮어쓸 수 있다. 각 소비 frame을 최대 한 번 내보내며, 입력 부재/오류를 명령 echo나 새로운 0값으로 대체하지 않는다.
- `run`의 `esc_rpm`, `esc_voltage`, `esc_current`, `esc_address`, `actuator_function` 대입을 확인한다. 내부 슬롯 0..3, 주소 1..4, x500 function 101..104는 서로 다른 번호다.
- [`patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch`](../patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch)는 원본 `GZMixingInterfaceESC.cpp`와 `module.yaml` **두 파일**의 상태 발행 전환이다. `SIM_GZ_ESC_ST` 기본값은 1이다.
- 원본 대상의 정확한 경로는 `PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp`, `PX4-Autopilot/src/modules/simulation/gz_bridge/module.yaml`이다.
- 서비스는 stock ESC 상태를 끄고 별도 bridge를 사용한다. 이 패치는 actuator 명령/비행제어를 교체하지 않으며 자동 fallback도 만들지 않는다.
- 필요한 vendor 코드만 `PX4-Autopilot/src/modules/mavlink/streams/ESC_STATUS.hpp`의 `MavlinkStreamESCStatus::send`를 읽는다. `esc_status` 배열을 MAVLink `rpm`/`voltage`/`current`로 옮긴다.
- **시계 경계:** Plugin sim stamp를 그대로 복사하지 않는다. bridge의 `hrt_absolute_time()` 수신 시각이 uORB timestamp, 이어서 MAVLink `time_usec`가 된다.
- 체크포인트: “Gazebo의 샘플 생성 시각, PX4 수신 시각, Python 수신 시각은 어디서 각각 생기는가?”
- **외부 문서 시점:** PX4 v1.17 uORB Messaging → Hello Sky의 구독/발행 예제 → 실제 bridge의 `run` 순서다. ESC_STATUS 필드 표는 출력 stream을 읽을 때만 옆에 둔다. 문서 예제 모듈을 새로 빌드할 필요는 없다.[9][10][3]

## 7. 배터리 sender와 isolated overlay: 기존 handler를 통과하기
- [`tools/battery_sender.cpp`](../tools/battery_sender.cpp): `decode_message` → `accept_sample` → `DeliveryState::poll` → `forward_frames` → `send_sample`; 다음에 `Subscription`, `UdpSocket`, `run`을 읽는다.
- `decode_message`는 schema·sim stamp·9필드를 검사한다. `accept_sample`은 같은 sim stamp를 중복 처리하고 되감김/0을 허용한다.
- `DeliveryState::poll`은 새 mailbox generation만 소비하고 **수신부터 1 wall초 초과** 샘플을 버린다. 5초 공백은 missing 전이이며 서비스의 `wait` 정책은 stale 재송신 없이 기다린다.
- 여기의 `Mailbox::generation`은 새 수신 번호다. Plugin 모델의 reset generation이나 서비스의 `px4_epoch`와 같은 카운터가 아니다.
- `send_sample`은 같은 system ID·`MAV_COMP_ID_BATTERY`(180)로 **BATTERY_STATUS만** loopback PX4 입력에 보낸다. 송신 담당은 sender이지 recorder가 아니다.
- vendor 참조: `PX4-Autopilot/src/modules/mavlink/mavlink_receiver.cpp`의 `MavlinkReceiver::handle_message_battery_status` → uORB `battery_status` → `PX4-Autopilot/src/modules/mavlink/streams/BATTERY_STATUS.hpp`의 `MavlinkStreamBatteryStatus::send`.
- 기존 handler는 다른 system/자기 autopilot component를 거부한다. 새 배터리 전용 PX4 모듈을 만든 경로가 아니다.
- [`patches/px4-v1.17.0/0002-sitl-external-battery.patch`](../patches/px4-v1.17.0/0002-sitl-external-battery.patch)를 **위 원본 함수와 나란히** 읽는다. 원본 checkout에는 이 overlay 수정이 없으므로 원본만 보면 현재 변환을 놓친다.
- 패치는 battery ID0→uORB ID1, external source, 온도/전류/SOC 단위·unknown sentinel, 재송신 변환을 다룬다. 다중 배터리 일반 지원이 아니라 ID0 제한이다.
- [`tools/build_sitl_overlay.py`](../tools/build_sitl_overlay.py): `build` → `relocate_ipc` → `rewrite_compile` → `rewrite_link` → `package_runtime`. 이 단계는 읽기만 한다.
- 원본 직접 전환 패치(0001), 신규 out-of-tree bridge, **복사본 전용** 배터리·IPC overlay(0002+builder)를 세 종류로 구별한다. 패치 파일 두 개와 원본 수정 파일 두 개는 같은 말이 아니다.
- builder의 `MESSAGES`는 수정 stream header를 포함하도록 복사·재컴파일하는 `mavlink_messages.cpp`다. 복사된 파일 모두가 기능 수정 파일은 아니다. 현재 bridge도 재컴파일하며 `etc`·alias·module symlink까지 묶는다.
- 체크포인트: “sender UDP 성공만으로 PX4 재발행 성공이라고 할 수 없는 이유와 ID/단위가 바뀌는 위치를 설명할 수 있는가?”
- **외부 문서 시점:** BATTERY_STATUS 표에서 mV/cA/cdegC와 unknown 값을 확인한다. 이후 overlay builder를 읽을 때 CMake Tutorial의 라이브러리/사용 요구 사항 절을 보고 실제 target과 링크 입력으로 돌아온다.[3][16]

## 8. 오프라인 CMU 산정: 런타임 수식의 근거 역추적
- [`docs/cmu-runtime-profiles.md`](../docs/cmu-runtime-profiles.md) → [`profiles/battery/cmu_runtime/catalog.json`](../profiles/battery/cmu_runtime/catalog.json)의 한 profile → [`profiles/battery/cmu_runtime/provenance.json`](../profiles/battery/cmu_runtime/provenance.json) 순서로 본다. 큰 숫자 배열부터 외우지 않는다.
- 현재 네 ID는 `cmu_vah01_rpt000`, `cmu_vah01_rpt002`, `cmu_vah01_rpt006`, `cmu_vah01_rpt013`; 상대 용량은 100.000000/95.027445/90.422638/84.812594%다.
- [`tools/battery_profiles/cmu_aging.py`](../tools/battery_profiles/cmu_aging.py): `verify_inputs` → `_segments` → `_measure_rpt` → `extract_rpts` → `select_stages` → `estimate_resistance` → `build_curve` → `generate`.
- 용량은 완전 RPT의 방전 counter 차이로 구하고 전류 적분으로 대조한다. tester `cycleNumber`를 열화 단계의 고유 ID로 쓰지 않는다.
- 저항은 두 완화 step의 정확한 +1초 ΔV/ΔI 공통 적합, OCV는 C/5 전압의 I·R 보정 등가곡선이다. 평형 OCV 실측이나 모든 SOC/온도의 저항 인증이 아니다.
- 4S2P는 같은 셀을 복제한 가상 팩 가정이다. `Q_pack=Q_cell*2`, `R_pack=R_cell*4/2`를 runtime의 Ah/Ω와 연결한다.
- 95% 단계의 저항이 초기보다 조금 낮은 것을 보존한다. 열화 순서를 보기 좋게 강제하거나 지원하지 않는 낮은 SOH 단계를 합성하지 않는다.
- `generate` 결과의 곡선 배열은 `write_model`을 거쳐 C++에 들어간다. 실행 중 매 tick마다 원본 CSV를 읽거나 `ProfileReplay`를 호출하지 않는다.
- 체크포인트: “실측 / 파생 추정 / 가정 / 미검증을 분리하고, 같은 셀 네 RPT를 독립 학습 셀 네 개로 볼 수 없는 이유를 말할 수 있는가?”
- **외부 문서 시점:** CMU eVTOL Battery Dataset **version 3**, DOI `10.1184/R1/14226830.v3`의 버전·파일 설명부터 확인한다. 공식 metadata의 `url_public_html`이 원자료 페이지다. 상세 RPT/저항 산정은 프로젝트 provenance와 그 출처를 대조한다. 데이터셋의 VAH 원자료와 처리된 impedance 자료를 같은 것으로 읽지 않는다.[17]

## 9. 수명과 종료: 정상 데이터보다 나중에, 하지만 생략하지 않기
- `ArionMotorBridge`의 `task_spawn` → `task_entry` → `request_stop` → `stop_command` → `join_until`/`lock_until`로 돌아간다.
- `CallbackState`와 `LifecycleState`의 역할을 분리한다. 콜백이 공유 상태를 값 캡처하는 이유는 unsubscribe 전에 복사된 callback이 owner 파괴 뒤에도 실행될 수 있기 때문이다.
- bridge는 spawn 전에 객체를 초기화·등록한다. stop timeout이면 살아 있는 owner/등록을 유지하고 재시작을 막는다. C++ 객체 파괴와 native thread join/reap 완료는 별개다.
- bridge stop의 5초 예산은 monotonic wall-clock이다. sim-time이 멈춘 상태에서 timeout까지 함께 멈추면 안전한 종료 제한이 되지 않는다.
- [`tools/sim_service/processes.py`](../tools/sim_service/processes.py): `RotatingLog` → `PtyConsole.__init__`/`_wait`/`stop` → `PipeProcess.stop` → `stop_group`. 소유 PTY 양끝·로그·worker·process group을 따로 표시한다.
- `Runtime.close`가 child 종료 코드·강제kill·잔류 group, recorder 최종 오류·남은 worker를 최종 실패에 반영하는지 따라간다. stop 접수나 데이터 수신 성공만으로 종료 성공을 말하지 않는다.
- `Runtime._world_reset`: sender/PX4 정리 → world reset 적용 확인 → `px4_epoch` 증가 → 새 PX4/sender → 메시지별 새 count·freshness·Disarmed 확인. 서비스/Gazebo/recorder/run은 유지되고 공백은 남는다.
- `clear`는 배율만, `reset`은 현재 배터리 SOC/소비량·이상, `restore`는 run 최초 profile, `world-reset`은 world/시계와 PX4 epoch까지 바꾼다.
- 체크포인트: “소켓/스레드/프로세스의 소유자가 누구이며, 부분 시작 실패와 지연 callback에서 누가 아직 살아 있어야 하는가?”
- **외부 문서 시점:** C++ Core Guidelines의 R.1/R.20/CP.2, Python subprocess의 Popen/timeout/종료, threading의 join/Event/Lock을 필요한 위치에서 읽는다. Python finally는 예외 경로 정리에 연결한다. 실제 native-worker 회수 의미는 고정 PX4 `ModuleBase`와 task 구현도 함께 확인한다.[12][14][15]

## 10. 두 번의 손추적 — 코드 위치를 따라 말로 설명하기
### A. 모터 1의 전류 샘플 한 개
1. `CoupledPowerState::PreUpdate`가 `rotor_0_joint` 관측을 요청하고 `PostUpdate`가 `PowerMotorInput::joint_rad_s`의 슬롯 0에 rad/s를 읽는다.
2. `CoupledPowerModel::update`가 `observed_rpm`의 슬롯 0을 복원하고 `current_for_rpm`·전류 배율로 `motor_current_a`의 슬롯 0을 만든다. 이것은 전류 센서 실측이 아니라 모델값이다.
3. `update_battery`가 이 전류를 다른 모터+보조 부하에 합쳐 SOC/팩 전압을 갱신한다. `motor_frame()` 반환 배열의 슬롯 8에 모터 1 A, 전압 슬롯에는 팩 V가 들어간다.
4. motor clock이 허용하면 `publish` → `/arion/<model_instance>/motor_state`의 Float_V. schema는 `arion.motor_state.v1`, header stamp는 sim-time이다.
5. `ArionMotorBridge::on_message` → `decode_motor_frame` → 공유 최신 슬롯 → `run`의 첫 `esc` 원소의 `esc_current`. uORB timestamp는 PX4 수신 시각으로 새로 붙는다.
6. `MavlinkStreamESCStatus::send` → `ESC_STATUS.current`의 슬롯 0 → PX4 UDP 출력. RPM은 bridge에서 정수 반올림되지만 ESC 전류는 A를 유지한다.
7. `MavlinkRecorder._run` → `_record` → `_write_record`: raw의 `fields.current`의 슬롯 0, source/component/sequence/peer, Python 수신 시각이 저장된다. `_latest`는 필터 일치 때만 갱신된다.
8. 같은 부하의 배터리 가지는 `battery_frame` → `/battery_state` → `decode_message`/`send_sample` → 기존 PX4 handler+overlay → uORB → PX4 `BATTERY_STATUS` → 같은 recorder다. 원본 sim stamp는 BATTERY_STATUS wire 필드로 전달되지 않는다.
- 손추적 팁: 각 화살표에 **프로세스 / 타입 / 단위 / 시계**를 하나씩 붙인다. 모든 계산 tick이 transport·uORB·수집의 각 단계에서 1:1 보존된다고 가정하지 않는다.

### B. CLI 이상 주입 한 건
- 실행하지 않을 입력 예: `python tools/arion_sim.py motor-fault 1 --current-scale 1.6 --rpm-scale 0.85`.
1. `parse_args` → `main`: `action='motor-fault'`, `motor=1`, `current_scale=1.6`, `rpm_scale=0.85`를 payload에 넣는다.
2. `request` → UNIX `control.sock` → `_Handler.handle` → `Runtime.dispatch`/`validate_action`. running/직렬화/pause 조건을 통과해야 한다.
3. `dispatch`가 CLI 번호 1을 내부 슬롯 0으로 바꿔 `Runtime.power('fault 0 1.6 0.85')`를 호출한다. 요청 시점은 `action_requested` 정답 이벤트다.
4. [`tools/power_control.cpp`](../tools/power_control.cpp)의 `main` → `parse_options`/`parse_power_command` → Gazebo `Node::Request` → `power_control_callback` → `PowerControlMailbox::handle`.
5. mailbox는 FIFO에 넣고 `accepted_revision`을 응답한다. 다음 실행 가능한 `PostUpdate`의 `apply_pending`이 `set_motor_fault`를 호출하고 `applied_revision`을 올린다.
6. `Runtime.power`는 필요하면 `status`를 재조회해 적용 revision을 기다린다. `dispatch`는 반환 결과를 `action_applied`로 남기며 오류는 `action_error`로 남긴다.
7. 그 다음 모델 관측/발행에서 A·보고 RPM이 달라진다. 적용 ACK와 첫 PX4 수신 시각은 같지 않으며 joint/추력/PX4 구동 명령은 직접 바뀌지 않는다.
- 손추적 팁: “접수됨 / 적용됨 / 새 모델 샘플 / PX4 출력 관측됨”을 네 칸으로 분리한다. timeout은 미적용 증명도 성공 증명도 아니므로 무조건 재시도한다고 읽지 않는다.

## 11. 테스트를 실행 대신 읽는 순서와 마지막 점검
| 순서 | 파일과 정확한 진입 기호 | 찾아낼 계약 |
|---|---|---|
| 1 | [`tests/motor_frame_test.cpp`](../tests/motor_frame_test.cpp): `main`; [`tests/battery_contract_compile_test.cpp`](../tests/battery_contract_compile_test.cpp): `fixed_battery_maps_to_mavlink_units` | 슬롯/단위, NaN·범위 거부. 배터리 파일은 static_assert 계약이다. |
| 2 | [`tests/coupled_power_model_test.cpp`](../tests/coupled_power_model_test.cpp): `joint_response`, `simulation_time_lifecycle`; [`tests/cmu_power_model_test.cpp`](../tests/cmu_power_model_test.cpp): `faults_and_reset` | joint→전류·pause/rewind·전류와 보고 RPM 분리·clear와 reset 차이. |
| 3 | [`tests/test_power_control.cpp`](../tests/test_power_control.cpp): `queued_control`; [`tests/test_power_control_sdf.cpp`](../tests/test_power_control_sdf.cpp): `control_callback_lifetime` | accepted≠applied, reset 취소·generation, owner 해제 뒤 callback. 실제 서비스 discovery 시험은 아니다. |
| 4 | [`tools/mavlink_collector/test_continuous_collector.py`](../tools/mavlink_collector/test_continuous_collector.py): `ContinuousRecorderTests.test_library_exclusive_bind_receive_identity_and_close` | 합성 UDP에 응답하지 않는 수신 전용성·ID·원문·소켓 정리. 같은 클래스에서 회전/NaN/필터도 읽는다. |
| 5 | [`tests/test_continuous_io.py`](../tests/test_continuous_io.py): `SenderOptionsTests.test_wall_freshness_drops_old_frames_and_accepts_reset_zero`; [`tests/test_sitl_overlay.py`](../tests/test_sitl_overlay.py): `OverlayTests.test_unknown_current_and_soc_restream` | sender stale/중복/되감기와 PX4 변환 sentinel 왕복. seam/부분 소스 시험이지 비행 인증이 아니다. |
| 6 | [`tests/test_sim_service.py`](../tests/test_sim_service.py): `PowerStatusTest`, `RecorderShutdownTest`, `WorldResetLifecycleTest` | 불필요 재조회와 필요한 적용 대기, 최종 recorder 오류 전파, epoch reset 순서. mock을 실제 비행으로 읽지 않는다. |
| 7 | [`tests/test_runtime_files.py`](../tests/test_runtime_files.py): `RuntimeFilesTest.test_ipc_markers_across_chunk_boundaries`; [`tests/test_cmu_aging.py`](../tests/test_cmu_aging.py): `StageSelectionTests.test_requires_four_distinct_measured_nearest_stages` | chunk 경계 검사 보존과 실제 단계 선택/합성 거부. 실제 원본 검증은 `PublicationTests`의 별도 조건부 시험이다. |
| 8 | [`tests/test_bridge_lifecycle.py`](../tests/test_bridge_lifecycle.py): `BridgeLifecycleTest`; [`tests/test_bridge_shutdown.py`](../tests/test_bridge_shutdown.py): `BridgeShutdownTest` | 지연 callback·시작 실패·frozen clock·native reap. 테스트 메서드→`_check`→HARNESS 해당 mode 순서로 좁혀 읽는다. |
- 추가 안전 판정은 [`tests/test_e2e_safety.py`](../tests/test_e2e_safety.py), [`tests/test_cleanup_regressions.py`](../tests/test_cleanup_regressions.py)에서 찾되 최초 회독의 필수 전체 독서로 늘리지 않는다. 소스 확인 범위는 JSON에 구분했다.
- 각 시험에서 준비값 → 호출 → 기대값 → 실패 시 cleanup만 밑줄 친다. 테스트 제목, 과거 문서의 통과 기록, 이번 실행 결과를 서로 바꿔 말하지 않는다.
- [`CMakeLists.txt`](../CMakeLists.txt)의 target/option과 [`px4/src/CMakeLists.txt`](../px4/src/CMakeLists.txt), [`px4/src/modules/arion_motor_bridge/CMakeLists.txt`](../px4/src/modules/arion_motor_bridge/CMakeLists.txt)는 마지막에 연결만 확인한다. 이 지도는 빌드를 요구하지 않는다.
- 마무리 설명: 두 손추적을 파일 검색 없이 말하고, 관측/정답 분리·단위/시계·활성/fixture·overlay 경계·종료 실패 한 가지씩 근거 함수로 돌아가 확인한다.
- 테스트 문법이 막힐 때만 Python unittest의 Basic example·Assertions·setUp/tearDown을 본다. 테스트 읽기와 실제 시험 실행은 다르다.[13]
- 이 가이드 자체는 코드 독해 자료이며 실제 통합시험 결과를 대신하지 않는다. 독해 때문에 전체 회귀·비행·원자료 재생성을 자동으로 실행하지 않는다.


## 12. 외부 문서는 언제, 어디까지 읽는가

링크 모음 전체를 선수과목처럼 끝내지 않는다. **지금 읽는 함수에서 필요한 개념만 확인하고, 그 함수의 다음 줄로 돌아오는 용도**다.

| 볼 시점 | 공식 문서·찾을 절 | 코드로 돌아올 위치 | 지금은 생략 |
|---|---|---|---|
| 0단계, PX4와 Gazebo 역할이 헷갈림 | PX4 v1.17 Gazebo Simulation의 개요·standalone/existing model 관련 설명.[11] | `Runtime.start`, `write_world` | 다른 기체·센서·월드 전부 |
| 1단계, Python 기초·CLI | Python Tutorial 4/5/6/8/9장 중 모르는 문법, argparse의 Sub-commands.[1][18] | `parse_args`, `main` | 전체 표준 라이브러리 |
| 2단계, 로컬 제어 서버 | socketserver의 handler, serve/shutdown 흐름.[19] | `_Handler.handle`, `ControlServer`, `Runtime.dispatch` | 인터넷 서버 운영·인증 시스템 구축 |
| 3단계, 수신 데이터 | MAVLink common의 HEARTBEAT·ESC_STATUS·BATTERY_STATUS 표; development dialect 범위.[3][4] | `_record`, `watch` | 모든 메시지 암기·프로토콜 직접 재구현 |
| 5단계, Plugin callback | Gazebo Sim 8 Create System Plugins / Using Components.[5][7] | `Configure`, `PreUpdate`, `PostUpdate` | Gazebo 내부 물리 엔진 전체 |
| 5단계, publish와 request | Gazebo Transport 13 Messages / Services.[6][20] | `publish`, `power_control_callback`, `Node::Request` | 새로운 메시지 형식 설계 |
| 5단계, headless 센서 | Gazebo Sim 8 Server Configuration.[8] | [`config/coupled-headless.config`](../config/coupled-headless.config) | GUI 설정부터 수정하기 |
| 6단계, PX4 내부 전달 | PX4 v1.17 uORB Messaging / Hello Sky의 pub-sub 예제.[9][10] | bridge `run` → `MavlinkStreamESCStatus::send` | 자세제어·EKF 전체 |
| 7단계, 배터리 변환 | MAVLink BATTERY_STATUS의 단위·sentinel.[3] | `decode_battery_frame`, handler와 stream 패치 | 실기체 센서 지원을 가정하기 |
| 7단계 마지막, 빌드 격리 | CMake Tutorial Step 2/3의 library·usage requirements.[16] | `rewrite_compile`, `rewrite_link`, `package_runtime` | 전체 PX4 재빌드 |
| 8단계, 실자료 출처 | CMU version 3 metadata의 DOI·원자료 페이지·파일 설명.[17] | `verify_inputs`, `extract_rpts`, `estimate_resistance` | 큰 CSV를 처음부터 눈으로 전부 읽기 |
| 9단계, 자원 수명 | C++ Core Guidelines R.1/R.20/CP.2; Python subprocess/threading/예외.[12][14][15] | `stop_command`, `Runtime.close`, `PtyConsole.stop` | 구조를 이해하기 전 cleanup 삭제·통합 |
| 11단계, 시험 읽기 | unittest Basic example·assertions·setUp/tearDown.[13] | 각 시험의 준비→호출→기대값 | 테스트 제목만 보고 통과로 판단하기 |

**버전 기준:** 프로젝트 PX4는 v1.17.0의 고정 소스, Gazebo는 Sim 8/Transport 13이다. Python은 3.12 계열 설명을 연결했다. 웹 문서의 패치 버전이나 MAVLink development 정의가 변하더라도 로컬 pinned XML·생성 decoder·실제 코드가 이 실행의 기준이다. CMake 문서는 문법 참고이며 특정 설치 버전을 강제하지 않는다.

## 13. 익히는 방법과 이해 확인

### 효과적인 읽기 습관

1. **한 번에 한 값만 추적한다.** 처음에는 모터 1의 전류만, 다음에는 배터리 전압만 따라간다. 네 모터·모든 상태를 한꺼번에 따라가지 않는다.
2. **함수 정의와 호출부를 함께 연다.** 함수 이름만 보고 의미를 정하지 않는다. `generation`처럼 이름이 같아도 다른 카운터가 있다.
3. **프로세스/타입/단위/시계를 화살표마다 적는다.** Python·Plugin·PX4는 같은 프로그램 안의 함수 호출이 아니다.
4. **접수, 적용, 관측을 구별한다.** `accepted_revision` → `applied_revision` → 새 PX4 수신값은 별도 단계다.
5. **조건문이 지키는 약속을 적는다.** NaN 거부, Disarmed 확인, freshness, callback 공유 소유권은 단순한 중복이 아니다.
6. **코드를 가리고 설명한다.** 설명이 막히면 해당 함수와 시험만 다시 연다. 읽은 줄 수보다 스스로 설명할 수 있는 경계가 중요하다.
7. **수정은 이해 후 작은 단위로 한다.** 별도 학습 브랜치에서 순수 함수 한 개의 입력/기대값을 먼저 바꾸는 정도부터 시작한다. 원본 PX4·데이터 provenance·비행 중 제어 제한은 연습용 수정 대상이 아니다.

### 더 읽기 전에 답해 볼 질문

- `watch`와 raw JSONL은 왜 같은 자료가 아닌가?
- 보고 RPM을 0.85배로 바꿔도 왜 joint/추진력이 곧바로 0.85배가 아닌가?
- ESC 전류 A와 BATTERY_STATUS 전류 cA는 어느 파일에서 구별하는가?
- Plugin sim-time, PX4 수신 시각, collector 수신 시각을 어떻게 구별하는가?
- `clear`·`reset`·`restore`·`world-reset`은 각각 무엇을 보존하는가?
- 수집한 값에 profile ID·주입 배율을 넣으면 왜 관측과 정답을 섞게 되는가?
- callback 공유 상태, C++ 객체 수명, native worker 회수는 왜 따로 확인하는가?
- 원본에 직접 적용한 0001과 복사본에만 적용하는 0002/IPC 변경은 어떻게 다른가?
- 어떤 시험은 합성 입력이고, 어떤 확인은 실제 PX4/Gazebo를 실행해야 하는가?

이 질문들에 **파일·함수·시험을 근거로** 답할 수 있으면 단순히 코드를 읽는 단계를 넘어 구조를 설명하고 수정 영향을 예측하는 단계에 도달한 것이다. 한 번의 독해만으로 모든 상황을 완벽하게 익혔다고 판단하지 말고, 작은 변경의 결과를 예측하고 확인하는 과정을 반복한다.

## Sources

[1] https://docs.python.org/3.12/tutorial/index.html
[2] https://docs.python.org/3.12/tutorial/errors.html
[3] https://mavlink.io/en/messages/common.html
[4] https://mavlink.io/en/messages/development.html
[5] https://gazebosim.org/api/sim/8/createsystemplugins.html
[6] https://gazebosim.org/api/transport/13/messages.html
[7] https://gazebosim.org/api/sim/8/usingcomponents.html
[8] https://gazebosim.org/api/sim/8/server_config.html
[9] https://docs.px4.io/v1.17/en/middleware/uorb
[10] https://docs.px4.io/v1.17/en/modules/hello_sky
[11] https://docs.px4.io/v1.17/en/sim_gazebo_gz/index
[12] https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines
[13] https://docs.python.org/3.12/library/unittest.html
[14] https://docs.python.org/3.12/library/subprocess.html
[15] https://docs.python.org/3.12/library/threading.html
[16] https://cmake.org/cmake/help/v3.28/guide/tutorial/index.html
[17] https://api.figshare.com/v2/articles/14226830/versions/3
[18] https://docs.python.org/3.12/library/argparse.html
[19] https://docs.python.org/3.12/library/socketserver.html
[20] https://gazebosim.org/api/transport/13/services.html
