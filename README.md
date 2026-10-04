# simulation

PX4 SITL과 Gazebo 기반 배터리·모터/ESC 상태 생성 및 MAVLink 데이터 수집 파트입니다.

## 범위

- Gazebo `custom_x500`, Custom Motor Plugin, Custom Battery Plugin
- 별도 PX4 모듈 `arion_motor_bridge` 및 uORB 연동
- 외부 배터리 입력과 PX4 MAVLink 재송신 검증
- 수신 전용 텔레메트리 수집·저장, 정상/이상 실험 시나리오

## 디렉터리

| 경로 | 역할 |
| --- | --- |
| `plugins/motor/` | 모터 상태 생성 플러그인 |
| `plugins/battery/` | 모터 부하 기반 배터리 플러그인 |
| `models/custom_x500/` | 기체 모델 및 설정 |
| `px4/src/modules/arion_motor_bridge/` | PX4 out-of-tree 브리지 모듈 |
| `patches/px4-v1.17.0/` | 기본 ESC 상태 발행 전환만 추가하는 재현 가능한 최소 패치 |
| `tools/telemetry/` | MAVLink 수신·저장 도구 |
| `scenarios/` | 비행·이상 주입 실험 정의 |
| `tests/` | 파트 검증 |
| `docs/` | 파트별 빌드·실행 문서 |

## 연동 원칙

| 대상 | 계획된 경로 |
| --- | --- |
| 모터 | Motor Plugin → Gazebo Transport → PX4 Motor Bridge → uORB `esc_status` → PX4 MAVLink |
| 배터리 | 모터 부하 → Battery Plugin → 외부 `BATTERY_STATUS` 송신 → PX4 외부 배터리 소스 → uORB `battery_status` → PX4 MAVLink |
| AI 입력 | PX4가 외부로 송신한 MAVLink 텔레메트리 |

기존 비행제어 경로는 유지합니다. 고정값 전달을 먼저 검증한 뒤 동적 모델을 추가합니다. 목표 구동 명령과 모사 상태를 구분하고, 숨겨진 이상 주입 설정은 AI 입력이 아닌 정답 기록으로 관리합니다. 메시지 지원, 단위, 주기와 설정은 선택한 버전에서 확인합니다.

## 현재 구현·검증 범위

모터 4개의 **고정 시험값 RPM·전압·전류**를 Gazebo → 별도 PX4 Bridge → uORB → 실제 MAVLink/Python 수신으로 전달하는 최소 경로를 구현했습니다. PX4 v1.17.0과 Gazebo Harmonic에서 headless·Disarmed 상태로 값·번호·발행원 전환·기본 경로 복귀·입력 부재를 확인합니다. 이는 정상/물리 모델, 배터리 연동, 비행 또는 AI 성능 검증 완료를 의미하지 않습니다.

- 실행 방법: [모터 고정값 연동 가이드](docs/motor-fixed-run-guide.md)
- 필드·단위·번호·시간·제한: [고정값 전달 계약](docs/motor-fixed-contract.md)
- 실제 수신 검증: `tests/run_motor_fixed_e2e.py` — 수신 전용, 모터/비행 명령 없음

PX4 원본에는 모터 구동·비행제어 변경 없이 ESC 상태 발행 전환 파라미터만 추가합니다. 기본값은 기존 경로 유지이며, Bridge 입력 부재 시 명령 값을 자동 대체하지 않습니다. 온도 등 미구현 채널의 0을 측정값으로 사용하지 않습니다. PX4 전체 소스, 가상환경, 빌드 결과와 로그는 업로드하지 않습니다.

## 관련 레포

- [`ai`](https://github.com/capstone1-irion/ai): 공통 전처리, 모델 개발과 평가
- [`ui`](https://github.com/capstone1-irion/ui): 상태와 이상 결과 표시
- [`docs`](https://github.com/capstone1-irion/docs): 공통 인터페이스와 통합 시연 문서

## 기능 추가: 수신 전용 상시 MAVLink 기록기 추가

운영 서비스와 향후 AI가 같은 PX4 원문 수집기를 재사용하도록 한다.

- MavlinkRecorder의 독점 UDP bind, 최신값 사본, 타입별 수신 나이와 bounded 오류 상태를 제공한다.
- 선택 메시지 전량 JSONL 저장, 파일별 회전, metadata/종료 summary와 정상 신호 종료를 구현한다.
- standard/arion_cmu_v1/legacy 품질 설명을 분리하고 숨겨진 주입 설정을 원문 fields에 넣지 않는다.

## 기능 추가: 외부 배터리 입력과 격리 SITL overlay 추가

배터리 모델 출력을 기존 PX4 외부 배터리 수신 경로로 전달한다.

- 배터리9필드 계약·sim-time 발행 시계·고정 Battery Plugin을 추가한다.
- sender가 BATTERY_STATUS만 PX4에 송신하며 duration0/stale wait·sim-time 되감김·신호 종료를 지원한다.
- 배터리 수신/재송신 변환과 POSIX IPC 위치를 별도 overlay에 적용하는 빌더를 제공한다.

## 기능 추가: CMU 정상·열화 4단계 배터리 자료 추가

임의 SOH 축소 대신 실제 CMU v3 같은 셀의 열화 구간으로 선택형 모델 파라미터를 만든다.

- 초기 VAH01/17/27 정상6프로파일과 원본 추적·재생 도구를 보존해 공개한다.
- 전체 VAH01 RPT에서 상대100/95.027445/90.422638/84.812594% 네 구간을 선택한다.
- 용량·effective DC1초 저항·각101점 OCV등가곡선과 catalog/provenance·재생성 도구를 추가한다.
