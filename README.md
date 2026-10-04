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
