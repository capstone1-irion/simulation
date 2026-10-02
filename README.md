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
| `px4/modules/arion_motor_bridge/` | PX4 측 별도 브리지 모듈 |
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

## 시작 상태

이 레포에는 협업용 기본 구조만 있습니다. 플러그인·브리지·수집 프로그램은 아직 구현되지 않았으며 빌드·비행 검증 완료를 의미하지 않습니다. PX4 전체 소스, 가상환경, 빌드 결과와 로그는 업로드하지 않습니다. 실제 개발 시 PX4/Gazebo 버전과 재현 가능한 설치·실행 방법을 명시합니다.

## 관련 레포

- [`ai`](https://github.com/capston1-irion/ai): 공통 전처리, 모델 개발과 평가
- [`ui`](https://github.com/capston1-irion/ui): 상태와 이상 결과 표시
- [`docs`](https://github.com/capston1-irion/docs): 공통 인터페이스와 통합 시연 문서
