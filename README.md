# simulation

PX4 SITL과 Gazebo 기반 배터리·모터/ESC 상태 생성 및 MAVLink 데이터 수집 파트입니다.

## 문서 바로가기

| 목적 | 문서 | 주요 내용 |
|---|---|---|
| 시작·제어·종료 및 새 PC 준비 | [CMU 서비스 사용 가이드](docs/cmu-service-guide.md) | 설치 준비, 명령별 사용법, 프로파일·이상 주입, 기록과 종료 |
| **직접 전체 기능 테스트** | **[전체 테스트 가이드](docs/full-test-guide.md)** | 순서별 명령·합격 기준·오류 대응·결과표. 네 모터, 네 프로파일, pause/reset, 반복 비행, 종료·원문 대조 |
| **AI에 제공할 MAVLink 값 확인** | **[MAVLink 데이터 계약](docs/ai-telemetry-contract.md)** | 기존 관측 6종 + 제어 문맥 3종의 수집 설정, 필드·단위·수신 근거 구분, 정상 전용 학습과 별도 이상 평가 원칙 |
| 구현 코드를 순서대로 학습 | [코드 학습 가이드](docs/code-reading-guide.md) | 파일·함수별 읽는 순서, 전류/명령 손추적, 이해 확인 질문과 단계별 공식 문서 |

처음 직접 시험한다면 **전체 테스트 가이드 2~10절**부터 따릅니다. **11절은 코드 자동 회귀·선택형 시험**이며 별도 준비와 실행 조건이 있습니다. MAVLink 데이터 계약은 현재 수집 가능한 관측을 설명하며, 최종 AI 입력 벡터·전처리·추론 연결까지 완료됐다는 뜻은 아닙니다.

제어 문맥(`POSITION_TARGET_LOCAL_NED`, `ATTITUDE_TARGET`, `ACTUATOR_OUTPUT_STATUS`)은 수집·조회 코드와 합성 시험에 더해 **2026-10-04 실제 로컬 SITL의 목표 3m 이륙·호버·착륙에서 수신·저장·최신값 조회를 확인**했습니다. 해당 실행에서 총 9종 2,617건을 저장하고 정상 Disarm·종료까지 확인했습니다. 실기체·모든 비행 모드·AI 시간 정렬 검증은 별도입니다. 학습에는 정상만 사용하되, 최종 성능 평가는 독립된 정상·비정상 데이터로 구분하는 방침입니다.

## 현재 사용: CMU 상시 운영 서비스

이미 준비된 PC는 기존 `config/service.local.json`을 사용하며, 다른 PC의 준비 순서는 서비스 사용 가이드에 있습니다. 개인 계정 경로나 에이전트 실행 환경을 요구하지 않습니다.

- 활성 배터리 자료는 `profiles/battery/cmu_runtime/catalog.json`입니다. 같은 CMU VAH01 셀의 실제 RPT에서 선택한 상대 용량 **100.000000 / 95.027445 / 90.422638 / 84.812594%**의 네 프로파일을 사용합니다. 4S2P는 가상 팩 가정이며 등가 OCV·유효 저항 추정의 한계는 [프로파일 근거](docs/cmu-runtime-profiles.md)에 있습니다.
- 모터 joint 기반 RPM·bench 전류와 배터리를 단방향으로 연동합니다. 모터별 전류/보고 RPM 이상 주입은 **전기 부하·텔레메트리 이상**이지 추진력 고장이 아닙니다. AI 입력용 PX4 MAVLink 원문과 주입 정답은 별도 기록합니다.
- 시작은 Disarmed이며 이륙은 별도 명령입니다. 시간 제한 없는 운영 루프와 파일별 로그 회전을 제공하지만, 24/7 내구성·전체 디스크 보관 상한·장애 자동 복구·부팅 자동 실행을 보장하지 않습니다. pause 중에는 최근 수신도 새 센서 생성의 증거가 아닙니다.
- 지상 네 프로파일 선택·원문 전압 대조, 7초 pause/resume, **지상 `world-reset` 복구와 새 PX4 epoch에서 두 차례 비행**을 확인했습니다. 정상 100%와 열화 84.812594%에서 목표 3m 이륙·호버, 모터 1·4 이상 주입/해제, 착륙·자동 Disarm을 관측했습니다. world-reset은 초기 world에 기체를 포함하고 PX4/sender만 재시작해 같은 서비스·Gazebo·수집기·run을 유지합니다. 짧은 수신 공백과 `px4_epoch` 경계를 기록하며 정확히 3m 정지나 무중단 메시지 수신을 뜻하지 않습니다. 최종 CLI 종료와 소유 프로세스·IPC 정리도 확인했으며 현재 서비스는 종료 상태입니다. 대규모·24/7 내구성·GUI 화면 확인은 수행하지 않았습니다.
- `cmu_evtol_normal_soh100`의 초기 정상 6개 프로파일은 [오프라인 참고·재생 도구](docs/cmu-evtol-normal-profile-guide.md)로 보존합니다. 네 런타임 프로파일과 용도가 다릅니다. NASA 구성은 [활성 경로에서 제외](docs/nasa-retirement.md)되었으며 출처는 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)에 안내합니다.

## 범위

- Gazebo `custom_x500`, Custom Motor Plugin, Custom Battery Plugin
- 별도 PX4 모듈 `arion_motor_bridge` 및 uORB 연동
- 외부 배터리 입력과 PX4 MAVLink 재송신 검증
- 수신 전용 텔레메트리 수집·저장, 정상/이상 실험 시나리오

## 디렉터리

| 경로 | 역할 |
| --- | --- |
| `plugins/motor/` | 초기 고정값 모터 상태 시험 플러그인 |
| `plugins/battery/` | 고정값 배터리 전달 시험 플러그인 |
| `plugins/power/` | 활성 CMU 모터·배터리 부하 연동 플러그인 |
| `models/custom_x500/` | 초기 고정값 시험 기체 모델 및 설정 |
| `px4/src/modules/arion_motor_bridge/` | PX4 out-of-tree 브리지 모듈 |
| `patches/px4-v1.17.0/` | 원본 ESC 상태 발행 전환 패치와 복사 SITL overlay용 외부 배터리 변환 패치 |
| `tools/arion_sim.py`, `tools/sim_service/` | 상시 운영 CLI와 소유 프로세스·제어 관리 |
| `tools/mavlink_collector/` | 수신 전용 MAVLink 원문·품질 주석·회전 기록 |
| `profiles/battery/cmu_runtime/` | 활성 CMU 네 프로파일·SOC 등가곡선·출처 |
| `scenarios/` | 비행·이상 주입 실험 정의 |
| `tests/` | 파트 검증 |
| `docs/` | 파트별 빌드·실행 문서 |

## 연동 원칙

| 대상 | 계획된 경로 |
| --- | --- |
| 모터 | Motor Plugin → Gazebo Transport → PX4 Motor Bridge → uORB `esc_status` → PX4 MAVLink |
| 배터리 | 모터 부하 → Battery Plugin → 외부 `BATTERY_STATUS` 송신 → PX4 외부 배터리 소스 → uORB `battery_status` → PX4 MAVLink |
| AI 입력 | PX4가 외부로 송신한 MAVLink 텔레메트리 |

기존 비행제어 경로는 유지합니다. 고정값 전달을 먼저 검증한 뒤 부하 연동 모델과 CMU 서비스를 추가했습니다. 목표 구동 명령과 모사 상태를 구분하고, 숨겨진 이상 주입 설정은 AI 입력이 아닌 정답 기록으로 관리합니다. 메시지 지원, 단위, 주기와 설정은 선택한 버전에서 확인합니다.

## 초기 고정값 연동 이력

초기 단계에서는 모터 4개의 **고정 시험값 RPM·전압·전류**를 Gazebo → 별도 PX4 Bridge → uORB → 실제 MAVLink/Python 수신으로 전달하는 최소 경로를 구현했습니다. PX4 v1.17.0과 Gazebo Harmonic에서 headless·Disarmed 상태로 값·번호·발행원 전환·기본 경로 복귀·입력 부재를 확인하는 도구입니다. 이 고정값 시험 자체는 정상/물리 모델, 배터리 연동, 비행 또는 AI 성능 검증 완료를 의미하지 않습니다. 현재 CMU 서비스와 별도인 초기 검증 경로로 보존합니다.

- 실행 방법: [모터 고정값 연동 가이드](docs/motor-fixed-run-guide.md)
- 필드·단위·번호·시간·제한: [고정값 전달 계약](docs/motor-fixed-contract.md)
- 실제 수신 검증: `tests/run_motor_fixed_e2e.py` — 수신 전용, 모터/비행 명령 없음

PX4 원본에는 모터 구동·비행제어 변경 없이 ESC 상태 발행 전환 파라미터만 추가합니다. 기본값은 기존 경로 유지이며, Bridge 입력 부재 시 명령 값을 자동 대체하지 않습니다. 온도 등 미구현 채널의 0을 측정값으로 사용하지 않습니다. PX4 전체 소스, 가상환경, 빌드 결과와 로그는 업로드하지 않습니다.

## 관련 레포

- [`ai`](https://github.com/capstone1-irion/ai): 공통 전처리, 모델 개발과 평가
- [`ui`](https://github.com/capstone1-irion/ui): 상태와 이상 결과 표시
- [`docs`](https://github.com/capstone1-irion/docs): 공통 인터페이스와 통합 시연 문서
