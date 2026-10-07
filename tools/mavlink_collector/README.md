# MAVLink 수신 전용 수집기와 상시 배터리 sender

수집기는 PX4가 **이미 전송 중인** MAVLink UDP만 받습니다. 메시지 요청·heartbeat·비행 명령을 보내지 않으며 PX4/Gazebo를 시작하지 않습니다. 별도 배터리 sender는 명시한 loopback PX4 입력 포트로 **BATTERY_STATUS만** 보냅니다. 두 역할을 혼동하지 마세요.

## 준비

- 기존 Python 환경의 `pymavlink`와 프로젝트 PX4 정의에서 생성한 Python 디코더를 사용합니다. 설치 파일이나 PX4를 수정하지 않습니다.
- 디코더는 Python 코드로 import되므로 **신뢰하는 파일만** 지정합니다.
- `collector.py`는 디코더의 `MAVLink(None)` 파서와 표준 UDP 소켓을 직접 사용합니다. `mavutil.mavlink` 전역 변경, `sys.path` 추가, 테스트 harness import가 없습니다.
- 수신 소켓에는 `SO_REUSEADDR`/`SO_REUSEPORT`를 설정하지 않습니다. 같은 주소·포트를 다른 수집기와 공유하지 마세요. 사용 중이면 bind가 실패합니다. 별도 포트/별도 PX4 출력 링크를 준비해야 합니다.

아래 `~/px4-workspace`는 예시 경로입니다. 기존 Python과 디코더 위치에 맞추세요. `hermes/mavlink/px4_messages.py`라는 기본 경로는 이 프로젝트의 생성 파일 위치일 뿐, Hermes 실행 환경을 요구한다는 뜻이 아닙니다.

## CLI

```bash
cd ~/px4-workspace/simulation
.venv/bin/python tools/mavlink_collector/collector.py \
  --decoder ../hermes/mavlink/px4_messages.py \
  --endpoint udpin:127.0.0.1:14540 \
  --duration 0 --label continuous \
  --source-system 42 --source-component 1 \
  --quality-profile arion_cmu_v1 --rotate-bytes 16777216
```

실제 송신자가 이 포트로 보내도록 구성한 경우에만 실행합니다. `42`는 예시이며 실제 PX4 system ID로 바꿉니다. `READY <새 결과 폴더>`는 bind/worker 시작을 뜻하며 **수신 성공은 아닙니다**. `--duration 0`은 Ctrl+C 또는 SIGTERM까지 계속 실행합니다. 수신 공백만으로 종료하지 않습니다.

| 옵션 | 기본값·의미 |
|---|---|
| `--duration` | `30`초. 양의 유한한 기간 또는 `0`(무기한). 음수·NaN·무한대 거부. 준비 완료 뒤의 wall-clock 수집 기간 |
| `--endpoint` | `udpin:127.0.0.1:14540`. IPv4 숫자 주소, 포트 0~65535. 0은 OS 임시 포트로 metadata에서 확인 |
| `--output-root` | 스크립트 기준 `simulation/experiments`. CLI는 아래에 고유 실행 폴더 생성 |
| `--label` | `capture`. ASCII 영문·숫자·`_`·`-` 1~64자 |
| `--decoder` | 기존 프로젝트 `hermes/mavlink/px4_messages.py` |
| `--source-system` | 미지정: 모든 system 허용. 다중 기체에서는 반드시 명시 |
| `--source-component` | `1`. 최신값의 추가 ID 필터. **최신값은 언제나 autopilot component 1만 허용**하므로 다른 값을 고르면 최신값은 비어 있음 |
| `--quality-profile` | **CLI 기본 `standard`**. `arion_cmu_v1`, `legacy_inspected_px4_setup`를 명시 선택 가능 |
| `--rotate-bytes` | `16777216`바이트. 정수 1024 이상. JSONL 파일별 UTF-8 크기 상한 |
| `--ulog` | 선택적 기존 파일. 경로 연관만 기록하며 읽기·복사·동일 비행 판정·시간 정렬은 하지 않음 |

Source 필터는 원문을 지우지 않습니다. 선택 11종과 UNKNOWN/BAD_DATA는 **모든 송신자**에서 저장하고, 최신값/모델 프로필 적용만 제한합니다. 필터 밖 원문에는 `standard` 주석을 적용합니다. MAVLink ID와 UDP peer는 **인증 수단이 아닙니다**. 외부 bind 주소를 명시하면 외부 패킷도 받을 수 있습니다.

## 서비스용 Python API

`simulation/`을 작업 디렉터리로 한 예시입니다. 서비스 구현은 테스트 파일을 import할 필요가 없습니다.

```python
from pathlib import Path
from tools.mavlink_collector.collector import MavlinkRecorder

recorder = MavlinkRecorder(
    decoder_path=Path("../hermes/mavlink/px4_messages.py"),
    output_dir=Path("experiments/new-service-run/raw"),  # 존재하지 않는 새 경로
    bind_host="127.0.0.1",
    bind_port=0,
    source_system=42,
    source_component=1,
    quality_profile="arion_cmu_v1",
    rotate_bytes=16777216,
)
try:
    recorder.start()  # self 반환
    port = recorder.port  # 실제 bind 포트; PX4 출력 목적지는 부모가 별도로 구성
    heartbeat = recorder.latest("HEARTBEAT", max_age_s=3.0)
    health = recorder.snapshot()
    # 부모 서비스는 health['errors'], worker_alive와 타입별 수신 나이를 검사해야 함
finally:
    recorder.close()
```

- 정확한 생성자 기본값은 위와 같지만 `source_system=None`, `output_dir`/`decoder_path`는 필수입니다. **API 기본 품질은 계약상 `arion_cmu_v1`**, 범용 CLI 기본은 `standard`입니다. 실기체/다른 생산자는 명시적으로 `standard`를 선택하세요.
- `start()`는 생성자 검증 뒤 새 폴더·소켓·worker를 소유합니다. 기존 폴더는 비어 있어도 거부합니다. 실패하면 예외를 반환하고 획득한 자원을 닫습니다. 폴더 확보 이후 실패한 경우 가능한 범위에서 오류 summary를 남깁니다.
- `latest(name, max_age_s=3.0)`은 해당 타입의 가장 최근 선택 autopilot **fields 사본**을 반환합니다. 없거나 수신 나이가 초 단위 한도를 넘으면 `{}`입니다. 원문 시간값이 0으로 reset되어도 새 수신으로 갱신합니다. 수신 freshness이지 센서 생성 시각의 freshness 인증은 아닙니다. `source_system=None`이면 여러 system의 component1 값이 교대로 최신값을 차지할 수 있습니다.
- `snapshot()`은 JSON-safe health/개수/경로를 반환하며 원문 이력을 반환하지 않습니다. `errors`는 항상 리스트이고 최근 16개(각 1024자까지), `error_total`은 전체 오류 수입니다. `worker_alive`, `running`, `healthy`, `receive_age_s`, `matching_receive_age_s`, `latest_age_s`를 구분합니다. `healthy`는 worker 활성·오류 없음·선택된 타입 중 최근 수신 3초 이내를 뜻합니다. heartbeat 하나만 있어도 true일 수 있으므로 배터리/ESC 가용성은 타입별로 확인하세요.
- `close()`는 정지를 요청하고 최대 2초 worker join을 기다립니다. 정상 종료는 파일/소켓 회수와 summary 저장 후 반환하고 반복 호출은 안전합니다. timeout이면 `errors`와 살아 있는 worker를 숨기지 않으며 후속 `close()`로 다시 정리를 시도할 수 있습니다. 디스크/커널 I/O 자체의 무한 정지를 Python thread가 강제 취소한다는 보장은 아닙니다. 새 서비스 실행은 새 인스턴스·새 출력 폴더를 사용합니다.
- 라이브러리는 프로세스 신호 핸들러를 바꾸지 않습니다. 부모가 SIGINT/SIGTERM에서 `close()`를 호출하고 health를 확인합니다. CLI만 자체 핸들러를 설치/복원합니다.

## 원문·회전·메모리

CLI는 `UTC시각-label-UUID/`를 만들고, API는 지정한 새 폴더를 만듭니다.

```text
<새 결과 폴더>/
├── metadata.json
├── mavlink_raw.jsonl
├── mavlink_raw.000001.jsonl
├── mavlink_raw.000002.jsonl
└── summary.json
```

원문은 `mavlink_raw.jsonl`부터 시작해 번호 순으로 읽습니다. 모든 파일은 exclusive-create이므로 기존 파일을 덮어쓰지 않습니다. 줄을 쓰기 전에 다음 파일로 회전하여 **각 JSONL의 바이트 크기를 상한 이내로** 유지합니다. 한 레코드 자체가 상한보다 크면 상한을 무시하거나 원문을 조용히 버리지 않고 worker 오류로 종료합니다. 너무 작은 상한을 사용하지 마세요. 회전 파일 수와 전체 디스크 사용량은 계속 늘 수 있습니다. **원문 자동 삭제/보관 정책은 이 수집기에 없으며** 서비스 운영자가 별도로 관리합니다.

메모리에는 선택 타입당 최신값 하나(최대 11개), 타입별 개수, 제한된 오류만 유지합니다. 회전 경로 목록이나 raw 레코드 리스트를 누적하지 않습니다. UNKNOWN ID별 개수는 메모리 증가를 막기 위해 `UNKNOWN` 한 항목으로 집계하고 실제 ID/이름은 원문에 남깁니다. UDP 파서는 데이터그램마다 새로 만들고 최대 UDP 크기로 한정합니다. 서로 다른 데이터그램/peer의 조각을 이어 붙이지 않습니다. 잘린 끝부분은 원문 바이트를 담은 `BAD_DATA` 진단으로 저장하므로 UDP 경계를 넘어 MAVLink 프레임을 분할하는 송신 방식은 지원하지 않습니다.

매 선택 메시지를 저장합니다(타입당 첫 메시지만 저장하는 방식이 아님):

`HEARTBEAT`, `EXTENDED_SYS_STATE`, `ATTITUDE`, `LOCAL_POSITION_NED`, `SERVO_OUTPUT_RAW`, `BATTERY_STATUS`, `ESC_INFO`, `ESC_STATUS`, `POSITION_TARGET_LOCAL_NED`, `ATTITUDE_TARGET`, `ACTUATOR_OUTPUT_STATUS`.

추가 3종은 PX4가 보고하는 제어 목표/구동 출력이며 `SET_*` 명령을 보내는 기능이 아닙니다. CMU 서비스는 기존 6종과 추가 3종의 총 9종을 설정·조회합니다. 이 수집기의 지원 11종 전체를 설정하는 것은 아니며, 모드에 따라 제어 토픽이 없을 수 있습니다. 합성 패킷 시험에 더해 2026-10-04 실제 로컬 SITL 비행에서 새 3종의 수신·저장을 확인한 **과거 이력**이 있습니다. 해당 비행 원문·검증 산출물은 승인된 정리 후 로컬에 보관하지 않으며 이번 문서 갱신은 새 시험이 아닙니다. 실기체·모든 모드·AI 시간 정렬 확인으로 확대하지 않습니다. 당시 건수와 원문 단위·mask·출력 채널 및 정상 전용 학습 원칙은 [AI 계약](../../docs/ai-telemetry-contract.md)을 따릅니다.

`UNKNOWN_*`, `BAD_DATA`도 저장합니다. 나머지 알려진 타입은 `seen_by_type`에 집계하지만 raw에는 저장하지 않습니다.

| 레코드 | 의미 |
|---|---|
| `fields` | `to_dict()` 원래 디코딩 필드. 단위 변환·보간·0/null 대체 없음 |
| `source_system`, `source_component`, `packet_sequence`, `message_id`, `message_name` | 디코더의 MAVLink 식별 정보. sequence는 8비트 순환 |
| `peer` | 실제 UDP 송신 IP와 포트. 이것만으로 최초 생성자/신뢰성을 보증하지 않음 |
| `receive_utc_ns`, `receive_monotonic_ns`, `elapsed_monotonic_ns` | 처리 시각/경과 ns. 센서 생성 시각이나 커널 수신 시각이 아님 |
| `quality` | 원문과 분리된 선언 프로필 기반 주석. 결함 정답이나 유효성 인증이 아님 |

`time_usec`/`time_boot_ms` 등 원래 시각도 그대로 둡니다. UNKNOWN/BAD_DATA는 유효 헤더가 없거나 기본 ID/sequence일 수 있습니다. 원래 UDP 캡처 PCAP이 아니라 **디코딩 원문 기록**입니다.

NaN/Infinity/bytes는 JSON 표준에 맞춰 `{"$float":"nan"}`, `{"$float":"inf"}`, `{"$float":"-inf"}`, `{"$bytes_hex":"ff00"}`로 표현합니다. 유한한 sentinel(-1, 32767, 65535 등)은 숫자 그대로이며 원본 입력 객체도 바꾸지 않습니다. NaN의 IEEE payload까지 보존하는 형식은 아닙니다.

## 품질 프로필

품질은 생산 경로에 대한 **선언**입니다. 프로필 이름을 고르는 것만으로 활성 SDF/드라이버가 확인되지는 않습니다. 원문 숫자에 대한 검사/실기체 보정/AI 성능 검증과 구분하세요.

모든 프로필에서 제어 목표는 측정값이 아님을 표시합니다. `POSITION_TARGET_LOCAL_NED`의 frame/type_mask, `ATTITUDE_TARGET`의 quaternion·각속도·정규화 추력/필드별 freshness, `ACTUATOR_OUTPUT_STATUS`의 드라이버별 단위·모터 대응 미확인을 주석으로 구분합니다. 이 주석은 mask 적용·좌표/단위 변환·활성 채널 판정을 수행하지 않습니다. 원문 0·NaN·마스크를 유지하며, 새 제어값을 관측 RPM/전류나 simulator 고장 정답으로 바꾸지 않습니다.

| 프로필 | 적용 범위 |
|---|---|
| `standard` | CLI 기본. 알려진 표준 sentinel만 보수적으로 표시. `-100` 전류를 특정 시뮬레이터 placeholder로 단정하지 않음 |
| `arion_cmu_v1` | 서비스 API 기본. command fallback OFF인 joint feedback 기반 RPM, bench lookup 전류, 단방향 모델 배터리 전압/전하/SOC 경로 |
| `legacy_inspected_px4_setup` | **명시 선택 때만** 이전 stock 경로의 command-derived RPM·미제공 V/A·전류 -100 placeholder 주석 제공 |

`arion_cmu_v1`의 중요한 한계:

- ESC RPM은 시뮬레이션 joint에서 유래하며 주입 scale이 보고값을 바꿀 수 있습니다. 물리 모터 고장·추력 손실·실측 센서로 표현하지 않습니다.
- ESC 전류는 bench lookup/telemetry scale 모델, 전압은 모델 pack terminal voltage입니다. 채워진 V/A를 legacy의 `unpopulated`로 오표시하지 않습니다. 전압 저하가 추진 물리에 되먹임되는 완전한 결합 모델도 아닙니다.
- 배터리 2500 cdegC는 **고정 25°C, 열 모델 아님**입니다. 다른 수치는 임의로 25°C로 고치지 않습니다. ESC_INFO 온도/고장 flag/error count는 이 브리지에서 미구현이므로 0을 정상 센서/정답 상태로 쓰지 않습니다.
- SOC는 SOH가 아닙니다. 실제 모델 용량·내부저항·주입 고장값·단계 정답을 `fields`/`latest`에 추가하지 않습니다. 그 값은 부모 서비스의 별도 ground-truth 로그에 둡니다.
- 일반 sentinel 주석은 프로필보다 우선합니다. 주석 없는 값이나 유한한 숫자도 물리 유효성을 보장하지 않습니다.

metadata에는 선택 프로필·source 필터·모델/물리 범위·수신 전용 여부·디코더/수집기 SHA-256·pymavlink 버전·회전 상한·선택 ULog 연관을 기록합니다.

## 종료·오류와 summary

- JSONL은 **매 레코드 flush**합니다. 매번 fsync하지 않으므로 전원/저장장치 장애 내구성까지 보장하지 않습니다.
- 수신/파서/파일 쓰기/회전 오류는 worker를 중지하고 `snapshot()['errors']`에 노출합니다. 부모는 이를 실제 서비스 실패로 처리해야 합니다. 자동으로 재시작하거나 오류를 덮지 않습니다.
- summary의 `seen_total`/`saved_total`은 파서 메시지·진단 개수이며, UDP 수는 별도 `datagrams_received`입니다. 손실/정확한 생산 주기 검증 도구는 아닙니다.
- `raw_file_count`, `raw_current_bytes`, `paths.raw`, `paths.raw_current`, `paths.raw_rotation_pattern`으로 결과 위치를 찾습니다. 전체 파일명 목록은 메모리에 쌓지 않습니다.
- CLI 종료 코드: 정상 유한 기간/요청한 SIGTERM `0`, 실행 오류 `1`, 인자 오류 `2`, 유한 기간 내 유효 디코딩 없음 `3`, Ctrl+C `130`. SIGINT/SIGTERM 모두 정상 정리 경로를 거칩니다. 종료 이유는 `duration_elapsed`, `no_data`, `interrupted`, `terminated`, `error`; API 직접 종료는 `closed`입니다.
- SIGKILL/전원 장애에서는 summary가 없을 수 있습니다. 초기화/디스크 오류로 metadata나 summary를 쓸 수 없는 경우도 실패이며, stdout의 READY만으로 완성된 기록으로 취급하지 마세요.

## 배터리 sender의 상시 실행

별도 실행파일 `arion_battery_sender`의 기존 필수 인자 `--send`, `--topic`, `--px4-port`, `--system-id`와 `GZ_PARTITION`을 유지합니다.

```bash
GZ_PARTITION=my-isolated-service ./arion_battery_sender \
  --send --topic /arion/MODEL/battery_state \
  --px4-port 18570 --system-id 42 \
  --duration 0 --stale-policy wait
```

포트·topic·partition·ID는 실행 중인 서비스에 맞춰 부모가 설정합니다. 이 예시가 시뮬레이터를 시작하지는 않습니다.

- 기본 `--duration 20`, 유한 범위 1~120초, 기본 `--stale-policy exit`를 보존합니다. `0`만 무기한이며 서비스는 반드시 `wait`를 함께 선택합니다.
- `exit`는 새 유효 프레임이 5 wall초 없으면 오류. `wait`는 같은 상황에서 `missing`을 한 번 기록하고 계속 대기하며 새 프레임에 `recovered`를 기록합니다. 반복 대기 로그는 쌓지 않습니다.
- 한 칸 mailbox만 유지하고 새 generation만 보냅니다. 수신 시각부터 1초 초과한 값은 보내지 않습니다. 같은 sim timestamp는 중복이며 새 측정으로 재전송하지 않습니다. sim timestamp 역행/0 reset은 받아들입니다.
- 상시 모드의 `udp_sent` 로그는 최대 1Hz 표본이며 **패킷 총수와 같지 않습니다**. 유한 기존 실행은 전송별 로그를 유지합니다. 종료 stderr의 `UDP sends`가 성공한 로컬 전송 수입니다. stdout 로그 보관/회전은 부모 서비스 책임입니다.
- SIGINT/SIGTERM은 wall-clock 루프에서 종료하여 소유 구독/소켓을 회수하고 `0`을 반환합니다. 이전 sender의 SIGINT 130을 정상으로 취급하던 호출자는 새 코드0도 정상 종료로 처리해야 합니다.
- lifetime은 startup/control 시도 제한과 별개입니다. transport 초기화/외부 제어를 제한하는 watchdog은 부모가 별도로 둡니다. 실제 Gazebo의 시작/파괴 지연은 아래 seam 시험으로 인증하지 않습니다.

## 집중 자동 시험

```bash
cd ~/px4-workspace/simulation
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest discover \
  -s tools/mavlink_collector -p test_continuous_collector.py -v
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest discover \
  -s tests -p test_continuous_io.py -v
```

새 시험은 프로젝트 `hermes/cmu-service/io/`의 고유 **synthetic** 폴더에 근거를 보존하며 OS 임시 loopback 포트만 사용합니다. sender 시험은 기존 `pkg-config gz-transport13` 및 생성 MAVLink C 헤더로 실제 translation unit을 빌드합니다. Gazebo/PX4 서버나 실기체는 실행/접속하지 않습니다. 5초를 넘는 pause의 forwarding seam, sim0 복구, stale 재송신 차단, 신호 종료, 독점 bind, 최신값 필터·나이, 선택 전 메시지 저장, UNKNOWN/BAD_DATA, 비유한값, 회전 충돌/오류 노출을 확인합니다. 실제 producer→PX4→출력 통합·비행·무기한 내구성 인증과는 별개입니다.

이전 `test_collector.py`는 현행 계약과 다른 역사적 시험으로 이번 정리에서 삭제했습니다. 현재 계약은 위 `test_continuous_collector.py`와 sender 집중시험이 담당합니다. 과거 synthetic 출력도 정리 후 미보관이며, 새 실행은 새 고유 결과 폴더를 만듭니다. 문서 갱신으로 시험을 새로 실행하거나 과거 시험 전체의 통과를 주장하지 않습니다.
