# 전체 테스트 가이드 — 사용자가 직접 확인하는 CMU 서비스

## 1. 무엇을 테스트하는가

기존 [CMU 서비스 사용 가이드](cmu-service-guide.md)는 개별 명령·설치·운영 설명이다. 이 문서는 그 명령을 **준비 → 정상 관측 → 프로파일 → 비행·이상 → 초기화·반복 → 종료·기록 확인**으로 연결한 실행 체크리스트다. 처음 실행하는 사람은 **2~10절의 운영 통합 테스트부터** 따라간다.

여기서 전체는 **현재 구현된 시뮬레이션·전달·수집·제어 기능**을 뜻한다. AI 정확도, UI 통합, 물리적 모터 파손, 24/7 내구성, 실기체 시험까지 포함한 완제품 인증이 아니다. 코드 자동 회귀와 사용자 통합 시험도 별도다.

| 구성 | 이 시험에서 하는 일 |
|---|---|
| PX4 SITL | PC에서 이륙·호버·착륙을 제어하고 MAVLink를 출력 |
| Gazebo + coupled Plugin | 기체 물리·센서·joint와 모터 전류/CMU 배터리 상태 계산 |
| 모터 bridge + 배터리 sender | 계산한 상태를 PX4 내부에 반영 |
| 수신 전용 collector | PX4 원문 메시지를 계속 수신·기록 |
| `arion_sim.py` | 위 프로그램의 시작·제어·종료를 담당하는 CLI |
| `watch` | 기록기와 별개인 화면 표시. Ctrl+C로 표시만 끝냄 |

**로컬 SITL 전용이다. 실제 비행제어 보드에는 적용하지 않는다.** 기존 force-arm/센서 검사 우회/전역 `pkill`/소유자 불명 socket 삭제는 사용하지 않는다. 오류가 나면 다음 비행을 이어 붙이지 말고 10절의 실패 대응을 따른다.

### 준비 상태

- Ubuntu의 기존 `simulation/` 저장소, `.venv`, Gazebo Harmonic, 고정 PX4·전용 overlay·Plugin·sender·디코더가 준비돼 있어야 한다.
- 기존 `config/service.local.json`이 있는 PC는 **재설치·init·재빌드를 반복하지 않는다.** 새 PC라면 먼저 [서비스 가이드 8절](cmu-service-guide.md#8-다른-팀원의-pc에서-준비하는-순서)을 끝낸다.
- 다른 서비스/시험이 같은 PX4 instance를 쓰고 있으면 동시에 진행하지 않는다. stock/fixed fixture와 CMU 운영 서비스를 섞지 않는다.
- 이번 설명은 사용자 실행 절차다. 문서 작성 중 시뮬레이터·전체 회귀를 새로 실행하지 않았다. 과거 확인과 사용자가 이번에 확인한 결과를 구분한다.

## 2. 두 터미널과 결과 폴더 준비

아래 `~/px4-workspace/simulation`은 예시다. 본인의 저장소 위치로 첫 `cd`만 바꾼다. 모든 명령은 Ubuntu Bash이며 PX4 `pxh>`에 입력하는 명령이 아니다.

### 터미널 A: 제어와 결과 저장

기존 환경을 활성화하고, 코드와 분리된 새 확인 폴더를 만든다. 패키지 설치·설정 변경·시뮬레이터 시작은 하지 않는다. `pipefail`은 `tee`로 저장할 때 앞 명령의 실패를 숨기지 않도록 한다.

```bash
cd ~/px4-workspace/simulation
source .venv/bin/activate
set -o pipefail
mkdir -p ../artifacts
CHECK_DIR="$(mktemp -d "$(cd .. && pwd)/artifacts/manual-check-XXXXXXXX")"
python tools/arion_sim.py profiles | tee "$CHECK_DIR/profiles.json"
```

**통과 기준:** 아래 네 ID가 나온다. 설정 파일이 없는데 `profiles`만 출력됐다고 실행 준비까지 끝난 것은 아니다.

- `cmu_vah01_rpt000`
- `cmu_vah01_rpt002`
- `cmu_vah01_rpt006`
- `cmu_vah01_rpt013`

다른 설정을 선택했다면 모든 CLI에서 `python tools/arion_sim.py --config <설정파일> <하위명령>` 형태를 일관되게 사용한다. 아래는 기본 설정 기준이다.

### 터미널 B: 관찰

같은 저장소에서 같은 `.venv`를 활성화만 해 둔다. `watch`는 다음 절에서 시작 성공 후 실행한다.

```bash
cd ~/px4-workspace/simulation
source .venv/bin/activate
```

## 3. Disarmed 시작과 기록 확인

터미널 A에서 정상 기준 프로파일로 시작한다. 프로그램·모델·원문 기록 폴더를 새로 만들지만 이륙하지 않는다.

```bash
python tools/arion_sim.py start --profile cmu_vah01_rpt000 --background | tee "$CHECK_DIR/start.json"
python tools/arion_sim.py status | tee "$CHECK_DIR/status-start.json"
RUN_DIR="$(python tools/arion_sim.py status | python -c 'import json,sys; s=json.load(sys.stdin); assert s["state"]=="running" and s["ok"]; print(s["run_dir"])')"
```

오류가 나면 `RUN_DIR`이 설정됐다고 가정하지 않는다. `start`를 반복하거나 무조건 오래 기다리지 말고 반환한 오류·launcher 로그를 확인한다. CLI는 시작 준비에 115초 제한을 사용하며 실패 정리는 추가 시간이 들 수 있다.

**통과 기준:**

- `state: running`, `ok: true`, `errors: []`.
- `selected_profile`이 `cmu_vah01_rpt000`.
- `telemetry.messages`의 `BATTERY_STATUS`, `ESC_STATUS`, `HEARTBEAT`, `LOCAL_POSITION_NED`, `EXTENDED_SYS_STATE`가 비어 있지 않고 타입별 수신이 이어짐. `ATTITUDE`도 확인한다.
- `HEARTBEAT.base_mode`에 무장 bit 128이 없고 `EXTENDED_SYS_STATE.landed_state`가 1(지상).
- `recorder.healthy` 하나가 아니라 위 타입별 내용을 확인한다.

터미널 B에서 실시간 값을 본다.

```bash
python tools/arion_sim.py watch
```

배터리 V/A/SOC/mAh와 네 모터 RPM·ESC_A가 보인다. 지상에서 모터가 돌지 않으면 RPM/모터 전류 0은 정상일 수 있고, 팩에는 보조 부하가 남는다. 고정 온도는 열 측정값이 아니다. 값의 정확한 의미는 [AI MAVLink 계약](ai-telemetry-contract.md)을 본다.

Ctrl+C로 **watch만 종료**하고 JSON 표시도 잠깐 확인한다.

```bash
python tools/arion_sim.py watch --json
```

확인 후 Ctrl+C, 다시 일반 `watch`를 켜 두어도 된다. `watch --json`은 주기적 최신값 조회이며 원문 기록을 대체하지 않는다. 별도 collector를 같은 UDP 포트에 띄우지 않는다.

## 4. 네 프로파일·clear/reset/restore·pause

### 4-A. 네 배터리 선택 — 지상에서 하나씩

다음 명령을 **한 줄씩** 실행하고 `status`를 확인한 뒤 다음 ID로 간다. 각 `profile`은 SOC/소비량·모터 이상을 초기화한다.

```bash
python tools/arion_sim.py profile cmu_vah01_rpt000
python tools/arion_sim.py status | tee "$CHECK_DIR/profile-000.json"
python tools/arion_sim.py profile cmu_vah01_rpt002
python tools/arion_sim.py status | tee "$CHECK_DIR/profile-002.json"
python tools/arion_sim.py profile cmu_vah01_rpt006
python tools/arion_sim.py status | tee "$CHECK_DIR/profile-006.json"
python tools/arion_sim.py profile cmu_vah01_rpt013
python tools/arion_sim.py status | tee "$CHECK_DIR/profile-013.json"
```

**통과 기준:** 응답 `ok: true`, 선택 ID 변경, 최신 배터리 메시지 지속. 적용 직후 SOC는 초기 95% 근처이고 소비량은 다시 증가할 수 있다. 실제 원문 V와 운영 `power`의 프로파일 설정을 구분한다. 네 전압이 SOH 순서대로 반드시 정렬된다는 판정은 하지 않는다.

### 4-B. reset과 restore 구분

```bash
python tools/arion_sim.py reset
python tools/arion_sim.py status | tee "$CHECK_DIR/after-reset.json"
python tools/arion_sim.py restore
python tools/arion_sim.py status | tee "$CHECK_DIR/after-restore.json"
```

- `reset` 후에는 선택한 `cmu_vah01_rpt013`을 유지하면서 소비량/SOC/이상을 초기화한다.
- `restore` 후에는 이 run의 최초 선택인 `cmu_vah01_rpt000`으로 돌아간다.
- 이후의 `clear`는 모터 배율만 복구하며 소비한 배터리를 충전하지 않는다.

### 4-C. 7초 pause 후 재개

지상에서 진행한다. `sleep`은 관찰용 7초 대기이며 프로그램 설치/설정 변경이 아니다.

```bash
python tools/arion_sim.py pause
python tools/arion_sim.py status | tee "$CHECK_DIR/paused.json"
sleep 7
python tools/arion_sim.py resume
python tools/arion_sim.py status | tee "$CHECK_DIR/resumed.json"
```

**통과 기준:** paused true → false, 재개 뒤 배터리·ESC 수신/새 값 복귀, 오류 없음. pause 중 watch가 계속 표시되거나 PX4가 값을 반복 송신할 수 있다. 이것을 새 모델 계산·배터리 적분으로 해석하지 않는다.

## 5. 정상 비행과 네 모터 이상 주입

이 절은 실제 **가상 기체를 비행**시킨다. 터미널 B의 `watch`를 보면서 터미널 A에서 제어한다. 각 이상을 넣기 전 정상 구간을, 해제 뒤 복구 구간을 몇 초씩 기록한다. 배터리 상태를 보며 불필요하게 오래 호버하지 않는다.

### 5-A. 이륙·무주입 기준

```bash
python tools/arion_sim.py takeoff --height 3
python tools/arion_sim.py status | tee "$CHECK_DIR/normal-hover.json"
```

**통과 기준:** 정상 preflight 통과, 무장·공중 상태, 목표 고도 근처의 호버. 도달 판정에 여유가 있으므로 고도가 정확히 3.000m여야 한다는 조건은 없다. 거부되면 로그를 보고 중단하며 force-arm하지 않는다.

### 5-B. 모터 1부터 4까지 — 한 번에 하나

아래 한 세트를 실행하고 관찰 후 해제한다. **다른 모터로 넘어가기 전 `clear`를 빠뜨리지 않는다.** 아래 네 세트를 모두 수행하면 현재 네 슬롯의 주입/해제를 각각 확인할 수 있다.

```bash
python tools/arion_sim.py motor-fault 1 --current-scale 1.6 --rpm-scale 0.85
```

터미널 B에서 모터 1의 RPM/A와 팩 V/A 변화를 몇 초 확인한 다음:

```bash
python tools/arion_sim.py clear
```

복구 구간을 확인한 뒤 나머지 모터도 동일하게 한 세트씩 수행한다.

```bash
python tools/arion_sim.py motor-fault 2 --current-scale 1.6 --rpm-scale 0.85
```

모터 2의 변화 관찰 후 해제한다.

```bash
python tools/arion_sim.py clear
```

복구를 확인한 뒤 모터 3에 주입한다.

```bash
python tools/arion_sim.py motor-fault 3 --current-scale 1.6 --rpm-scale 0.85
```

모터 3의 변화 관찰 후 해제한다.

```bash
python tools/arion_sim.py clear
```

복구를 확인한 뒤 모터 4에 주입한다.

```bash
python tools/arion_sim.py motor-fault 4 --current-scale 1.6 --rpm-scale 0.85
```

모터 4의 변화 관찰 후 해제한다.

```bash
python tools/arion_sim.py clear
```

**통과 기준:** 지정 슬롯의 보고 RPM 감소·모터 전류 증가, 그에 따른 팩 부하/전압 변화, clear 뒤 배율 복구가 원문에 나타난다. 호버 부하도 변하므로 서로 다른 시각의 한 값으로 정확히 1.6/0.85 배라고 판정하지 않는다. 초기 정상·주입·복구의 여러 표본과 정답 이벤트를 함께 비교한다. 적용 ACK만으로 관측 성공 처리하지 않는다.

이것은 **텔레메트리/전기 부하 이상**이지 프로펠러 파손·추력 손실 시험이 아니다. 모델이 기울거나 추락하지 않는다고 주입 실패라고 판단하지 않는다.

### 5-C. 공중 초기화 거부 — 예상 실패 시험

이 한 명령은 **실패해야 통과**다. 전체 실행을 `set -e` 자동 스크립트로 묶거나, 모든 오류를 무조건 무시하는 방식으로 바꾸지 않는다.

```bash
python tools/arion_sim.py reset
```

**기대 결과:** nonzero 종료와 Disarm/먼저 착륙하라는 오류. 서비스는 계속 running이며 기존 비행·수신이 유지된다. 초기화가 실제로 적용되면 정상 통과가 아니므로 다음 시험을 계속하지 않는다.

### 5-D. 착륙·자동 Disarm

```bash
python tools/arion_sim.py clear
python tools/arion_sim.py land
python tools/arion_sim.py status | tee "$CHECK_DIR/landed.json"
```

**통과 기준:** `land` 응답의 `disarmed: true`, 최신 heartbeat 무장 bit 없음, landed_state=1. 지상 확인 전 profile/reset/world-reset/stop으로 넘어가지 않는다.

## 6. world-reset과 새 epoch의 후속 비행

이 시험은 단순 배터리 reset과 다르다. PX4와 sender가 재시작하며 짧은 수신 공백이 생긴다. Gazebo·수집기·서비스·run은 유지한다.

```bash
python tools/arion_sim.py status | tee "$CHECK_DIR/before-world-reset.json"
python tools/arion_sim.py world-reset | tee "$CHECK_DIR/world-reset.json"
python tools/arion_sim.py status | tee "$CHECK_DIR/after-world-reset.json"
```

**통과 기준:** 같은 `run_id`/`run_dir`, `px4_epoch` 증가, PX4/sender PID 변경, running·Disarmed 및 필수 메시지 수신 복귀. 각 타입의 saved count가 reset 전보다 증가했는지 확인한다. 이전 cache가 남아 있다는 이유만으로 복구됐다고 판정하지 않는다.

새 epoch에서 열화 단계의 이륙·호버·착륙도 따로 확인한다.

```bash
python tools/arion_sim.py profile cmu_vah01_rpt013
python tools/arion_sim.py takeoff --height 3
python tools/arion_sim.py status | tee "$CHECK_DIR/aged-hover.json"
```

짧게 관찰하고 착륙한다. 필요하면 5-B의 주입/해제를 재사용하되 조건과 관찰 구간을 따로 기록한다.

```bash
python tools/arion_sim.py land
python tools/arion_sim.py restore
```

**확인 범위를 구분한다:** world-reset 지상 복구와 후속 비행은 각각 별도 항목이다. 첫 비행 성공만으로 둘 다 완료 처리하지 않는다.

### 선택 확장: 모든 배터리×모터 조합

기본 전체 기능 확인 뒤 모든 조합을 비교하려면 아래 표를 사용한다. **이 표 전체가 이전에 실비행 검증됐다는 뜻은 아니다.** 각 프로파일을 지상에서 선택하고 정상 기준을 기록한 뒤, 한 모터씩 주입→관측→해제를 반복하고 착륙한다.

| 배터리 | 모터1 주입/해제 | 모터2 주입/해제 | 모터3 주입/해제 | 모터4 주입/해제 |
|---|---|---|---|---|
| `cmu_vah01_rpt000` | 미실행 | 미실행 | 미실행 | 미실행 |
| `cmu_vah01_rpt002` | 미실행 | 미실행 | 미실행 | 미실행 |
| `cmu_vah01_rpt006` | 미실행 | 미실행 | 미실행 | 미실행 |
| `cmu_vah01_rpt013` | 미실행 | 미실행 | 미실행 | 미실행 |

각 칸은 사용자가 이번에 수행한 결과와 run/구간으로 갱신한다. 같은 셀의 네 열화 단계는 독립된 네 실물 셀이 아니고, 이 행렬을 모두 실행해도 AI 성능 검증은 아니다. 한 run으로 이어 하면 profile/reset/epoch 경계를 반드시 남긴다. run을 분리하려면 착륙→stop→새 profile로 start한다.

## 7. 정상 종료와 다음 run

터미널 B의 watch는 Ctrl+C로 끝낸다. 터미널 A에서는 이미 지상·Disarmed이고 pause가 해제됐는지 확인한 뒤 종료한다.

```bash
python tools/arion_sim.py stop | tee "$CHECK_DIR/stop.json"
python tools/arion_sim.py status | tee "$CHECK_DIR/status-final.json"
```

**통과 기준:** `state: stopped`, `ok: true`, `errors: []`. `cleanup`의 소유 PX4/sender/Gazebo와 이전 epoch 종료 결과가 정상이고 `forced_kill: false`, `group_alive: false`, 반환 코드 0이어야 한다. `stop` 요청이 접수됐다는 출력만으로 완료 처리하지 않는다. GUI를 열었다면 그 종료 결과도 따로 확인한다.

종료 뒤 status의 `live_connection: false`는 저장된 마지막 상태를 읽었다는 뜻이다. stopped와 모순되지 않지만, 이 응답 자체가 현재 살아 있는 서비스 연결은 아니다.

새 run 시작/종료 반복도 확인하려면 다음을 별도 단계로 실행한다. 아래 시작 후 `status`를 보고 종료한다; 두 명령을 자동 연속 비행으로 만들지 않는다.

```bash
python tools/arion_sim.py start --profile cmu_vah01_rpt000 --background
python tools/arion_sim.py status | tee "$CHECK_DIR/restarted.json"
python tools/arion_sim.py stop
```

새 `run_id`·새 `run_dir`인지 확인한다. **앞서 저장한 `$RUN_DIR`은 첫 번째 시험의 원문을 가리키므로 다음 절의 확인에 그대로 사용한다.** 두 번째 run도 검사하려면 그 run_dir로 별도 읽는다.

## 8. 저장된 원문·종료 summary 대조

다음 Python은 **파일만 읽는다**. 네트워크·시뮬레이터를 시작하거나 파일을 변경하지 않는다. 종료한 첫 번째 run의 원문 전량을 세고 metadata source, summary, 필수 타입을 대조한다. 이 대조가 값의 물리 타당성이나 AI 정확도를 대신하지는 않는다.

```bash
python - "$RUN_DIR" <<'PY'
import json
import sys
from collections import Counter
from pathlib import Path

run = Path(sys.argv[1])
telemetry = run / "telemetry"
summary = json.loads((telemetry / "summary.json").read_text())
meta = json.loads((telemetry / "metadata.json").read_text())
service = json.loads((run / "service.summary.json").read_text())
cleanup = service["cleanup"]
paths = sorted(telemetry.glob("mavlink_raw*.jsonl"),
               key=lambda p: (p.name != "mavlink_raw.jsonl", p.name))
counts = Counter()
sources = set()
for path in paths:
    for line in path.open():
        row = json.loads(line)
        name = row["message_name"]
        counts["UNKNOWN" if name.startswith("UNKNOWN") else name] += 1
        sources.add((row["source_system"], row["source_component"]))
required = {"ESC_STATUS", "BATTERY_STATUS", "HEARTBEAT", "ATTITUDE",
            "LOCAL_POSITION_NED", "EXTENDED_SYS_STATE"}
checks = {
    "raw_files_exist": bool(paths),
    "counts_equal_summary": dict(counts) == summary["saved_by_type"],
    "total_equal_summary": sum(counts.values()) == summary["saved_total"],
    "all_six_present": required <= counts.keys(),
    "expected_source_only": sources == {(meta["source_system"], meta["source_component"])},
    "no_decode_error": summary["unknown_count"] == 0 and summary["bad_count"] == 0,
    "recorder_closed_cleanly": not summary["errors"] and not summary["worker_alive"],
    "service_stopped_cleanly": service["state"] == "stopped" and service["ok"] and not service["errors"],
    "all_owned_children_clean": {"px4", "sender", "gazebo"} <= cleanup.keys() and all(
        result.get("returncode") == 0 and result.get("forced_kill") is False
        and result.get("group_alive") is False and not result.get("error")
        for result in cleanup.values()),
}
print(json.dumps({"counts": dict(counts), "checks": checks}, indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
PY
```

- `checks`가 모두 `true`면 이 파일 정합성 확인은 통과다. 하나라도 `false`면 해당 raw/summary/metadata를 조사한다. 성공 건수를 과거의 2,615개에 고정하지 않는다.
- 원문·metadata·`groundtruth.events.jsonl*`·프로세스 로그·`service.summary.json`과 `$CHECK_DIR`를 함께 보관한다. 원본 로그를 덮어쓰지 않는다.
- 파일 회전은 파일 하나의 상한이지 전체 디스크 보관 상한이 아니다. 실행 중인 로그를 삭제하지 않는다.

## 9. GUI는 독립적인 선택 확인

3D 화면도 확인하려는 경우에만 실행 중인 같은 서비스에 다음을 사용한다.

```bash
python tools/arion_sim.py gui
```

응답은 GUI 프로세스 시작이지 화면 표시·종료 안정성의 합격 판정이 아니다. 화면 표시, 같은 기체의 움직임, GUI 종료, crash 보고를 따로 확인한다. headless 성공을 GUI 성공으로 대신하지 않는다. GUI에서 물리나 기체를 변경하지 않는다.

## 10. 오류 대응과 완료 기록

| 상황 | 다음 행동 |
|---|---|
| `start` 실패/IPC 소유자 불명 | 표시한 launcher 로그, 기존 서비스/instance 상태 확인. 중복 시작·무조건 삭제 금지 |
| `UNKNOWN_*`/`BAD_DATA` | decoder와 고정 PX4 XML, 송신원/수신 경로 확인. 정상값으로 채우지 않음 |
| `watch`에 None/빈 값 | 타입별 수신·freshness와 프로세스 로그 확인. heartbeat만으로 정상 판정하지 않음 |
| `takeoff` 거부 | `px4.log*`의 preflight 이유 확인. force-arm 금지 |
| 비행 중 다음 시험 실패 | 다른 fault/reset을 추가하지 않고, CLI가 응답하면 일반 `land`로 착륙 시도. pause 중 land는 재개 후 착륙을 시도함 |
| `land`/제어 무응답 | 새로운 start/전역 kill을 겹치지 않음. status·소유 PID·로그를 보존하고 해당 run은 실패로 기록한 뒤 소유 자원 회수를 개별 조사 |
| `stop` 후 failed/강제kill/잔류worker | 종료 성공 아님. `cleanup`, 수집기 error, `service.summary.json` 확인 |
| 의도적으로 요청한 공중 reset 거부 | 예상한 실패. 일반 명령의 실패와 구분 |

직접 실행한 결과를 다음 항목별로 **PASS / FAIL / 미실행**으로 남긴다. 과거 통과 기록을 이번 통과로 대신 채우지 않는다.

| 항목 | 결과 | run/로그/관측 구간 |
|---|---|---|
| Disarmed 시작·6메시지 수신·watch 두 형식 | 미실행 | |
| 4프로파일 선택과 reset/restore 차이 | 미실행 | |
| 7초 pause/resume·수신 복귀 | 미실행 | |
| 정상 기준 이륙·호버 | 미실행 | |
| 모터1 fault/clear | 미실행 | |
| 모터2 fault/clear | 미실행 | |
| 모터3 fault/clear | 미실행 | |
| 모터4 fault/clear | 미실행 | |
| 공중 reset 거부·착륙/Disarm | 미실행 | |
| world-reset의 새 epoch·같은 run·타입별 새 수신 | 미실행 | |
| 새 epoch/열화 profile의 후속 비행·착륙 | 미실행 | |
| 정상 stop·새 run 시작/종료 | 미실행 | |
| 원문/summary/source ID 일치·오류 없음 | 미실행 | |
| 전체 profile×motor 확장, GUI, 코드 회귀 | 미실행(별도 범위) | |

## 11. 코드 자동 회귀와 저수준 fixture

운영 통합 확인과 별도로 C++ 모델·제어, Python 서비스·collector, callback/worker/프로세스 회수를 확인하는 시험이다. **기능을 직접 써 보려는 첫 실행에서는 이 절을 건너뛰어도 된다.** 코드 회귀까지 확인하려는 경우에만 아래 단계를 추가한다. 운영 서비스가 이 시험들을 내부에서 호출하는 것은 아니다.

### 11-A. 자동 시험의 범위

2026-10-04 기준 추적 소스를 Git 목록·AST·CMake로 집계하면 **unittest 16모듈/126메서드**, **CTest 기본 3개·coupled 옵션 ON일 때 8개**, 별도 **compile-only 계약 1개**다. 이것은 소스의 시험 수이지 이번에 실행·통과한 수가 아니다. `subTest` 반복도 별도 메서드로 더하지 않았다.

주의할 점:

- `python -m unittest discover -s tests`는 **opt-in 없이 실제 Gazebo를 시작**한다. 단순한 정적 검사 명령이 아니다.
- 그 discover는 collector·CTest·독립 E2E·비행·CMU 운영 인수시험을 모두 실행하지 않는다.
- 실종료 회귀 4개는 `ARION_RUN_E2E_REGRESSION=1`, 실원본 발행 1개는 `CMU_AGING_RAW_DIR`가 있어야 실행된다. 따라서 기본 discover의 `OK (skipped=5)`를 전체 통과라고 적지 않는다.
- collector는 아래의 **정확한 파일 패턴**으로 선택한다. 로컬에 남아 있을 수 있는 ignored `test_collector.py`는 현재 제외된 이전 계약 시험이므로 `test*.py`로 함께 수집하지 않는다.

### 11-B. 경로와 시험 전용 빌드 준비

수동 운영 시험을 정상 종료한 뒤 같은 Bash 터미널에서 진행한다. 기존 `.venv`, CMake/Ninja/C++17, Gazebo Harmonic 개발 패키지(`gz-sim8`, `gz-transport13`, `gz-plugin2`, `gz-msgs10`), `pymavlink`와 `patch`가 필요하다. 일부 bridge 시험은 ASan/UBSan·실제 pthread/PTY를 쓴다. `pytest` 설치는 필요하지 않다.

다음은 기존 설정과 overlay manifest에서 **경로만 읽고**, 이번 결과용 새 폴더를 만든다. 다른 설정을 쓴 경우 Python 코드의 설정 파일명만 해당 파일로 바꾼다. 설정/manifest가 없으면 임의 경로로 진행하지 말고 [서비스 준비 8절](cmu-service-guide.md#8-다른-팀원의-pc에서-준비하는-순서)을 따른다.

```bash
export SIM_DIR="$(pwd -P)"
config_env=$(python -B - <<'PY'
import json, shlex
from pathlib import Path
repo = Path.cwd()
cfg = json.loads((repo / 'config/service.local.json').read_text())
paths = {}
for name, key in [('PX4_DIR', 'px4_source'), ('PX4_BINARY', 'px4_binary'),
                  ('DECODER', 'decoder'), ('IPC_DIR', 'ipc_dir')]:
    p = Path(cfg[key]).expanduser()
    paths[name] = (p if p.is_absolute() else repo / p).resolve()
manifest = paths['PX4_BINARY'].parent.parent / 'manifest.json'
paths['BASE_BUILD'] = Path(json.loads(manifest.read_text())['base_build'])
for name, path in paths.items():
    print('export ' + name + '=' + shlex.quote(str(path)))
PY
) && eval "$config_env"
```

위 경로 읽기가 성공한 경우에만 다음으로 진행한다. 새 결과 폴더와 환경변수는 현재 터미널의 시험에만 사용한다. 최적화 옵션은 safety assertion을 제거할 수 있으므로 사용하지 않는다.

```bash
mkdir -p ../artifacts
export TEST_OUT="$(mktemp -d "$(cd .. && pwd)/artifacts/code-tests-XXXXXXXX")"
export CPP_BUILD="$TEST_OUT/cpp"
mkdir -p "$TEST_OUT/tmp" "$TEST_OUT/motor" "$TEST_OUT/overlay" "$TEST_OUT/battery"
export PYTHONDONTWRITEBYTECODE=1 TMPDIR="$TEST_OUT/tmp"
unset PYTHONOPTIMIZE ARION_RUN_E2E_REGRESSION CMU_AGING_RAW_DIR
export ARION_TEST_ROOT="$TEST_OUT/motor" ARION_TEST_ARTIFACT_ROOT="$TEST_OUT/motor"
export ARION_OVERLAY_TEST_ROOT="$TEST_OUT/overlay" ARION_BATTERY_TEST_ROOT="$TEST_OUT/battery"
export ARION_PX4_SOURCE="$PX4_DIR"
export PYTHONPATH="$SIM_DIR/tests${PYTHONPATH:+:$PYTHONPATH}"
```

`PX4_BINARY`는 기존 **overlay**이고, `BASE_BUILD`는 그 생성 헤더·라이브러리를 제공한 기존 기본 빌드다. 둘 다 보호 입력이다. 아래는 그 입력으로 **simulation의 별도 시험 빌드만** 만들며 PX4를 재빌드하거나 vendor에 패치를 다시 적용하지 않는다.

```bash
test -f "$BASE_BUILD/mavlink/common/mavlink.h" &&
cmake -S "$SIM_DIR" -B "$CPP_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DARION_BUILD_BATTERY_COMPONENTS=ON \
  -DARION_BUILD_COUPLED_POWER=ON \
  -DARION_MAVLINK_INCLUDE_DIR="$BASE_BUILD/mavlink" &&
cmake --build "$CPP_BUILD" --parallel 2 &&
ctest --test-dir "$CPP_BUILD" --output-on-failure
```

**통과 기준:** 전체 build 성공, CTest 8개 통과. `battery_contract_compile_test`는 `static_assert`를 컴파일하는 object target이므로 별도 실행 결과/CTest 항목은 없다. 운영 서비스 가이드의 세 target만 빌드한 폴더에는 FixedMotor/FixedBattery/observer·시험 실행파일이 없을 수 있다. 이 단계는 target을 제한하지 않아 함께 만든다.

CTest의 이름은 `motor_frame_mapping`, `motor_frame_nonfinite`, `motor_frame_rpm_range`, `coupled_power_model`, `cmu_power_model`, `power_control_mailbox`, `power_control_cli_contract`, `power_control_sdf_callback`이다. CTest 성공만으로 실제 Gazebo/PX4 연결이나 비행을 검증한 것은 아니다.

### 11-C. 현재 시험 코드의 고정 경로

일부 fixture는 설정 파일/환경변수로 입력·출력을 전부 옮길 수 없다. `../hermes`는 **현재 코드에 고정된 일반 폴더명이며 Hermes 설치·실행을 요구하지 않는다.** 이 절 때문에 기존 환경을 이동하거나 제품 코드를 수정하지 않는다.

| 고정 경로 또는 조건 | 해당 시험과 의미 |
|---|---|
| `simulation/PX4-Autopilot/` | bridge 3모듈·실종료 회귀의 고정 source. `ARION_PX4_SOURCE`는 overlay 단위 시험만 바꿈 |
| `../hermes/motor-fixed/build/px4/mavlink/` | `test_continuous_io`가 읽는 생성 C 헤더 루트 |
| `../hermes/mavlink/px4_messages.py` | collector 시험 및 자식 CLI의 기본 decoder. `ARION_DECODER`로 바뀌지 않음 |
| `../hermes/cmu-service/io/` | sender/collector의 합성 시험 결과 |
| `../hermes/cmu-service/service-tests/` | service/runtime 파일 시험 결과. 일부는 import 때 폴더를 만듦 |
| PX4 instance **31** | 실종료 회귀에 고정. `ARION_TEST_ROOT`로 instance가 바뀌지 않음 |

고정 헤더/decoder가 이미 있는 PC는 그대로 쓰되 현재 고정 PX4 정의로 만든 것인지 확인한다. **새 PC에서 해당 경로가 없는 경우에만**, 준비해 둔 동일 버전 생성물에 아래처럼 링크할 수 있다. `ln -sT`에는 덮어쓰기 옵션이 없으므로 기존 경로가 있으면 실패한다. 기존 경로를 지우거나 `-f`로 강제하지 않는다. 두 대상 중 하나만 없다면 그 명령만 실행한다. 원본 생성물과 서비스 설정은 변경하지 않는다.

```bash
mkdir -p ../hermes/motor-fixed/build/px4 ../hermes/mavlink
ln -sT "$BASE_BUILD/mavlink" ../hermes/motor-fixed/build/px4/mavlink
ln -sT "$DECODER" ../hermes/mavlink/px4_messages.py
```

`ARION_TEST_ROOT`가 모든 결과를 `$TEST_OUT`에 모으지는 않는다. 위 고정 출력도 함께 보관한다. decoder는 실행되는 Python 코드이므로 신뢰하는 고정 XML로 직접 생성한 파일만 연결한다.

### 11-D. 시뮬레이터 없는 Python 회귀

다음은 실제 PX4/Gazebo **서버 없이** 실행하는 선택이다. C++ 컴파일, 합성 UDP, UNIX socket, PTY, 로컬 하위 프로세스는 사용한다. 앞의 두 opt-in 변수를 unset한 상태에서 실행한다.

```bash
python -B -m unittest -v \
  test_bridge_lifecycle test_bridge_worker test_bridge_shutdown \
  test_cleanup_regressions test_e2e_safety.CleanupTest test_e2e_safety.OptimizationTest \
  test_px4_console test_coupled_server_config test_power_flight_analysis \
  test_cmu_aging test_cmu_evtol_profiles test_continuous_io \
  test_runtime_files test_sim_service test_sitl_overlay
```

collector는 시작 폴더와 파일을 따로 지정한다.

```bash
python -B -m unittest discover -s tools/mavlink_collector \
  -p test_continuous_collector.py -v
```

**통과 기준:** outer unittest 종료 0, failure/error 없음. 첫 명령의 실원본 1개는 기본 skip이며 미실행으로 기록한다. 실종료 회귀 클래스는 이 명령에 선택되지 않았다. `ARION_OVERLAY_TEST_ROOT` 누락은 skip이 아니라 오류다.

현재 unittest 모듈 목록은 다음과 같다. collector 외의 파일은 모두 `tests/` 아래다.

| 모듈 | 메서드 수 | 범위 |
|---|---:|---|
| `test_bridge_lifecycle` | 6 | C++ callback·시작/종료 소유권 |
| `test_bridge_worker` | 2 | 실제 pthread의 늦은 entry/owner |
| `test_bridge_shutdown` | 9 | 고정 시계·종료 deadline·native 회수 |
| `test_cleanup_regressions` | 14 | 합성 프로세스·socket·PTY·FD 오류 회귀 |
| `test_cmu_aging` | 7 | 합성 6 + opt-in 전체 원본 1 |
| `test_cmu_evtol_profiles` | 32 | 합성 6-profile 번들·무결성·split·실패 안전성 |
| `test_continuous_io` | 3 | sender C++/합성 UDP·시간 공백·종료 |
| `test_coupled_server_config` | 1 | 필수 센서 system XML 검사; 센서 실행 아님 |
| `test_e2e_safety` | 10 | 합성 안전성 6 + opt-in 실제 종료회귀 4 |
| `test_gazebo_motor` | 2 | 실제 Gazebo의 fixed 전달/주기; 11-E에서 선택 |
| `test_power_flight_analysis` | 2 | 합성 데이터의 판정 함수; 실제 비행 아님 |
| `test_px4_console` | 2 | Python PTY fixture; 실제 PX4 아님 |
| `test_runtime_files` | 4 | 합성 ELF/IPC·소유자 불명 거부 |
| `test_sim_service` | 16 | 서비스 정책·API·파일·mock world-reset |
| `test_sitl_overlay` | 7 | 복사 소스 변환/packaging·좁은 C++ seam |
| `tools/mavlink_collector/test_continuous_collector` | 9 | 합성 UDP·원문·오류·시그널 |

### 11-E. 실제 Gazebo/PX4 시험 — 순차 선택

각 코드 블록은 **별도 선택 단계**다. 운영 서비스와 같은 instance를 쓰거나 시험끼리 병렬 실행하지 않는다. 모든 하위 프로세스가 종료됐는지 확인한 뒤 다음 블록으로 간다. 기존 motor/Gazebo runner는 운영 서비스와 같은 HOME/XDG·IPC 격리 검사를 제공하지 않으므로 `TMPDIR`만 바꿔 완전히 격리됐다고 보지 않는다.

**① Gazebo-only:** FixedMotor Plugin의 네 슬롯과 발행 주기를 실제 server에서 확인한다. PX4나 비행은 없다.

```bash
ARION_PLUGIN_BUILD="$CPP_BUILD" \
  python -B -m unittest discover -s tests -p test_gazebo_motor.py -v
```

**② 정상 fixed motor E2E:** PX4 기본값 → bridge 고정값 → 입력 부재 → 기본 복귀를 확인한다. Disarmed이며 비행하지 않는다. instance 31이 다른 실행에서 사용 중이면 진행하지 않는다.

```bash
python -B tests/run_motor_fixed_e2e.py \
  --px4-source "$PX4_DIR" --px4-binary "$PX4_BINARY" \
  --plugin-build "$CPP_BUILD" --decoder "$DECODER" \
  --artifacts "$TEST_OUT/motor-live" --instance 31
```

**③ 실종료 오류 회귀:** 네 시험이 각각 실제 Disarmed E2E 뒤 종료 오류를 주입한다. 고정 instance 31을 사용한다. 여기서만 세 입력 환경변수를 직접 읽는다.

```bash
ARION_RUN_E2E_REGRESSION=1 ARION_PX4_BINARY="$PX4_BINARY" \
ARION_PLUGIN_BUILD="$CPP_BUILD" ARION_DECODER="$DECODER" \
  python -B -m unittest -v test_e2e_safety.RealShutdownRegressionTest
```

이 회귀는 **하위 report의 `passed: false`와 하위 실패 종료가 기대값**이다. outer unittest가 주입 오류·관측·소유 그룹 종료를 정확히 확인하고 성공했는지 본다. 정상 E2E의 통과 판정과 혼동하지 않는다.

**④ 고정 배터리 왕복:** FixedMotor/FixedBattery, sender, observer, overlay를 사용하는 별도 Disarmed 시험이다. 기본 운영 instance와 같은 35를 쓰므로 운영 서비스를 먼저 정상 종료한다.

```bash
args=(--px4-source "$PX4_DIR" --px4-binary "$PX4_BINARY"
      --plugin-build "$CPP_BUILD" --decoder "$DECODER"
      --artifacts "$TEST_OUT/coupled-live" --ipc-dir "$IPC_DIR" --instance 35)
python -B tools/run_coupled_flight.py "${args[@]}" --mode fixed
```

**⑤ 기존 연동 모델 비행:** 바로 앞에서 정의한 `args`를 사용한다. 이 명령은 실제 가상 기체를 이륙·호버·착륙시키므로 별도로 선택한다. `--fly`를 빼거나 정상 preflight를 우회하지 않는다.

```bash
python -B tools/run_coupled_flight.py "${args[@]}" --mode flight --fly --hover-seconds 8
```

④/⑤에는 `arion_power_observer`도 필요하다. ⑤는 기존 assumed-linear reference 모델·전용 수신기를 쓰며 **CMU 네 프로파일/운영 공용 수집기 인수시험이 아니다.** 그것은 앞의 2~10절로 확인한다. `tests/battery_fixture.sdf`도 보존된 보조 fixture일 뿐 위 suite가 자동 실행하는 별도 시험이 아니다.

정상 E2E/왕복/비행은 **exit 0 + `report.passed: true` + 정상 cleanup**을 함께 확인한다. `E2E_EVIDENCE`/`RUN_EVIDENCE`는 결과 폴더 위치이지 성공 인증이 아니다. 중단·종료 예외로 보고서가 없거나 불완전하면 실패/미완료로 남기고 소유 PID·정확한 instance 자원을 조사한다. 모든 예외 경로에서 자동 회수가 보장된다고 가정하지 않는다.

### 11-F. 실제 CMU 원본과 보존 번들 — 별도 선택

전체 VAH01 원본을 가진 경우에만 경로를 바꿔 실행한다. 이 시험은 원본 폴더의 **부모 아래 임시 catalog**를 생성하므로 부모 쓰기 권한이 필요하다. 원본은 보존한다.

```bash
CMU_AGING_RAW_DIR="$HOME/px4-workspace/battery-data/cmu-evtol-v3" \
  python -B -m unittest -v test_cmu_aging.PublicationTests
```

`VAH01.csv`, `VAH01_impedance.csv`, `README.txt`, `cmu-v3.json`, `article.xml`의 고정 크기/checksum을 요구한다. 잘린 prefix나 합성 CSV로 대체하지 않는다. 원본 준비 근거는 [CMU runtime 프로파일](cmu-runtime-profiles.md)을 따른다. `ARION_CMU_SOURCE`/`ARION_CMU_VAH01_SOURCE`라는 변수는 현재 이 시험의 설정이 아니다.

다음은 저장소에 보존한 **초기 6개 정상 번들** 검사이며 전체 원본 발행/CMU 실비행/AI 성능 시험과 다르다.

```bash
python -B tools/battery_profiles/cmu_evtol.py verify \
  --bundle profiles/battery/cmu_evtol_normal_soh100
```

### 11-G. 자동 시험 완료표

앞의 단계로 확인했다면 단지 discover 결과를 다시 얻으려고 같은 시험을 반복할 필요는 없다. 추적 Python 전체 파일 선택을 한 번에 하려는 경우에만, 모든 준비가 끝난 뒤 다음을 쓸 수 있다. **첫 명령은 실제 Gazebo를 실행하고, 기본 상태에서는 두 opt-in의 5개 메서드를 skip한다.** collector는 여전히 별도다.

```bash
ARION_PLUGIN_BUILD="$CPP_BUILD" \
  python -B -m unittest discover -s tests -p 'test_*.py' -v
python -B -m unittest discover -s tools/mavlink_collector \
  -p test_continuous_collector.py -v
```

| 범위 | 결과 | 근거 |
|---|---|---|
| 전체 C++ build·CTest 8개·compile-only 계약 | 미실행 | build/CTest 출력 |
| simulator 없는 Python 선택·collector | 미실행 | unittest 결과·skip 수·시험 폴더 |
| Gazebo-only·정상 fixed motor E2E | 미실행 | probe·raw·report·cleanup |
| 실제 종료 음성 회귀 4개 | 미실행 | outer unittest 및 예상한 하위 실패 |
| 고정 배터리 왕복·기존 모델 비행 | 미실행 | 각 run의 report·raw·cleanup |
| CMU 실제 원본 발행·초기 번들 verify | 미실행 | 각기 다른 입력/출력과 결과 |
| CMU 운영 인수시험 | 미실행 | 10절 완료표 |

skip, 미선택, 실패, 통과를 구분한다. sanitizer 오류나 소유 자원 잔류를 단순한 “예상 실패”로 덮지 않는다. 시험의 종료 코드만으로 GUI 안정성·장기 내구성·AI 정확도까지 통과 처리하지 않는다.

자동 시험의 소스는 [CMakeLists.txt](../CMakeLists.txt), [tests](../tests), [collector 시험](../tools/mavlink_collector/test_continuous_collector.py)에 있다. 고정값 시험의 상세 배경은 [motor guide](motor-fixed-run-guide.md), [battery guide](battery-fixed-run-guide.md), [coupled flight guide](coupled-flight-run-guide.md)를 참고한다.
