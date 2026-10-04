# CMU 배터리·모터 이상 시뮬레이션 사용 가이드

이 문서는 Ubuntu 터미널에서 CMU 기반 배터리를 선택하고, 비행 중 모터 텔레메트리 이상을 넣고, PX4 출력 데이터를 계속 기록하는 방법을 설명한다. 기본 동작은 **Disarmed 상태로 시작하는 로컬 SITL**이다. 시작만으로 이륙하거나 실제 비행제어 보드에 연결하지 않는다.

**상시 운영**은 사용자가 종료할 때까지 실행 시간을 제한하지 않는다는 뜻이다. 24/7 내구시험 완료, 무한 비행·디스크, 장애 후 자동 복구, OS 부팅 자동 실행 또는 systemd 설치를 뜻하지 않는다. `world-reset`은 PX4/sender를 재시작하므로 짧은 수신 공백이 있다. 실제 확인한 프로파일 선택·reset·두 차례 비행의 범위는 9절에 구분해 기록했다.

## 1. 각 프로그램이 하는 일

| 구성 | 역할 |
|---|---|
| PX4 SITL | PC에서 실행되는 비행제어기. Gazebo 센서로 자세·위치를 추정하고 이륙·호버·착륙을 제어한다. |
| Gazebo Harmonic | x500의 기체 물리, 센서, 회전자 joint 움직임을 계산한다. 기본 서비스는 3D 창 없이 실행한다. |
| `ArionCoupledPowerPlugin` / `CoupledPowerState` | joint RPM에서 모터별 전류를 근사하고, 그 부하로 선택한 CMU 배터리의 전류·전압·SOC·소비량을 계산한다. 추진 물리를 교체하지 않는다. |
| `arion_motor_bridge` | Plugin의 네 모터 상태를 PX4 내부 uORB `esc_status`에 전달하는 외부 모듈이다. |
| `arion_battery_sender` | Plugin의 배터리 상태를 표준 MAVLink `BATTERY_STATUS`로 PX4의 외부 배터리 입력에 보낸다. |
| 공용 MAVLink 수집기 | PX4가 출력하는 메시지를 **수신·저장만** 한다. 메시지 요청, heartbeat, 비행 명령을 보내지 않는다. |
| `tools/arion_sim.py` | 위 프로세스의 시작·종료, 상태 확인, 프로파일 선택, 이상 주입과 소유한 시뮬레이터의 비행 제어를 묶은 운영 CLI다. |
| Python `.venv`와 생성 디코더 | CLI·수집기를 실행하고 해당 PX4의 MAVLink 메시지 정의를 해석한다. 디코더가 센서 데이터를 만들어 내지는 않는다. |
| QGroundControl(QGC) | 사람이 사용하는 지상관제 프로그램이다. 이 서비스는 자체 로컬 통신 링크를 구성하므로 QGC 자동 연결을 가정하지 않으며 필수 구성도 아니다. |

아래 명령은 모두 **Ubuntu 셸**에 입력한다. PX4의 `pxh>`에 입력하는 명령이 아니다. 별도 에이전트, ROS 2 또는 실제 기체가 필요하지 않다.

## 2. 데이터 경로와 정답의 분리

| 데이터 | 전달 경로 |
|---|---|
| 모터 관측 | Gazebo joint → Plugin → Gazebo Transport → `arion_motor_bridge` → uORB `esc_status` → PX4 MAVLink → 수집기 |
| 배터리 관측 | 모터 전류 합과 보조 부하 → CMU 기반 모델 → sender → PX4 외부 배터리 수신부 → uORB `battery_status` → PX4 MAVLink → 수집기 |
| 제어 문맥 | PX4 기존 제어 목표·구동 출력 → `POSITION_TARGET_LOCAL_NED`, `ATTITUDE_TARGET`, `ACTUATOR_OUTPUT_STATUS` → 수집기. 조종 명령 송신 기능이 아님 |
| 사용자 제어 | CLI → 같은 사용자 전용 UNIX socket → 서비스가 소유한 PX4 콘솔 또는 Gazebo 제어 서비스 |
| 시나리오 정답 | 프로파일·용량·저항·주입 배율·제어 시점 → 별도 실행 metadata와 `groundtruth.events.jsonl*` |

AI 입력의 출발점은 **PX4에서 받은 MAVLink 관측 필드**다. simulator의 참 용량·저항, 상대 SOH, RPT ID, 주입 배율·시작/종료 정답을 센서 특징에 합치지 않는다. `status`는 이러한 운영 정보를 포함하므로 전체 응답을 AI 입력으로 사용하지 않는다. Gazebo 상태를 직접 AI에 보내 PX4 경로를 우회하는 구성도 아니다.

## 3. 이미 준비된 PC에서 시작

현재 준비된 PC에는 `config/service.local.json`이 생성되어 있다. 재설치하거나 `init`을 반복하지 않는다. 아래 `~/px4-workspace/simulation`은 **팀원용 예시 위치**다. 기존 설치 위치가 다르면 첫 `cd`만 본인의 `simulation` 저장소 위치로 바꾸며, 이 문서 때문에 환경을 옮기지 않는다. 새 터미널에서도 같은 저장소와 `.venv`를 사용한다.

먼저 선택 목록을 보고 서비스를 시작한다. 이 단계는 시뮬레이터와 기록 파일을 만들지만 이륙하지 않는다.

```bash
cd ~/px4-workspace/simulation
source .venv/bin/activate
python tools/arion_sim.py profiles
python tools/arion_sim.py start --background
python tools/arion_sim.py status
```

| 명령·출력 | 읽는 방법 |
|---|---|
| `profiles` | 네 프로파일의 실제 ID, 상대 SOH, 팩 용량·저항을 출력한다. |
| `start --background` | 터미널과 분리해 실행한다. 준비가 실패하면 오류와 시작 로그 위치를 반환한다. 같은 설정으로 중복 시작하지 않는다. |
| `status`의 `state` | `running`이면 서비스 운영 중이다. `failed`, `unreachable`은 정상 실행으로 해석하지 않는다. |
| `run_dir` | 이번 실행의 기록 폴더다. 설정에 따라 달라지므로 고정 경로를 추측하지 않는다. |
| `selected_profile`, `power` | 운영자용 선택 상태와 마지막 제어 read-back이다. AI 센서 입력이 아니다. |
| `px4_epoch` | 같은 run 안에서 PX4가 시작된 세대다. 처음은 0, `world-reset` 후 증가하며 시계 경계를 구분한다. |
| `telemetry.messages`, `recorder.latest_age_s` | 타입별 최신 수신 필드와 수신 나이를 확인한다. `recorder.healthy` 하나만으로 배터리와 ESC가 모두 정상이라고 판정하지 않는다. |
| `errors`, `cleanup` | 실행 오류와 종료 정리 결과다. 오류가 있으면 성공으로 취급하지 않는다. |

서비스가 꺼진 뒤 `status`는 마지막 저장 상태를 보여 줄 수 있다. 이때의 `live_connection: false`는 살아 있는 서비스에 접속했다는 뜻이 아니다.

전경 실행은 `start`만 사용한다. 그 터미널의 Ctrl+C는 **서비스 종료 요청**이므로 비행 중에는 종료 수단으로 쓰지 말고 먼저 착륙한다. 아래 `watch`의 Ctrl+C는 표시만 끝낸다.

### 실시간 값 보기

```bash
python tools/arion_sim.py watch
```

표시는 기본 1초 간격이다. `V`는 팩 전압 V, `A`는 팩 전류 A, `SOC`는 잔량 %, `mAh`는 누적 소비량, `RPM`과 `ESC_A`는 네 모터의 보고 RPM·전류 A다. 배열은 모터 1→4 순서이며 CLI 모터 1은 내부 슬롯 0에 해당한다. SOC와 상대 SOH는 다른 값이다.

수신값이 없거나 해당 타입을 3초 넘게 받지 못하면 최신 필드는 비고 일반 표시에는 `None`이 나올 수 있다. **수신 freshness와 센서 freshness는 다르다.** Gazebo가 pause되어도 PX4가 이전 상태를 반복 송신할 수 있어 값이 계속 표시될 수 있다. 표시 갱신이나 최근 수신만으로 새 센서 측정이라고 판단하지 않는다.

프로그램이 읽을 수 있는 최신값 JSONL 표시도 가능하다.

```bash
python tools/arion_sim.py watch --json
```

이는 주기적으로 조회한 현재값이며 동일 메시지가 반복될 수 있다. 수집 대상 메시지의 수신 기록을 대신하지 않는다. 해당 메시지의 실제 수신 순서·시각을 보존한 데이터는 `run_dir`의 `telemetry/mavlink_raw*.jsonl`을 사용한다. 원문 `BATTERY_STATUS` 전압은 mV, 전류는 cA이며 일반 `watch`만 V/A로 표시하므로 두 형식을 혼동하지 않는다.

`NaN`, 양/음의 무한대는 원문 JSONL과 같은 `{"$float": "nan"}`, `{"$float": "inf"}`, `{"$float": "-inf"}` 태그로 표시한다. 0이나 정상 측정값으로 치환하지 않는다. `status`의 텔레메트리와 `watch --json`에도 같은 표현을 사용하며, 수집기 내부의 원래 수치와 유한 sentinel은 보존한다.

`watch --json`과 `status`에는 기존 6종에 더해 제어 문맥 3종의 최신값이 공개된다. 요청 목표는 위치/속도 목표 10Hz, 자세/추력 목표 10Hz, actuator 출력 20Hz이며 수신 보장 주기가 아니다. 일반 `watch`의 표시 형식은 그대로다. **2026-10-04 실제 로컬 SITL에서 새 3종의 수신·저장·두 조회 경로를 확인**했다. 상세 run·건수·한계는 [AI 계약](ai-telemetry-contract.md) 2-A절에 있다. `status`는 `telemetry.messages`, `watch --json`은 최상위 `messages`로 읽는다. 지상이나 해당 제어기가 비활성인 모드에서는 `{}`일 수 있어 지상 시작 필수 조건으로 사용하지 않는다. 부하 AI에 사용하기 전 단위·mask·모터 대응과 source 시각을 확인한다. 기존 서비스 실행 중 코드를 바꿨다면 먼저 착륙·정상 종료하고 다시 시작한다.

### 목표 3m 이륙·호버·착륙

이륙은 별도 명령이다. 정상 preflight 검사를 통과한 뒤 목표 상대고도 근처에서 호버하는지 기다린다. 지원 고도 범위는 1~10m다. 도달 판정에는 고도 여유와 수직 속도 조건이 있으므로 명령 성공을 정확히 3.000m에 정지했다는 뜻으로 읽지 않는다.

```bash
python tools/arion_sim.py takeoff --height 3
```

비행 중에도 기록은 계속된다. 관찰을 마치면 다음 명령으로 착륙과 자동 Disarm을 기다린다.

```bash
python tools/arion_sim.py land
```

이륙이 거부되면 `status`와 `run_dir`의 `px4.log*`를 확인한다. force-arm으로 우회하지 않는다. 배터리 잔량과 전압을 보며 운용하고, 무기한 수집을 무기한 비행으로 해석하지 않는다.

로컬 데스크톱에서 같은 시뮬레이션의 3D 창을 열려면 다음을 선택적으로 사용한다. 기본 수집에는 필요하지 않으며 명령 성공은 GUI 프로세스 시작이지 화면 표시 검증까지 의미하지 않는다.

```bash
python tools/arion_sim.py gui
```

## 4. 네 배터리 프로파일의 의미와 선택

기준 자료는 Alexander Bills et al., CMU **eVTOL Battery Dataset version 3**, DOI `10.1184/R1/14226830.v3`다. 같은 셀 **VAH01**의 서로 다른 실제 RPT(기준 성능 시험)에서 용량을 구하고, 첫 완전 RPT 대비 상대 용량으로 단계를 선택했다. 임의로 용량을 95/90/85%로 줄인 네 합성 측정값이 아니다.

아래는 [런타임 카탈로그](../profiles/battery/cmu_runtime/catalog.json)와 [프로파일 산정 문서](cmu-runtime-profiles.md)의 값이다. 표의 팩 Ah·Ω는 표시 자릿수로 반올림했으며 원래 정밀도는 카탈로그에 있다.

| ID | 상대 SOH % | 가정 4S2P 팩 용량 Ah | 추정치를 환산한 팩 저항 Ω |
|---|---:|---:|---:|
| `cmu_vah01_rpt000` | 100.000000 | 6.178676150 | 0.046155310 |
| `cmu_vah01_rpt002` | 95.027445 | 5.871438083 | 0.045534175 |
| `cmu_vah01_rpt006` | 90.422638 | 5.586921940 | 0.050836011 |
| `cmu_vah01_rpt013` | 84.812594 | 5.240295514 | 0.058362409 |

- **100%는 초기 상대 기준**이며 절대 무열화 인증이 아니다. 네 단계 모두 초기 SOC는 가정값 0.95다. `watch`의 SOC가 95%라고 상대 SOH도 95%인 것은 아니다.
- **4S2P는 가상 팩 설계 가정**이다. 단일 셀의 특성을 동일한 셀들에 적용해 팩 용량·저항·전압으로 확대한 것이며 실제 x500 팩 실측 보정이 아니다.
- CSV는 **SOC→셀 OCV 등가곡선 조회표**다. 현재 모터 부하와 시뮬레이션 경과시간으로 소비량을 적분하고 전압 강하를 계산한다. 원본의 시간축 V/I를 그대로 재생하지 않는다.
- OCV는 저율 방전 전압에 추정 I·R을 보정한 **등가 근사**이지 평형 OCV 실측이 아니다. 저항은 두 전류 step의 **유효 DC 1초 추정치**로, 고부하·모든 SOC·온도에서 일정한 실제 내부저항을 인증하지 않는다.
- **95.027445% 단계의 저항은 초기보다 조금 낮은 값을 그대로 보존했다.** SOH가 낮아질수록 저항이 반드시 커지도록 보정하지 않았다. 전압도 모든 부하에서 단순한 열화 순서로 정렬될 것이라고 가정하지 않는다.

상세 원본 행·체크섬·산정법과 한계는 [프로파일 문서](cmu-runtime-profiles.md), [provenance.json](../profiles/battery/cmu_runtime/provenance.json)에 있다.

### 실행 중 선택

`profile`은 **착륙·Disarmed 상태에서만** 사용한다. 선택한 프로파일의 용량·저항·곡선을 적용하고 초기 SOC·소비량으로 되돌리며 모든 모터 이상도 해제한다. 비행 중 배터리 교체 명령이 아니다.

```bash
python tools/arion_sim.py profile cmu_vah01_rpt002
python tools/arion_sim.py status
```

지상에서도 보조 부하를 계산하므로 초기화 후 시간이 흐르면 소비량이 다시 증가할 수 있다. 명령의 적용 결과와 이후 수신값을 구분해 읽는다.

서비스가 꺼진 상태에서 특정 프로파일로 시작할 수도 있다.

```bash
python tools/arion_sim.py start --profile cmu_vah01_rpt006 --background
```

`--profile` 생략 시 `cmu_vah01_rpt000`이 기본이다. 나중에 `restore`를 실행하면 **해당 run의 최초 선택**으로 돌아간다. 위 예시로 시작했다면 정상 기준이 아니라 `cmu_vah01_rpt006`으로 돌아간다.

### 초기 정상 번들·NASA와의 구분

`profiles/battery/cmu_evtol_normal_soh100/`의 **초기 정상 6개 프로파일**은 보존된 오프라인 원본 추적·재생 자료다. VAH01/VAH17/VAH27의 초기 RPT와 첫 임무 자료이며, 위 네 런타임 프로파일과 별개다. 이를 삭제하거나 같은 학습 분할로 혼합하지 않는다. 사용법은 [초기 정상 프로파일 안내](cmu-evtol-normal-profile-guide.md)를 따른다.

NASA 구성은 활성 제품·실행 경로에서 제외되었고 새 서비스의 선택 목록에 없다. 과거 원본과 근거는 보존하지만 실행 의존성이 아니다. [NASA 폐기 안내](nasa-retirement.md)를 참고한다.

## 5. 모터 텔레메트리 이상 주입·해제

모터 번호는 **1~4**다. 이륙 후 호버 중 다음 명령은 모터 1의 추정 전류를 1.6배, **보고 RPM**을 0.85배로 바꾼다.

```bash
python tools/arion_sim.py motor-fault 1 --current-scale 1.6 --rpm-scale 0.85
```

`watch`에서 해당 모터와 배터리 값을 관찰한 뒤 해제한다.

```bash
python tools/arion_sim.py clear
```

다른 모터를 독립적으로 비교하려면 해제 후 번호를 바꾼다.

```bash
python tools/arion_sim.py motor-fault 4 --current-scale 1.6 --rpm-scale 0.85
```

관찰 후 다시 `clear`하고 `land`한다. `clear` 없이 다른 모터에 주입하면 이전 모터의 이상도 남으므로, 한 모터씩 비교하려면 중간 해제를 생략하지 않는다.

| 항목 | 의미와 한계 |
|---|---|
| `--current-scale` | 0~3. joint RPM으로 구한 전류에 적용한다. 바뀐 모터 전류는 배터리 전류·전압 강하·소비량에도 반영된다. |
| `--rpm-scale` | 0~1. 출력되는 RPM에만 적용한다. 전류는 이 축소된 보고 RPM을 다시 조회한 값이 아니다. |
| `clear` | 네 모터의 두 배율을 모두 1로 복원한다. 이미 소비한 배터리를 충전하거나 프로파일을 바꾸지 않는다. |
| 물리 영향 | Gazebo 추진력, 실제 joint 속도, PX4의 모터 제어 출력을 직접 바꾸지 않는다. 프로펠러 파손·물리적 모터 정지·추락 시험이 아니다. |

배터리 단계와 모터 이상은 별도로 선택할 수 있다. 처음에는 정상 기준/무주입 상태를 관찰하고 한 조건씩 바꿔 비교한다. 설정이 구분된다고 AI의 탐지·모터 위치 판별 성능이 검증된 것은 아니다.

## 6. 초기화·pause·반복의 차이

| 명령 | 바뀌는 범위 | 조건·현재 상태 |
|---|---|---|
| `clear` | 모터 이상 배율만 해제 | 실행 중 가능, pause 중에는 먼저 resume |
| `reset` | 현재 배터리의 초기 SOC·소비량 복원, 모터 이상 해제 | 지상 Disarmed. Gazebo 세계·시간은 되감지 않음 |
| `restore` | run 최초 선택 프로파일로 돌아가 초기화 | 지상 Disarmed |
| `pause` / `resume` | Gazebo 시간 정지/재개 | 서비스·수집기는 계속 실행. 새 모델 계산과 반복 수신을 구분 |
| `world-reset` | 세계·시뮬레이션 시간 reset과 PX4/sender 재시작 | 지상 Disarmed. 같은 run 유지, 수신 공백·시계 세대 경계 발생 |
| `stop` 후 `start` | PX4/Gazebo를 종료하고 새 run·로그로 시작 | 먼저 착륙·Disarm |

일시정지와 재개:

```bash
python tools/arion_sim.py pause
python tools/arion_sim.py status
python tools/arion_sim.py resume
```

pause 중에는 `status`·`watch`로 관찰할 수 있지만 일반 상태 변경 명령은 거부된다. 재개 후 프로파일 선택·이상 주입·종료를 수행한다. `land`는 pause 상태에서도 재개 후 착륙을 시도하는 예외다. 배터리 적분은 시뮬레이션 시간 기준이라 pause 동안 진행하지 않지만 PX4의 이전 값 재송신은 있을 수 있다.

같은 배터리 조건을 다시 시작하기 위한 명령 순서는 `land` → `reset` → `takeoff`다. 다른 배터리 조건은 지상에서 `profile ID`로 초기화한 뒤 이륙한다. 정상 기준과 84.812594% 단계의 순차 비행은 확인했지만 모든 단계·임의 횟수 반복의 내구성을 보장하는 것은 아니다. 문제가 생기면 강제로 반복하지 말고 로그를 남긴다. run 자체를 분리하려면 착륙 후 `stop` → `start`를 사용한다.

### 세계와 시계를 처음으로 되돌리기

배터리만 초기화하려면 `reset`을 사용한다. 기체와 세계·시뮬레이션 시간까지 초기화할 때만, 착륙·Disarm 및 pause 해제 후 다음을 사용한다.

```bash
python tools/arion_sim.py world-reset
python tools/arion_sim.py status
```

서비스는 run의 초기 `world.sdf`에 CMU 기체를 include하고 `PX4_GZ_MODEL_NAME`으로 PX4를 **이미 존재하는 기체**에 연결한다. 앞선 동적 spawn 방식에서는 Gazebo 전체 reset이 기체를 제거했고, PX4를 그대로 둔 시계 역행은 lockstep 메시지 대기 지연을 일으켰다. 현재는 다음 순서로 시계 세대를 분리한다.

1. 최신 Disarmed 상태를 확인하고 배터리 sender를 종료한다.
2. 소유 PX4를 정상 종료한다. 비정상 종료이면 reset 성공으로 진행하지 않는다.
3. Gazebo의 세계·시간 전체 reset과 Plugin의 reset 적용을 확인한다.
4. `px4_epoch`를 증가시키고 새 PID의 PX4와 sender를 시작한다. 자동 무장은 하지 않는다.
5. `BATTERY_STATUS`, `ESC_STATUS`, `HEARTBEAT`, `LOCAL_POSITION_NED`, `EXTENDED_SYS_STATE` **각각의 새 수신 count 증가와 최신값**, Disarmed를 확인한다. 이전 cache만으로 성공 처리하지 않는다.

**Gazebo·공용 수집기·서비스 프로세스와 `run_dir`는 유지**된다. `status`는 처리 중 `world_resetting`, 완료 후 `running`을 보여 준다. 응답의 `telemetry_gap_expected: true`처럼 PX4 재시작 동안 **짧은 수신 공백을 예상**해야 한다. 무중단 메시지 수신이라는 뜻이 아니다. 실패하면 오류와 종료 결과를 확인하고 비행을 계속하지 않는다.

새 epoch의 진단은 `px4-epochN.log*`, `sender-epochN.log*`로 나뉜다. 정답 로그의 `px4_epoch_closed`와 `world-reset` 요청/적용 기록을 함께 보며, 하나의 run 안에서도 시뮬레이션·PX4 시각의 되감김 경계를 분리해 분석한다. `clear`와 배터리 `reset`은 이 PX4 재시작을 하지 않는다. 지상 reset·수신 복구와 새 epoch에서의 후속 비행은 각각 별도 관측으로 확인했다(9절).

## 7. 기록 보관과 종료

기록은 `status`의 `run_dir`에 있다. 기본 `init`의 출력 루트는 `experiments/service/`지만 이미 준비된 PC의 설정은 다를 수 있다.

| 파일/폴더 | 용도 |
|---|---|
| `telemetry/mavlink_raw.jsonl`, `mavlink_raw.000001.jsonl` 등 | PX4 출력의 디코딩 원문 필드, source ID, packet sequence, 수신 시각, 별도 품질 주석 |
| `telemetry/metadata.json` | 디코더 식별, 수신 endpoint, source 필터, 품질 해석 설정 |
| `telemetry/summary.json` | 종료 시 파서·저장 건수와 오류. UDP 수와 메시지 수는 별도 |
| `run.metadata.json` | 최초 프로파일·전체 카탈로그·바이너리 식별·실행 설정. AI 센서 입력 아님 |
| `groundtruth.events.jsonl*` | 명령 요청/적용/실패, 프로파일·주입·초기화와 정답 구간을 추적하는 별도 기록 |
| `px4.log*`, `sender.log*`, `px4-epochN.log*`, `sender-epochN.log*`, `gazebo.log*` | 프로세스 진단 원문. `N`은 world-reset 후 PX4 세대 번호. GUI를 열었다면 `gui-*.log*`도 생성 |
| `world.sdf`, `models/` | 이번 run의 초기 세계·CMU 모델. 기체는 세계 시작 시 include하며 원본 PX4 world는 바꾸지 않음 |
| `rootfs/log/` | PX4가 생성하는 ULog. MAVLink 기록과의 시각 정렬을 자동 보장하지 않음 |
| `service.summary.json` | 최종 상태·오류·소유 프로세스 정리 결과 |

`mavlink_raw`는 **디코딩한 원문 기록**이지 UDP 바이트 캡처(PCAP)가 아니다. 수신 시각은 센서 생성 시각과 다르며 원래 메시지의 시각 필드도 함께 보존한다. 품질 주석은 데이터의 의미·제한을 설명하며 정답이나 측정 유효성 인증이 아니다. 제어 요청 시각을 실제 센서 이상 발생 시각으로 무조건 대입하지 않는다. 상세 형식은 [공용 수집기 안내](../tools/mavlink_collector/README.md)를 따른다.

### 파일 회전은 전체 보관 상한이 아니다

- `rotate_bytes`의 기본값은 파일당 **16MiB**다. MAVLink JSONL과 프로세스·groundtruth 로그에 적용되며 과거 파일을 자동 삭제하지 않는다.
- MAVLink JSONL은 레코드 사이에서 나뉜다. groundtruth와 프로세스 로그는 바이트 단위로 나뉠 수 있으므로 기본 파일 뒤에 `.000001`, `.000002` 순서로 이어 읽는다. 분할 하나의 끝이 JSON 행 중간일 수 있다.
- ULog, metadata, 시작 launcher 로그까지 모두 이 상한으로 관리된다는 뜻은 아니다. **전체 디스크 보관량은 별도 관리**해야 한다.
- run 파일시스템의 여유 공간이 64MiB 미만이면 서비스는 기록 손실을 숨기지 않고 오류 종료 경로로 들어간다. 이 감시는 백업·보관 정책이나 전원 장애 내구성 보장이 아니다.

장기 운영 전 저장 공간과 종료된 run의 보관·이동 정책을 정한다. 실행 중인 로그를 임의로 삭제하지 않는다.

### 정상 종료

```bash
python tools/arion_sim.py land
python tools/arion_sim.py stop
python tools/arion_sim.py status
```

이미 지상 Disarmed라면 `land`는 생략한다. pause 상태라면 먼저 `resume`한다. 공중에서 `stop`은 거부되므로 먼저 착륙한다. 정상 종료 판단은 `state: stopped`, 오류 목록, `cleanup`과 `service.summary.json`을 함께 확인한다. 중지 요청 수락만으로 모든 자원 정리가 끝났다고 해석하지 않는다.

수집기 최종 summary 쓰기 실패나 종료 후 남아 있는 수신 worker도 서비스 오류로 처리한다. 이 경우 `state: failed`, `ok: false`와 실패 종료 코드가 반환되므로 수집기 하위 오류를 확인한다.

서비스는 자신이 시작한 PID 그룹과 해당 instance의 socket을 대상으로 정리한다. 다른 Gazebo/PX4를 전역 종료하는 명령은 쓰지 않는다. 이전 소유자가 불명확한 lock/socket이 있다는 오류가 나면 파일을 바로 지우거나 중복 서비스를 띄우지 말고 해당 실행의 상태와 로그부터 확인한다.

## 8. 다른 팀원의 PC에서 준비하는 순서

이 절은 **아직 서비스 설정·전용 빌드가 없는 PC용**이다. 현재 준비된 PC에서는 건너뛴다. 팀의 `PX4_설치부터_첫비행까지.md`로 Ubuntu 24.04 / x86_64, Gazebo Harmonic, PX4 v1.17.0, `.venv`와 기본 첫 비행을 먼저 준비한다. 팀 `simulation` 저장소, 재귀 submodule을 포함한 `PX4-Autopilot/`, 기존 Python 환경의 `pymavlink`·`mavgen.py`가 필요하다. 아래 절차는 기본 의존성 설치를 대신하지 않는다.

예시 경로를 먼저 지정한다. 빌드 도구는 설치된 컴파일러·CMake·Ninja를 사용하며, 결과와 생성 디코더는 소스와 별도 폴더에 둔다. **overlay 출력은 제품 저장소 밖이어야 한다.** 아래 `artifacts/`는 팀원이 직접 사용하는 일반 작업 폴더이며 에이전트 전용 경로가 아니다.

```bash
cd ~/px4-workspace/simulation
source .venv/bin/activate
SIM_DIR="$PWD"
PX4_DIR="$SIM_DIR/PX4-Autopilot"
ARTIFACT_DIR="$(cd .. && pwd)/artifacts/cmu-service"
BASE_BUILD="$ARTIFACT_DIR/px4-base"
PX4_OVERLAY="$ARTIFACT_DIR/px4-cmu"
POWER_BUILD="$ARTIFACT_DIR/power"
IPC_DIR="$ARTIFACT_DIR/ipc"
DECODER="$ARTIFACT_DIR/generated/px4_messages.py"
mkdir -p "$ARTIFACT_DIR"
```

각 단계가 오류 없이 끝나야 다음으로 넘어간다. 아래 준비 명령은 아직 시뮬레이터를 시작하거나 비행하지 않는다. 경로를 바꿨다면 이후 단계에도 같은 변수를 사용한다.

### 8-A. 외부 모터 모듈을 포함한 기본 빌드

고정 대상은 PX4 **v1.17.0 / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`**다. 깨끗한 해당 체크아웃에는 기존 ESC 상태 발행 전환 패치가 필요하다. 원본에 직접 적용하는 것은 이 전환 패치이며 기존 모터 구동 경로를 교체하지 않는다. 이미 적용된 PC에서는 아래 패치 명령을 반복하지 않는다.

```bash
git -C "$PX4_DIR" apply --check \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch" &&
git -C "$PX4_DIR" apply \
  "$SIM_DIR/patches/px4-v1.17.0/0001-gz-esc-status-source-switch.patch"
```

`--check`가 실패하면 강제로 적용하지 않는다. 기존 적용 여부·수정·버전을 확인한다. 다음은 외부 `arion_motor_bridge` 모듈을 포함한 **별도 기본 빌드**를 만드는 명령이다. CMake가 빌드 규칙을 만들고, `cmake --build`가 실행파일을 컴파일한다.

```bash
cmake -S "$PX4_DIR" -B "$BASE_BUILD" -G Ninja \
  -DCONFIG=px4_sitl_default \
  -DEXTERNAL_MODULES_LOCATION="$SIM_DIR/px4" \
  -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$BASE_BUILD" --target px4 -j 2
```

overlay는 이 빌드의 생성 헤더·라이브러리·`compile_commands.json`·모듈 등록·시작 파일을 재사용한다. 모듈이 빠진 일반 stock 빌드나 이동해 경로가 달라진 cache를 대신 지정하지 않는다.

### 8-B. 배터리 변환과 IPC를 분리한 SITL overlay

overlay는 기본 cache와 원본을 입력으로 삼아 **새 Linux SITL 실행파일**을 만든다. 배터리 변환 수정과 IPC 경로 수정은 복사된 overlay 소스에 적용한다. `0002-sitl-external-battery.patch`를 원본 PX4에 직접 적용하는 절차가 아니다.

```bash
python tools/build_sitl_overlay.py \
  --repo "$SIM_DIR" --px4-source "$PX4_DIR" \
  --base-build "$BASE_BUILD" \
  --output "$PX4_OVERLAY" \
  --ipc-dir "$IPC_DIR"
```

결과는 `$PX4_OVERLAY/bin/px4`다. 같은 출력의 `etc/`, 명령 symlink와 `px4-alias.sh`를 함께 유지한다. 기존 생성 결과를 덮어쓰지 않으므로 재빌드는 새 output 경로를 선택한다. IPC 경로가 실행파일에 포함되므로 `init`에도 **같은 IPC 경로**를 넣는다. 빌드 성공은 아직 실행·비행 성공이 아니다. 상세 경계는 [SITL overlay 안내](sitl-overlay-build.md)를 따른다.

### 8-C. Plugin·sender·제어기 빌드

세 결과물은 각각 모델 계산, 배터리 입력 전달, Plugin 제어 요청을 담당한다. 고정값 시험 Plugin을 CMU 모델에 함께 붙이지 않는다.

```bash
cmake -S "$SIM_DIR" -B "$POWER_BUILD" -G Ninja \
  -DARION_BUILD_BATTERY_COMPONENTS=ON \
  -DARION_BUILD_COUPLED_POWER=ON \
  -DARION_MAVLINK_INCLUDE_DIR="$BASE_BUILD/mavlink"
cmake --build "$POWER_BUILD" --target \
  ArionCoupledPowerPlugin arion_battery_sender arion_power_control -j 2
```

`libArionCoupledPowerPlugin.so`, `arion_battery_sender`, `arion_power_control`이 이 빌드 폴더에 생성되어야 한다. 시스템 Gazebo에 설치하거나 기존 라이브러리를 덮어쓰는 명령이 아니다.

### 8-D. PX4 정의와 맞는 Python 디코더 생성

해당 PX4의 메시지 정의를 읽어 Python용 해석 파일을 만든다. 특히 ESC_STATUS 정의를 임의의 다른 dialect로 바꾸지 않는다. 생성물은 Python 코드로 읽히므로 신뢰하는 소스에서 직접 생성한 파일만 사용한다.

```bash
mkdir -p "$ARTIFACT_DIR/generated"
mavgen.py --lang=Python3 --wire-protocol=2.0 \
  --output="$DECODER" \
  "$PX4_DIR/src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml"
```

### 8-E. 이 PC의 실행 경로 저장

`init`은 경로와 설정을 저장할 뿐 설치·자동 시작을 하지 않는다. 파일 생성 성공만으로 바이너리 로딩과 텔레메트리 수신이 검증되는 것은 아니다.

```bash
python tools/arion_sim.py init \
  --px4-source "$PX4_DIR" \
  --px4-binary "$PX4_OVERLAY/bin/px4" \
  --plugin-build "$POWER_BUILD" \
  --decoder "$DECODER" \
  --ipc-dir "$IPC_DIR"
```

기본 설정 파일은 `config/service.local.json`이다. 기본 catalog는 `profiles/battery/cmu_runtime/catalog.json`, 상태 폴더는 `.arion-service/`, 기록 루트는 `experiments/service/`다. 필요하면 `init`의 `--output-root`, `--state-dir`로 위치를 명시한다. 설정의 상대 경로는 **설정 파일 폴더가 아니라 simulation 저장소** 기준이다.

기존 설정은 덮어쓰지 않는다. 설치 위치를 확인해 의도적으로 편집하거나, `--config config/service.other.json init ...`처럼 새 설정을 만든다. `--config`는 **하위 명령 앞**에 쓰며, 별도 설정을 썼으면 이후 `start`, `status`, `stop`에도 같은 옵션을 붙인다. 다른 PC의 절대 경로 설정을 그대로 복사하지 않는다.

이제 3절의 `profiles` → `start` → `status`부터 진행한다. 별도 PC의 설치·실행 결과는 그 PC에서 확인해야 하며, 아래의 기존 통합 관측을 그대로 승계하지 않는다.

## 9. 현재 확인된 범위와 남은 확인

다음은 실제 로컬 통합 실행과 저장 원문 재계산으로 확인한 범위다. 문서 작성 자체가 새 빌드·시험·비행을 수행했다는 뜻은 아니다. 최종 run은 약289.70초 동안 대상 메시지13,717개를 저장했으며, 원문 전체가 PX4 system36/component1이고 종료 summary와 개수가 일치했다. `verified-runtime/parent-readback.json` 및 `final-stop.json`으로 종료·수신 근거를 구분해 보존한다.

| 항목 | 상태 |
|---|---|
| 네 CMU 프로파일 지상 선택, 수신 원문 전압 대조 | 확인됨 |
| 7초 pause 뒤 재개·수신 복구 | 확인됨. pause 중 반복 송신과 새 센서 측정은 구분해야 함 |
| 지상 `world-reset` 뒤 기체·PX4·필수 메시지 수신 복구 | 재확인 통과. 통합 기록 `verified-runtime/ground-reset-result.json`의 `passed: true`; 수신 공백을 포함한 재시작 방식 |
| world-reset 후 PX4 epoch 1에서 목표 3m 순차 비행 | 정상 100% → 열화 84.812594%의 두 차례 이륙·호버·착륙·자동 Disarm 통과. `verified-runtime/repeated-flights-result.json`의 `passed: true` |
| 매 비행의 모터 1·4 이상 주입·해제 | 전류 1.6/RPM 0.85 주입과 `clear`의 수신값 변화 확인. 공중 reset 거부도 확인 |
| 최종 서비스 종료·소유 자원 회수 | 확인 완료. CLI `stop` 후 `stopped`, 오류 없음, 현·이전 epoch의 모든 소유 프로세스 exit0·강제kill/잔류 그룹 없음, IPC/control socket 정리 확인 |
| 모든 프로파일의 반복 비행, 다른 PC 재현, 대규모 시험·24/7 내구성 | 수행하지 않음 |
| 선택 GUI 창 표시·종료 안정성 | 이 서비스의 확인 완료 범위에 포함하지 않음 |
| 실기체 적용과 AI 탐지 성능 | 검증하지 않음 |

두 비행의 초기 무주입 baseline 구간에서 관측한 중앙값은 다음과 같다. 예시 출력이나 실기체 성능 예측값이 아니다.

| 배터리 단계 | 팩 전류 A | 팩 전압 V | 고도 m |
|---|---:|---:|---:|
| 정상 상대 100% | 28.12 | 15.048 | 2.634 |
| 열화 상대 84.812594% | 28.11 | 14.612 | 2.667 |

목표는 3m였으나 baseline 고도 중앙값이 정확히 3m인 것은 아니다. 비슷한 부하에서 다른 전압이 관측된 이번 두 구간은 모델의 시나리오 동작 확인이며, 모든 열화 단계의 전압 순서나 AI 정확도를 보증하지 않는다.

모터 전류는 기존 **16V bench 표 기반 근사**이고 배터리 전압이 추진 물리로 되먹임되지 않는다. 배터리 온도 25°C는 고정 placeholder이며 열 모델·실측 온도 특징이 아니다. 이 서비스의 RC/GCS 연결 손실 관련 설정은 **로컬 SITL 전용**으로 실제 기체에 복사하지 않는다.

### 문서·구현 근거

- 명령·허용 범위: [`tools/arion_sim.py`](../tools/arion_sim.py), [`tools/sim_service/runtime.py`](../tools/sim_service/runtime.py)
- 프로파일의 실측·추정·가정 구분: [CMU 런타임 프로파일](cmu-runtime-profiles.md), [catalog](../profiles/battery/cmu_runtime/catalog.json), [provenance](../profiles/battery/cmu_runtime/provenance.json)
- 원래 부하 연동 모델과 레거시 기본값: [coupled-power-model.md](coupled-power-model.md)
- 수신 형식·freshness·회전: [공용 MAVLink 수집기](../tools/mavlink_collector/README.md)
- 보존된 오프라인 초기 자료: [초기 정상 프로파일 안내](cmu-evtol-normal-profile-guide.md)
- 활성 경로에서 제외한 자료: [NASA 폐기 안내](nasa-retirement.md)
- 자료 출처 고지: [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)
