# CMU 실측 열화 기반 런타임 배터리 프로파일

## 결과와 적용 범위

`profiles/battery/cmu_runtime/catalog.json`은 **한 셀 VAH01의 서로 다른 실제 RPT 4회**에서 얻은 용량·전류 step 저항·전압 곡선을 담는다. 서로 다른 셀 4개, 학습된 AI, 실측 X500 팩이라는 뜻이 아니다. 원본은 CMU eVTOL **version 3**, DOI `10.1184/R1/14226830.v3`로 고정했다.[1][4]

- 기본 선택: `cmu_vah01_rpt000`.
- 공통 계약: `arion.cmu_battery_profiles.v1`, 4S2P, 초기 SOC 0.95.
- 각 CSV는 **SOC → 셀 등가 OCV** 조회표다. 시간축 V/I 재생 파일이 아니다.
- 카탈로그에 실측값과 파생 추정값, 가상 팩 설계 가정을 따로 기록했다.
- 활성 catalog·곡선 CSV·provenance와 `tools/battery_profiles/cmu_aging.py`는 유지한다. 초기 정상 3-cell bundle·재생기, 내려받은 CMU/NASA 원본, 과거 검증 산출물은 승인된 정리 후 로컬에 보관하지 않는다. 재생성·실원본 발행 시험에는 아래 고정 입력 재취득이 필요하며 서비스 실행에는 원본이 필요하지 않다.
- PX4/Gazebo, 비행, MAVLink 전달, AI 학습은 당시 프로파일 작업에서 실행하지 않았다. 이 문서 갱신도 새 실행 검증이 아니다.

## 선택한 단계

전체 VAH01에는 연속 프로토콜로 확인한 완전한 RPT가 18회 있다. 첫 RPT 대비 마지막 상대 용량은 **81.477380%**였다. 따라서 70% 실측 프로파일을 만들지 않고, 목표 100/95/90/85%에 가장 가까운 **서로 다른 실제 RPT**를 선택했다. 표의 SOH는 목표값이 아니라 원본의 용량 비율이다.[4]

| ID | RPT 순번(0기반) | 앞선 mission 수 | 실측 상대 SOH % | 실측 셀 Ah | 가정 4S2P 팩 Ah | 추정 셀 Ω | 가정 팩 Ω |
|---|---:|---:|---:|---:|---:|---:|---:|
| `cmu_vah01_rpt000` | 0 | 0 | 100.000000 | 3.089338075 | 6.178676150 | 0.023077655 | 0.046155310 |
| `cmu_vah01_rpt002` | 2 | 100 | 95.027445 | 2.935719042 | 5.871438083 | 0.022767087 | 0.045534175 |
| `cmu_vah01_rpt006` | 6 | 300 | 90.422638 | 2.793460970 | 5.586921940 | 0.025418005 | 0.050836011 |
| `cmu_vah01_rpt013` | 13 | 650 | 84.812594 | 2.620147757 | 5.240295514 | 0.029181204 | 0.058362409 |

표의 셀 용량과 전류 step은 원본 VAH01의 각 RPT에서 독립 추출했다. 팩 수치는 아래 설계 가정에 따른 변환이다.[4]

**95% 단계의 두 지점 공통 저항 추정은 초기보다 조금 낮다.** 열화 순서에 맞춰 저항을 강제로 증가시키거나 scalar multiplier를 적용하지 않았다. 이것은 동일 온도·동일 실제 SOC에서의 정밀 저항 비교가 아니며, 개별 RPT의 온도 범위와 두 step 잔차를 함께 보아야 한다.[4][5]

## RPT 식별과 용량

원저자 README는 합쳐진 CSV의 tester `cycleNumber`가 정확한 전체 cycle 번호가 아니라고 경고한다. 논문은 초기 및 50회 mission 뒤의 RPT, C/5 방전과 두 완화 구간을 설명한다.[2][3]

추출기는 다음을 실제 원본에서 확인한다.

1. **연속 Ns 0→1→2→3→4→5→6→7→8**: CC/CV 충전, 휴지, C/5 방전, 두 C/50 완화, 최종 방전과 휴지.
2. 전류 부호와 수준, 3/5 구간 약 3600/7200초, 4/6 구간 약 30초, 초기/후기 휴지의 길이.
3. 방전 시작 4.1–4.25V 범위, 끝 2.45–2.55V 범위, 방전 중 증가하는 전하 counter, 시간 역행·큰 표본 공백 없음.
4. RPT가 불완전하거나 전류가 충전 방향이거나 전하 counter가 재설정되면 실패한다. 서로 다른 RPT가 같은 `cycleNumber`를 가져도 순서상 별개로 취급한다.

이 조건은 검증한 **VAH01 v3 파일의 프로토콜용**이며, 모든 셀·다른 자료에 적용되는 범용 RPT 탐지기로 주장하지 않는다.

```text
Q_cell_Ah = (QDischarge_mA_h 마지막 - 첫 방전 표본) / 1000
relative_SOH_percent = 100 * Q_cell_Ah / 첫 완전 RPT의 Q_cell_Ah
```

별도로 원본 `time_s`와 `-I_mA/1000`을 사다리꼴 적분해 검산한다. 선택 RPT의 counter 대비 적분 차이는 각각 +0.000090320, −0.000011439, −0.000033745, +0.000019115Ah였다. 상대 SOH 100%는 이 셀의 첫 RPT를 정규화한 값이지 인증된 절대 건전성이다.[4]

### 원본 행 범위

숫자는 **헤더 제외, 0기반, 시작 포함·끝 제외**다. 전압 곡선·저항 추정에 사용한 세부 구간과 보간에 이용한 두 행 번호까지 catalog의 `source`에 기록했다.

| ID | 전체 RPT 행 범위 | 용량 산정 방전 행 수 |
|---|---|---:|
| `cmu_vah01_rpt000` | `[0, 19211)` | 18,334 |
| `cmu_vah01_rpt002` | `[214520, 232622)` | 17,403 |
| `cmu_vah01_rpt006` | `[658376, 675739)` | 16,541 |
| `cmu_vah01_rpt013` | `[949107, 965552)` | 15,502 |

## 저항: effective DC 1s step estimate

논문의 별도 impedance 파일은 AC/EIS 결과가 아니라, C/5에서 C/50로 낮춘 뒤 1초/30초의 **DCIR**이다. 이 구현은 원본 전압·전류에서 직접 재계산하고 별도 impedance CSV는 교차 확인에만 사용한다.[3][5]

- 각 Ns 3→4, 5→6 전환 직전 표본을 기준으로 **정확히 +1초**의 V/I를 두 인접 원본 표본 사이에서 보간한다.
- 두 지점의 `ΔV = R × ΔI`를 절편 0 최소제곱으로 적합한다. catalog의 명칭은 `effective_DC_1s_step_estimate`다.
- 두 개뿐인 관측점, 약 0.54A 전류 변화에서의 유효 저항이다. SOC·온도·완화 시간에 무관한 고유 상수가 아니다. 수십 A 팩 부하에 그대로 적용하는 것은 명시적 단순화다.
- 두 step의 원저자 명칭은 20%/60% DOD지만 방전량 기준점이 대략 고정되어 있다. 감소한 실측 용량으로 재정규화한 실제 DOD는 달라지므로 `capacity_relative_dod`에 별도 기록했다.[4][5]
- raw RPT 순번과 원저자 impedance 행을 대응한 뒤, 두 30초 DCIR 값 일치로 대응을 검증한다. 원저자의 `cycle numbers`도 raw tester counter로 오인하지 않는다.
- 정확한 +1초 보간과 원저자 이산 표본 계산 사이의 작은 차이를 보존한다. 마지막 18번째 raw RPT는 별도 impedance 표에 없으며 선택하지 않았다.[4][5]

| ID | 저항 공통 적합 RMSE (V) | 등가곡선 재구성 RMSE (V) | 곡선 최대 절대 잔차 (V) |
|---|---:|---:|---:|
| `cmu_vah01_rpt000` | 0.000302176 | 0.001188666 | 0.012920539 |
| `cmu_vah01_rpt002` | 0.000751244 | 0.001474397 | 0.018413109 |
| `cmu_vah01_rpt006` | 0.000886482 | 0.002259530 | 0.030016516 |
| `cmu_vah01_rpt013` | 0.000565828 | 0.003071146 | 0.042833068 |

이 잔차는 동일 RPT에 대한 내부 근사 오차이며 독립 비행/고부하 검증 오차가 아니다.

## OCV 등가곡선의 정의와 한계

각 단계는 해당 RPT의 C/5 구간 Ns 3/5/7에서 따로 추출한다. C/50 완화 구간을 제외하고, 완화 후 5/7 구간의 처음 5초도 제외한다. 전하 counter로 각 RPT의 SOC 축을 만들고 다음 보정을 수행한다.

```text
SOC_reference = 1 - 해당 RPT 방전 Ah / 해당 RPT 전체 Ah
OCV_equivalent_cell_V = 측정 terminal_V + 측정 discharge_A * 추정 cell_Ohm
```

- 0..1을 포함하는 101개 SOC 지점으로 선형 보간한다. 외삽이나 단조화 보정은 하지 않는다.
- 이 곡선은 **평형 OCV 실측이 아니다.** C/5 분극·히스테리시스·온도 영향이 남아 있다.
- SOC=0은 해당 C/5 시험의 **2.5V 부하 차단점**이다. 화학적 완전 방전이나 무부하 평형 SOC=0으로 해석하지 않는다.
- 셀 등가 전압 끝점은 약 2.514–2.518V와 4.182–4.190V다. 낮은 SOC의 급격한 변화 구간은 비행 가능한 고부하 영역의 보증이 아니다.[4]
- RPT별 온도 범위는 각각 21.748–24.713°C, 20.965–25.101°C, 20.633–25.093°C, 24.112–27.077°C였다. 동일 온도 보정이나 열 모델은 제공하지 않는다.[4]
- 저항이 일정한 1차 근사이므로 RC 완화, 열화의 연속 진행, rate-dependent capacity, 병렬 셀 불균형, 전압 저하가 추진력에 돌아가는 폐루프 모델을 제공하지 않는다.

## 가상 팩·실시간 소비 계약

**4S2P는 설계 가정이다.** 같은 셀 특성을 가상의 동일 셀 8개에 적용하는 시나리오이며 실제 팩 측정이 아니다.

```text
Q_pack = Q_cell * 2
R_pack = R_cell * 4 / 2
V_pack = 4 * OCV_equivalent_cell(SOC) - I_pack * R_pack
SOC_next = clamp(SOC - I_pack * simulation_dt_seconds / (3600 * Q_pack), 0, 1)
```

서비스는 catalog를 검증해 SDF의 `battery_profiles/profile`로 전달한다. C++ JSON 파서를 새로 만들 필요는 없다. 모델은 **현재의 모터/보조 부하 전류와 simulation dt**를 사용해야 한다. CSV 행을 wall-time에 따라 순서대로 재생하면 이 계약을 충족하지 않는다. pause 시 dt=0이어야 하고 프로파일 변경 시 선택한 용량·저항·곡선과 초기 SOC를 함께 적용해야 한다. 이 문서는 데이터 계약이며 실제 서비스의 프로파일 선택·reset·부하 반응 실행 검증을 대신하지 않는다.

숨겨진 SOH·용량·저항·RPT ID는 시나리오 설정/정답용이다. 실기체에서 얻지 못하는 값을 AI 입력 특징으로 넘기지 않는다. 이 한 셀의 열화 단계 4개로 독립 train/validation/test 셀을 구성하거나 AI를 학습했다고 표현하지 않는다.

## 원본 무결성

당시 API에서 파일 ID·이름·크기·download URL·MD5를 확인했고 전체 `VAH01.csv`와 README, impedance CSV의 MD5 일치를 기록했다.[1] 아래 값은 보존된 provenance의 식별 정보이며 정리 후 로컬 원본이 존재하거나 이번에 다시 검증했다는 뜻은 아니다.

| 원본 | byte 수 | 실제 MD5 |
|---|---:|---|
| `VAH01.csv` | 110884708 | `5b0ac8963f1d161d4b74c731f8d8640a` |
| `README.txt` | 5832 | `d65302bc58a41b5272594ae2f8766525` |
| `VAH01_impedance.csv` | 1522 | `02c687dbcfaa1ccc23c66f5ca51a9514` |

VAH01 전체 SHA-256:

```text
596ab1e1e80eeac3509f789609f751527c2073a4310deec8274cbfaf4fef39c4
```

각 원본·참고문서의 실제 hash, 고정 출처 URL, 전체 여부, 추출기 hash, catalog/CSV hash는 `provenance.json`에 있다. 메타데이터/논문 XML은 로컬 hash를 측정했지만 저장소 제공 checksum이 있다는 뜻은 아니다. 기존 3MB 초기 prefix는 열화 catalog 입력으로 허용하지 않는다.

## 재생성

**먼저 고정 원본을 다시 확보해야 한다.** 보존된 catalog/CSV는 실행 입력이지 전체 원본의 대체물이 아니다. Python 표준 라이브러리만 사용하며 패키지 설치나 simulator 실행이 없다. `simulation` 저장소에서 실행한다. 아래 `DATA_DIR`는 팀원용 분석/다운로드 예시 위치이며, 제거된 과거 작업 폴더가 이미 있다고 가정하지 않는다.

```bash
DATA_DIR="$HOME/px4-workspace/battery-data/cmu-evtol-v3"
python3 -B tools/battery_profiles/cmu_aging.py \
  --raw-dir "$DATA_DIR" \
  --output-dir "${DATA_DIR}-rebuilt-cmu-runtime"
```

입력 폴더에는 `VAH01.csv`, `VAH01_impedance.csv`, `README.txt`, API 응답 `cmu-v3.json`, 논문 원문 `article.xml`이 필요하다. 아래 Sources의 고정 주소에서 받아 둔다. 모든 다운로드 시도에는 연결/전체 시간 상한을 두며, 120초 내 실패한 다운로드를 완료 파일로 사용하지 않는다. 대용량 응답이 중단되면 검증된 `Content-Range`와 byte 수의 범위를 재시도/결합한 뒤 **전체 MD5**를 확인한다. 이미 받은 올바른 부분은 보존할 수 있지만 전체 checksum 검증 전에는 승인 입력이 아니다.

추출기의 `FILE_PINS`는 앞의 세 파일의 크기·MD5와 API metadata의 파일 식별을 고정한다. `cmu-v3.json`·`article.xml`은 당시 로컬 hash를 [provenance](../profiles/battery/cmu_runtime/provenance.json)에 기록한 참고 입력이다. 같은 URL의 현재 응답이 같은 bytes라는 보장은 없으므로 **당시 산출물의 byte 재현에는 두 문서도 기록된 hash와 일치하는 사본이 필요**하다. 일치 사본을 확보하지 못하면 동일 재생성 확인은 미완료이며 검증 조건을 완화하지 않는다. 이 정리에서 다운로드·재생성·`PublicationTests`는 실행하지 않았다.

출력 폴더가 이미 있으면 덮어쓰지 않고 실패한다. 파싱·해시·수치 검증과 엄격한 JSON 직렬화를 끝낸 뒤 임시 분석 폴더에서 완성 디렉터리를 공개한다. raw 폴더와 출력은 atomic rename 가능한 같은 파일시스템을 사용한다.

```bash
CMU_AGING_RAW_DIR="$DATA_DIR" \
  python3 -B -m unittest discover -s tests -p test_cmu_aging.py -v
```

원본 경로를 주지 않으면 기존 시험 조건에 따라 실제 원본 발행 시험만 건너뛰고 합성 단위 시험을 실행한다. 이는 전체 검증 통과가 아니며 `PublicationTests`는 미실행으로 남긴다. 합성 fixture를 CMU 실측으로 발행하지 않는다.

## 과거 프로파일 생성 검증 이력

아래는 당시 기록이며 현재 재실행 결과가 아니다. 원본과 검증 로그·분석 산출물은 정리 후 미보관이므로 현재 로컬 증거 파일로 재대조할 수 없다. 활성 파생 산출물과 provenance의 식별·산정 정보는 유지한다.

- 단계별 RED → GREEN 집중 TDD와 최종 **7 tests, skip 0, warning 0** 통과.
- 전체 VAH01 **1,091,477 data rows** 읽기와 완전 RPT 18회 식별.
- 별도 원본 parser로 선택 구간의 67,780개 행을 읽고, 용량·저항과 **4×101개 곡선값**을 원본 인접 행에서 독립 재계산하여 일치 확인.
- 재생성한 catalog/CSV/provenance **6개 파일의 byte 동일성** 확인.
- prefix 원본 거부, 미완료 RPT·비정상 전류/시간/전하/전압 거부, 지원하지 않는 70% 목표 합성 거부, 기존 출력 덮어쓰기 거부 확인.
- 당시 부산물 위치는 프로젝트 `hermes/cmu-service/data/`였다. TDD 로그, 다운로드·독립 재계산·재생성 대조 결과는 현재 미보관이다.
- 최초 전체 다운로드와 일부 큰 범위 다운로드는 제한시간으로 실패했다. 당시 실패 부분/헤더를 보존하고 빠진 범위를 제한시간 내 재취득한 뒤 전체 MD5를 확인했다고 기록했다. 그 파일들도 정리 대상이며 실패 이력을 성공으로 바꾸지 않는다.
- **미실행:** PX4/Gazebo, 서비스 SDF 소비, 동적 부하/프로파일 전환, 비행·센서·AI 검증. 상위 통합 검증에서 별도 확인해야 한다.

## Sources

[1] https://api.figshare.com/v2/articles/14226830/versions/3
[2] https://ndownloader.figshare.com/files/26855063
[3] https://www.ebi.ac.uk/europepmc/webservices/rest/PMC10238407/fullTextXML
[4] https://ndownloader.figshare.com/files/26855147
[5] https://ndownloader.figshare.com/files/40183168
