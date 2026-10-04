# CMU eVTOL 초기 정상 프로파일 사용 안내

## 현재 위치: 보존된 오프라인 초기 정상 자료

이 문서의 `cmu_evtol_normal_soh100` **6개 프로파일과 도구는 그대로 보존**한다. 초기 원본의 추적·단위 확인·오프라인 재생용이며, 현재 부하 연동 서비스에서 선택하는 네 배터리는 별도의 `profiles/battery/cmu_runtime/catalog.json`이다. 실제 운영은 [CMU 서비스 가이드](cmu-service-guide.md), 네 단계의 원본·등가 OCV·저항 산정은 [런타임 프로파일 문서](cmu-runtime-profiles.md)를 따른다.

런타임의 상대 용량은 100.000000 / 95.027445 / 90.422638 / 84.812594%이며 **4S2P 가정과 파생 모델**을 사용한다. 아래 오프라인 번들은 그런 팩 모델이 아니다. 아래의 무스케일링·무열화 주입·원본 prefix 무결성 설명은 이 초기 번들의 계약으로 유지하며, 이후 전체 VAH01 검증을 이 번들의 생성 이력에 소급 적용하지 않는다. NASA 구성은 현재 활성 경로에서 제외되었다. [NASA 폐기 안내](nasa-retirement.md)와 [출처 고지](../THIRD_PARTY_NOTICES.md)를 참고한다.

## 역할과 범위

이 도구는 CMU eVTOL 실험의 **초기 기준 방전과 첫 임무 방전**을 읽어 전압·전류·표면 온도의 원본 시계열을 별도 CSV로 보존한다. 초기 RPT(Reference Performance Test)는 노화 시험 시작 시 수행한 기준 용량 시험이다. 이륙·순항·착륙을 모사한 부하 실험이며 실제 X500 비행 로그는 아니다.[38][39]

- Python 3: CSV 생성·무결성 검사·인과 재생에 사용한다. 별도 라이브러리 설치가 필요 없다.
- 원본 CSV: 셀별 시험 기록이다. 문서와 원본은 CC BY 4.0 출처를 표시한다.[25]
- 제품 CSV: 시간 `elapsed_s`와 관측값 `voltage_v`, `current_a`, `temperature_c`만 포함한다.
- metadata/manifest: 출처, 용량 기준, 셀 분할, 선택한 행 범위, 해시를 담는다. **AI 관측 특징에 합치지 않는다.**

SOH 100%는 셀의 **초기 기준 용량을 100%로 정한 상대 기준**이다. 절대적인 무열화 인증은 아니다. 방전 중 SOC 감소는 정상이며, 원본에 없는 SOC를 만들어 넣지 않는다. 용량 축소, 열화·고장 주입, 팩 스케일링 및 보간은 하지 않았다.

## 저장 위치와 분할

저장소 기준 제품 위치: `profiles/battery/cmu_evtol_normal_soh100/`.

| 원본 셀 | 역할 | 초기 기준 방전 행 | 첫 임무 방전 행 |
| --- | --- | ---: | ---: |
| VAH01 | train | 18,334 | 983 |
| VAH17 | validation | 18,396 | 102 |
| VAH27 | test_normal | 18,425 | 102 |

실제 생성 결과는 **6개·56,342행**이다. 번들 쓰기와 검증은 VAH01/train, VAH17/validation, VAH27/test_normal 각각의 `initial_reference_discharge`·`first_mission_discharge` 두 종류를 모두 요구한다. 누락·중복·예상 밖 ID와 불완전한 분할은 거부한다. 같은 셀의 두 프로파일는 반드시 같은 분할에 속한다. 정상 테스트와 비정상 테스트는 별개이며, 이 제품에는 비정상을 생성하지 않았다. 초기 생성 당시 NASA bundle과 별도 디렉터리로 보존했다. NASA bundle은 이후 활성 구성에서 제외되었으며, 어느 자료든 수치·전지 특성이 같은 데이터처럼 임의 혼합하지 않는다.

## 원본과 초기 구간 확인 방법

고정 출처는 eVTOL Battery Dataset **version 3**, DOI `10.1184/R1/14226830.v3`다.[25]

원본 README는 VAH01/17/27을 Baseline으로 설명하고, `cycleNumber`가 전 수명의 정확한 사이클 번호가 아니라고 경고한다. 따라서 `cycleNumber == 0`인 행을 모두 모으지 않는다. 파일 시작부터의 연속 segment 순서와 행 위치, 첫 완전한 임무 이후 경계를 함께 검사한다.[38]

초기 segment 순서는 `0,1,2,3,4,5,6,7,8,0,1,3,4,5,6,7,0`이다. 첫 기준 방전은 이 중 index 3~7, 첫 임무 방전은 index 12~14만 선택한다. 이륙·순항·착륙 시간은 각각 75/800/105초의 기준과 원본 종료 시점의 0.1초 이내 차이만 허용한다. 후속 휴지 구간과 다음 시험 시작 행이 없는 잘린 입력은 거부한다. 기준 용량은 선택한 초기 RPT 방전 전류를 **원본 시간 차이**로 사다리꼴 적분해 Ah로 계산한다.[39]

구현은 전체 원본의 크기/공식 MD5가 일치하거나, 이번에 HTTP Range로 별도 검증한 시작 구간의 크기/SHA-256이 정확히 일치할 때만 입력을 받아들인다. 생성된 metadata는 전체 파일 검증과 시작 구간 검증을 구분한다. 이번 제품은 시작 구간에서 완전한 CSV 행까지만 보존한 입력으로 만들었고, **전체 노화 CSV를 다운로드·MD5 검증했다고 표시하지 않는다.**

게시하는 번들에는 `source_integrity`가 필수다. DOI·파일명·원본 크기·공식 체크섬은 고정 출처 값과 문자열 그대로 비교하며 정규화하거나 보정하지 않는다. 시작 구간은 고정 크기/SHA-256과 전체 MD5 미검증 표시를, 전체 파일은 원본 크기와 전체 MD5 검증 표시를 요구한다. 단위, 무보간, 선택한 segment 순서, 원본/제품 행의 동일 개수·연속 범위·경계 시각 정렬도 검사한다. 선택한 초기 RPT 관측 행으로 Ah를 다시 적분하고 같은 셀의 임무가 동일한 출처와 용량 기준을 공유하는지 확인한다. 개별 `build_profiles`·`ProfileReplay`는 출처 정보를 붙이기 전에도 사용할 수 있지만 그 상태로 번들을 게시할 수는 없다.

## 팀원 실행 순서

아래는 저장소가 `~/px4-workspace/simulation`에 있고 원본을 `~/px4-workspace/battery-data/cmu-evtol-v3`에 보관하는 예다. 개인 사용자명이나 Hermes가 필요하지 않다.

1. 공식 데이터 페이지에서 version 3의 `VAH01.csv`, `VAH17.csv`, `VAH27.csv`를 내려받아 원본 폴더에 넣는다. 도구가 공식 크기/MD5를 확인하므로 다른 버전이나 불완전한 다운로드는 거부한다.[25]
2. 저장소로 이동한다. 이미 제공된 제품을 검사하려면 다음 명령을 사용한다.

```bash
cd ~/px4-workspace/simulation
python3 tools/battery_profiles/cmu_evtol.py verify --bundle profiles/battery/cmu_evtol_normal_soh100
```

3. 원본에서 재생성할 때는 기존 제품이 아닌 **새 출력 폴더**를 지정한다. 도구는 기존 폴더를 덮어쓰지 않는다.

```bash
python3 tools/battery_profiles/cmu_evtol.py generate --sources ../battery-data/cmu-evtol-v3 --output ../battery-output/cmu-evtol-normal-v3
```

성공 출력의 `passed=true`는 위의 로컬 파일·출처 기록·선택 범위·용량 기준 계약과 해시가 서로 일치한다는 뜻이다. `verify`는 원본을 다시 읽거나 다운로드하지 않으며, 번들의 비서명 해시만으로 원본 측정값의 진위를 인증하지 않는다. 특히 전체 파일의 기록된 SHA-256은 형식·동일 셀 간 일관성 확인이며 원본 바이트의 재검증은 `generate`의 `read_source` 경로에서 수행한다. `px4_runtime_verified=false`, `absolute_soh_certified=false`를 별도로 읽는다. 명령은 PX4/Gazebo를 시작하거나 MAVLink를 보내지 않는다.

쓰기 전에 모든 metadata와 최종 manifest를 `allow_nan=False`로 직렬화하고 CSV 바이트를 준비한다. 따라서 중첩 NaN/무한대나 JSON으로 표현할 수 없는 추가 값 때문에 출력 폴더·CSV가 일부만 생기지 않으며 기존 폴더는 계속 덮어쓰지 않는다. 이 사전검사는 이후 디스크 오류·프로세스 중단까지 원자적 쓰기로 보장한다는 뜻은 아니다.

## 단위·시간·재생 계약

| 제품 필드 | 원본 | 처리 |
| --- | --- | --- |
| elapsed_s | time_s | 해당 프로파일 첫 시각을 빼며 원본 시간 차이는 보존 |
| voltage_v | Ecell_V | V 원본값 그대로 |
| current_a | I_mA | 원본 충전 +/방전 − 부호를 방전 +로 전환하고 mA→A 변환 |
| temperature_c | Temperature__C | 셀 표면 °C 원본값 그대로 |

원본 전류와 표면 온도의 의미는 저자 설명에 따른다.[38][39] 위의 변환 이외 임의 클리핑·절댓값 처리·온도 합성·리샘플링은 없다.

`ProfileReplay`는 요청 시각 이하의 최신 측정값만 반환하는 zero-order hold다. 측정 시각 사이에서 미래 값을 보간하지 않고 마지막 측정 이후에는 `None`이다. 시간을 되돌리려면 `reset()`이 필요하며 반환값 수정은 내부 원본에 영향을 주지 않는다.

**표본 주기 제한:** 이번 원본에서 VAH01 임무는 약 1초, VAH17/27 임무는 약 10초 간격이다. 초기 RPT는 세 셀 모두 약 1초다. 10초짜리 원본을 1Hz 고해상도 실측으로 표현하지 않는다. 빠른 과도응답 검증에는 쓰지 않고, 향후 학습 시 공통 시간 기준·유효 표본 처리부터 정한다.

## 이 오프라인 번들의 X500·실행 경계

CMU 시험 전지는 Sony-Murata 18650 VTC-6 단일 셀이다.[38] X500 실제 LiPo 팩의 용량·병렬수·내부저항을 이 데이터로 확정할 수 없다. 단순 전압 4배로 실제 4S 팩 모델이라고 표현하지 않는다. 이 CSV 재생 자체는 모터 부하에 반응하는 동적 Battery Plugin이 아니다.

이 오프라인 제품의 준비와 단위 시험은 실시간 `BATTERY_STATUS → PX4 → MAVLink → Python` 전달 검증을 대신하지 않는다. 초기 번들 제작 작업은 기존 정상 모터 경로, PX4 core, 모터 명령·비행제어, Git index/HEAD를 변경하지 않는 범위였다. 이후 추가된 CMU 서비스의 별도 통합 관측은 [서비스 가이드](cmu-service-guide.md)에 기록한다.

## Sources

[25] https://kilthub.cmu.edu/articles/dataset/eVTOL_Battery_Dataset/14226830
[38] https://ndownloader.figshare.com/files/26855063
[39] https://www.ebi.ac.uk/europepmc/webservices/rest/PMC10238407/fullTextXML
