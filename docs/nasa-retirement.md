# NASA 배터리 구성 폐기

사용자가 CMU를 배터리 자료의 기준으로 선택함에 따라 NASA 구성은 **활성 제품·실행 경로에서 제외**했다.

제외 항목:

- `tools/battery_profiles/nasa_rw3.py`
- `profiles/battery/normal_soh100/`
- `docs/battery-normal-profile-guide.md`
- `tests/test_battery_profiles.py` (NASA 전용 검증)

새 운영기는 CMU 카탈로그만 선택한다. 기존 `cmu_evtol_normal_soh100`은 초기 정상 구간의 원본 추적용 참고 자료이며, 부하 연동 운영 프로파일은 `cmu_runtime/catalog.json`이다. 두 자료의 용도는 같지 않다.

삭제로 출처를 잃지 않도록, 이 작업 환경에서는 위 파일의 원래 bytes를 프로젝트 작업 기록 아래 `hermes/cmu-service/retired-nasa/`로 옮겼다. `retirement-manifest.json`에 이전 경로·보관 경로·SHA-256을 기록했고 옮긴 뒤 해시 일치를 확인했다. NASA 원본 다운로드와 과거 보고서는 과거 근거로 보존했으며 새 학습·실행 입력으로 선택하지 않는다.

보관 디렉터리는 운영기에 필요한 의존성이 아니므로 팀원은 이 디렉터리 없이 CMU 서비스를 사용할 수 있다. 과거 보고서의 NASA 설명은 당시 수행 이력이지 현재 활성 구성 안내가 아니다.
