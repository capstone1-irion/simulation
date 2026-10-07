# NASA 배터리 구성 폐기

사용자가 CMU를 배터리 자료의 기준으로 선택함에 따라 NASA 구성은 **활성 제품·실행 경로에서 제외**했다.

제외 항목:

- `tools/battery_profiles/nasa_rw3.py`
- `profiles/battery/normal_soh100/`
- `docs/battery-normal-profile-guide.md`
- `tests/test_battery_profiles.py` (NASA 전용 검증)

새 운영기는 [CMU 런타임 카탈로그](../profiles/battery/cmu_runtime/catalog.json)만 선택한다. 초기 정상 6개 프로파일 번들은 별도 과거 자료였으며 이번 미사용 파일 정리로 삭제했다. 활성 런타임 카탈로그·CSV·provenance는 유지한다.

과거에는 제외 파일을 프로젝트 `hermes/cmu-service/retired-nasa/`에 옮기고 해시를 대조했다. 이는 당시 보관 이력이며 현재 보존 약속이 아니다. 승인된 정리 후 이 보관 사본·manifest, 내려받은 NASA/CMU 원본, 과거 비행 로그·검증 산출물은 로컬에 보관하지 않는다. 남아 있는 보고서는 당시 설명이며 원문 증거 파일이 현재 제공된다는 뜻이 아니다.

이 자료들은 CMU 서비스 실행 의존성이 아니다. CMU 프로파일 생성기와 실원본 발행 시험은 유지하지만, 실행하려면 [고정 원본을 다시 확보](cmu-runtime-profiles.md#재생성)해야 한다. NASA를 현재 서비스 선택지나 학습 입력으로 되살리지 않는다.
