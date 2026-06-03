# User Instructions

본 파일은 사용자 원문 보존 — 요약 / 해석 / 재구성 금지. KST 시각 + 시간 역순 (최신 위, prepend).

---

## 2026-06-03 16:18 KST — openarmx_pick 분석·문서화 + pick_and_place/cyclo/로 이동

> "폴더 /home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_pick 에 대해서 분석하고 /home/openarmx/TR-Works/kkw/China/openarmx_ws/src/openarmx_pick/docs 에 기록 팀 20명 최대 투입.."

> "/home/openarmx/TR-Works/kkw/China/openarmx_ws/src/pick_and_place 로 이동 해야 할 것 같은데 pick_and_place 에서 대상물에 따라서 다르니 .."

> "중복된 기능들이 많고 추후에 만든것이 더 효과적이 않나? 대상물만 action msg수정하면"

> "box_align이 의존하는 (B) solver + openarmx_movel_bimanual.launch.py 를 어디에 둘지입니다: <- 이것이 왜 필요?"

> "그러면 결국 planner sovler 가 cyclo  pilz가 아니라는 것인데"

> "지금처럼 패키지를 두 벌 두지 않음 <- 이건 나의 정책임 명확히 구분하려고 함"

> "이동 만 하고 나중에.... 진행"

> "관련 문서 깃 커밋 푸쉬"

> "재빌드"

> "기록, 커밋  푸쉬"

---

## 2026-06-03 13:40 KST — Yolov8 CPU 추론 속도 10Hz 검증 + 문서화

> "폴더 /home/openarmx/TR-Works/kkw/China/Yolo 에서 v8로 구현한  Yolov8 cpu 수행 속도를 검증해주세요."

> "10hz가 나온느지 검증하고 검정 결과를 /home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/docs 에 문서화 해주세요."

> "깃 커밋 푸쉬"

> "세션 종료 하도록 기록 깃 커밋 푸쉬 완료"

---

## 2026-06-03 12:58 KST — pick_and_place 백엔드별(cyclo/·pilz/) 하위그룹 정리

> "폴더 /home/openarmx/TR-Works/kkw/China/openarmx_ws/src/cyclo_robot_controller와 같이 pilz도 폴더를 만들어서 별도로 관리하는 것이 좋지 않을까? 의견 주세요."

> "pick_and_place/
>   cyclo/   openarmx_cyclo_box_align(_msgs)
>   pilz/    openarmx_pilz_box_align(_msgs) 으로 좋겠네요"

> "기록 후 킷 커밋 세션 종료"

---

## 2026-05-22 (KST) — external_reference_handling.md SSOT 신설 + v2 마이그레이션 Step 1-3

> "폴더 /home/amap/Study/ros2_3dslam_ws/docs 를 참조해서 manual.md 분석 부탁"

> "우리 새로운 /home/amap/Project/claude_code/kuks_claude_setup_new 에 적용하기 위해서 개선될 내용은?"

> "독립적으로 작동하게 .. 현재는 신규 생성이 다 독립적으로 수행이 가능하도록 작성하는데"

> "현재 모든 파일은 독립 SSOT 으로 ㅈ가성중입니다."

> "사용자가 \"QoS 원칙 추가\" 별도 언급 → 사례와 원칙이 같은 §에서 강화됨 <- 이건 코딩으로
> manual.md는 관련 도메인의 문서를 참조하는 것이아닌지?"

> "파일 이름부터가 직관적이지 못함"

> "A"

> "도메인에 따라서 다양한 manual이 존재 할 수 있으므로 이에 대한 보완이 필요함"

> "opencv 도 포함"

> "1"

> "a"

> "왜 omc skill ?"

> "진행 해줏ㅅ"

> "commit / push 는 별도 명시 승인"

> "커밋푸쉬"

> "커밋푸쉬  kuks_claude_setup_new/ 의 내용을 깃 커밋해야 함"

> "기존 것을  /home/amap/Project/claude_code/kuks_claude_setup_new로 교체하는 작업이니 이에 적합하도록 설정"

> "전부다 순서적으로 진행할 것"

> "완료 가 모두 되엇으면 세션 종료 진행하기 위햐서 기록, 깃 커밋 푸쉬 부탁"

---
