# openarmx_gravity_comp 코드 리뷰 타임라인

대상: `openarmx_ws/src/openarmx_ros2/openarmx_gravity_comp` (중력 피드포워드 보상 노드 + Dynamics KDL 래퍼)

| 날짜 | 코드 버전 | Verdict | 핵심 |
| --- | --- | --- | --- |
| [2026-06-25](2026-06-25.md) | `19295f2` (main) | COMMENT | Medium 3 (enable_left/right 런타임 무시·g_scale default 불일치·hw 부호 숨은결합), Low 5 / Info 3 |

## 산출물

- [gravity_comp_documentation.md](gravity_comp_documentation.md) — 읽기용 종합 문서(개요·데이터 체인·플로우차트·부호 규약·결함 요약)
- [gravity_comp_flowchart.drawio](gravity_comp_flowchart.drawio) — 편집용 플로우차트 3페이지(① 초기화 / ② 런타임 핫패스 / ③ 파라미터 콜백), 흰 배경
- [2026-06-25.md](2026-06-25.md) — 코드 리뷰(함수표·전역·의존성·ROS2 QoS·severity)
