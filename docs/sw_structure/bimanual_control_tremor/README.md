# 양팔(bimanual) 제어 떨림 — 구조 분석 타임라인

OpenArmX V10 양팔 로봇 저수준 제어 경로의 SW(Software) 구조 분석 + 떨림 원인 진단·해결방안 누적 인덱스 (날짜=버전, 최신 위).

| 날짜 | 코드 버전 | 핵심 |
| --- | --- | --- |
| [2026-06-29](2026-06-29.md) | `afa3b42` (main) | 양팔 = 단일 controller_manager에 좌·우 HW 구성→블로킹 CAN 직렬화. 떨림 주원인=모터 MIT PD 한계진동, 양팔 구조가 루프율↓로 증폭. P0 양팔 루프율 실측→P1 CAN 비동기+속도 LPF 제안 |

## 관련 문서
- [2026-06-07 JTC 떨림 진단](../../issues_and_fixes/2026-06-07_jtc_tremor_diagnosis.md) — 단일팔 rosbag 실측(MIT PD 한계진동 근거)
- [openarmx_gravity_comp 리뷰 2026-06-25](../../code_review/openarmx_gravity_comp/2026-06-25.md) — τ_ff 주입 경로(처짐 보상, 떨림 무관)
