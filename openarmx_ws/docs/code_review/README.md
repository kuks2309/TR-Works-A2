# Code Review 산출물

본 디렉토리는 `docs/claude_guideline/code_review.md` SOP 에 따라 작성된 코드 리뷰 문서를 모은다.

## 룰

- 파일명 = `<주제>.md` (대상 파일·모듈·패키지명)
- 동일 주제 기존 파일 → prepend (최신 위, 시간 역순)
- 같은 시각의 사용자 지시 entry 가 `docs/user_instructions/user_instructions.md` 에 있으면 제목 매핑

## 인덱스

| 주제 | 최신 리뷰 시각 (KST) | Verdict | 비고 |
|------|------------------|---------|------|
| [cyclo_control](cyclo_control.md) | 2026-05-23 08:09 | REQUEST CHANGES | 패키지 6개·노드 10개 전체 SOP 리뷰 (Core + ROS2 + 동시성). Reviewer lane(`code-reviewer`) Verdict=COMMENT, REQUEST CHANGES 유지. 22 항목: H2 / M11 / L6 / I3 |
