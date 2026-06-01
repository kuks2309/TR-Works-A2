# 코드 리뷰 산출물

코드 리뷰 / 코드 분석 요청의 결과를 주제별로 기록한다. 방법론·형식의 단일 근원은 [../claude_guideline/code_review.md](../claude_guideline/code_review.md).

## 파일 구조

- 주제별 파일 1개: `<주제>.md` (`<주제>` = 대상 파일명 / 모듈명 / 패키지명)
- 동일 주제 재리뷰 시 기존 파일에 prepend (최신 위, 시간 역순)

## 기록 형식

[../claude_guideline/code_review.md](../claude_guideline/code_review.md) §기록 위치 / 템플릿 의 템플릿을 따른다. Core 인벤토리 5 항목(목적·플로우·함수·전역·의존성) + 감지된 도메인 Add-on(ROS2/동시성/임베디드) + severity 클러스터 평가.
