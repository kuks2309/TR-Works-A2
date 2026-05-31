# Changelog

## 1.4.1 — 2026-05-07

표현 수정 — `skills.md` 를 `skill_update.md` 로 rename. "스킬 목록" 으로 오해할 여지를 줄이고 "스킬 갱신·등록 절차" 의미를 명시화. 내용은 동일.

- `skills.md` → `skill_update.md`
- `README.md`, `templates/CLAUDE.md.template` 의 링크 갱신

## 1.4.0 — 2026-05-07

신규 워크스페이스 자산(스킬 / hook / 가이드라인 / 템플릿)이 SSOT 스킬 레포에 누락되지 않도록 등록 절차를 메타 규칙으로 표준화.

### 추가된 파일

- `skills.md` 신규 추가 — 스킬 / 자동화 자산 SSOT 등록 규칙 (적용 대상 5 종, 등록 절차 6 단계, 워크스페이스↔SSOT 우선순위, 비공개 자산 처리, deprecate 정책)

### 보강된 파일

- `README.md`: 진입점 표에 `skills.md` 행 추가
- `templates/CLAUDE.md.template`: 메타 규칙 진입 링크에 `skills.md` 행 추가

### 신규 정책 요약

- 워크스페이스에서 만든 신규 스킬 / hook / 가이드라인 보강 / sub-agent / 도메인 템플릿은 SSOT 레포(`kuks_claude_setup`)에 등록한다.
- 등록 전 워크스페이스 검증 필수, 비공개 / 환경 의존 부분은 placeholder 또는 `local/` 처리.
- SSOT 가 단일 근원이며 워크스페이스 측 직접 수정 후 SSOT 갱신을 잊는 패턴 금지.

## 1.3.0 — 2026-05-05

ROS2 + 임베디드 + 모듈 CLAUDE.md override 계층을 가진 워크스페이스에서 운영하면서 발견된 규칙 보강. FITO AMR ROS2 워크스페이스 배포에서 검증된 변경분 contributions.

### 추가된 파일

- `ros2.md` 신규 추가 — ROS2 + 임베디드 결합 환경의 도메인 SSOT (빌드 / src 원본 / COLCON_IGNORE / vendored read-only / 시리얼 함정 / 패키지 종류별 주의)
- `manual.md` 신규 추가 — 외부 벤더 매뉴얼·데이터시트 보관·인용·검증 규칙 (추정 금지, 실측 검증, NDA / 라이선스 처리)
- `local/README.md` 신규 추가 — 프로젝트별 비공개 override 폴더 패턴 (하드웨어 IP / 사설 네트워크 / read-only 경로)

### 보강된 파일

- `workflow.md`:
  - 시작 7 항목 / 종료 8 항목 체크리스트로 확장
  - 임베디드 도메인의 펌웨어 다운로드(플래시) 절차 추가 — 포트 점유 확인 → 부트모드 → 플래시 → 실패 시 진단 우선순위
  - 보고 형식을 "매 답변 강제" 에서 "분기 시점에만" 으로 완화 (사전 승인 트리거 / workaround / "기록" / 범위 외 변경)
- `tech_debt.md`:
  - 실시간 / 임베디드 시스템에서 정공법이 특히 중요한 이유 명시
  - 하드웨어 quirk 우회 강화 조항 (벤더 펌웨어 errata / 외부 SDK / 외부 의존성 알려진 버그)
  - TODO 형식 표준화: `// TODO(YYYY-MM-DD): <할 일> [참조]` + 30 일 정리 룰
- `coding.md`:
  - **상수 분리 원칙** 신규 — 의미가 다른 두 값이 우연히 같을 때 한 상수로 합치지 않기 (silent bug 방지)
  - 사전 승인 트리거 5 항목 명시 (패키지 신규 / 외부 인터페이스 변경 / 빌드 시스템 / 하드웨어 인터페이스 / 데이터 스키마)
  - 사전 승인 없이 진행 가능한 변경 명시 (단일 파일 버그 / 파라미터 추가 / 내부 리팩터)
  - 코딩 스타일 섹션 추가 — `.clang-format` 등 저장소 설정 우선, 모듈 CLAUDE.md 가 워크스페이스 가이드보다 우선
  - 보고 양식을 "매 답변" 에서 "변경 분기에서만" 으로 완화
- `github.md`:
  - "기록" 명령에서 push 자동 실행 분리 — push 는 별도 명시 확인 후 수행
  - Read-only 외부 vendored 저장소 가드 섹션 신규 (외부 SDK / 시스템 라이브러리 / COLCON_IGNORE)
  - scope 별 commit 분할 예시 + 자동 staging(`git add -A`/`.`) 금지 명시
  - Push 전 확인에 사설 IP/MAC/endpoint, vendored 원본 저장소 오염 방지 추가
- `documentation.md`:
  - 모듈 CLAUDE.md 가 워크스페이스 가이드를 복제하지 않는다는 SSOT 규칙 명시
  - 도메인 식별자(토픽 / 노드 / 명령 / 레지스터 / 핀맵) 원문 유지 명시
- `README.md`: 진입점 표에 `ros2.md`, `manual.md`, `local/`, `CHANGELOG.md` 행 추가

### 신규 정책 요약

- 모듈 CLAUDE.md override 계층 — 모듈 CLAUDE.md 의 모듈 고유 규칙(핀맵·상수·하드웨어 명령)이 워크스페이스 가이드라인보다 우선
- "기록" → 자동 push 분리
- 보고 형식을 "매 답변 강제" → "분기 시점에만" 으로 완화

## 1.2.0 — 2026-05-01

- `tech_debt.md` 신규 추가 — 기술 부채 방지 원칙 (정공법 우선)
  - 핵심 원칙: 시간이 더 걸리더라도 근본 원인 해결, workaround 금지
  - 우회 사용 시 3가지 조건 (비용/리스크 제시 + 사용자 승인 + 정리 일정 기록) 모두 만족 필수
  - 시간 트레이드오프 보고 의무
  - 임시·진단 코드 정리 / TODO 코멘트 정책 / ADR Open Question 30일 재평가 정책
  - `coding.md` "회피 대안 절대 금지" 와 충돌 시 본 규칙 우선
- `README.md`: 진입점 표에 `tech_debt.md` 행 추가

## 1.1.0 — 2026-04-30

- `github.md`: "커밋·푸쉬는 작업 단위로 분리" 섹션 추가
  - 작업 단위 = 커밋 단위 = push 단위 원칙
  - staged 범위 검증 절차 (`git status --short`, `git diff --cached --name-only`)

## 1.0.0 — 2026-04-30

- 초기 릴리스
- `claude_guideline/` 5 개 파일 추가: `README`, `github`, `coding`, `workflow`, `documentation`
- `install.sh`, `update.sh`, `VERSION`, `CHANGELOG.md` 추가
- `templates/CLAUDE.md.template` 추가 (프로젝트별 CLAUDE.md 골격)

## 정책

- **major** (X.0.0): 기존 규칙과 호환 안 됨, 수동 마이그레이션 필요
- **minor** (X.Y.0): 규칙 추가, 호환됨
- **patch** (X.Y.Z): 오탈자, 표현 수정
