# GitHub 워크플로

Git / GitHub 작업 규칙의 단일 근원.

## "기록" 명령 처리

사용자가 "기록" 또는 "기록해" 등의 지시를 내리면 다음을 수행한다.

1. 변경 파일 staging (`git add <명시 파일>`)
2. 커밋 메시지 작성 (작업 단위별 1 커밋)
3. 커밋 생성 (`git commit`)
4. **push 는 별도 명시 확인 후 수행** (`git push` 자동 실행 금지)

자동 push 는 사고 비용이 크므로 commit 과 분리한다. push 가 필요하면 사용자 명시 지시("푸쉬", "push" 등)를 별도로 받는다.

## 작업 단위 = 커밋 단위 = push 단위

서로 다른 작업의 변경을 한 커밋/푸쉬에 섞지 않는다.

- `git add` 직전·직후 `git status --short` 로 사전 staged 된 무관 파일 확인.
- 무관한 파일이 staged 면 `git restore --staged <file>` 로 분리.
- commit 직전 `git diff --cached --name-only` 로 staged 범위 마지막 검증.

다중 scope 가 누적된 dirty tree 는 scope 별 commit 으로 분할한다.

```bash
# scope 별 분할 예시
git add <scope-A 경로>
git commit -m "<type>(<scope-A>): ..."
git add <scope-B 경로>
git commit -m "<type>(<scope-B>): ..."
```

자동 staging(`git add -A`, `git add .`) 사용 금지. 항상 명시 경로로 add 한다.

## Read-only 외부 vendored 저장소 가드

다음 유형의 경로는 외부 공식 저장소에서 가져온 vendored 코드. **수정 금지, config / launch overlay 만 변경 허용**.

- 외부 SDK / 드라이버 (예: 센서 공식 드라이버, 모터 SDK)
- 외부 ROS 패키지 fork
- 시스템 설치 라이브러리 (`/usr/local/lib` 등)
- 의도적으로 빌드 제외된 레거시 패키지(`COLCON_IGNORE` 등)

수정이 불가피한 경우 wrapper 패키지 또는 launch / config overlay 로 처리하고, 변경 사유를 commit 메시지 본문에 명시한다.

프로젝트별 구체 경로 목록은 워크스페이스 루트 `CLAUDE.md` 또는 [`local/`](local/) 의 override 문서에서 정의한다.

## 일반 Git 작업 규칙

- 사용자 명시 요청 없이 commit/push 수행 금지 ("기록" 만 예외, 단 push 는 별도 확인).
- 코드 수정 후에는 staging 까지만 수행하고 commit 은 사용자 지시("기록" 또는 "커밋")로 진행.
- 파괴적 명령(`git push --force`, `git reset --hard`, 브랜치 삭제, `git clean -f` 등)은 사용자 명시 승인 후에만 실행.
- 외부 vendored 저장소(read-only 가드 목록)에는 절대 push 하지 않는다.

## 커밋 메시지

- 권장 형식: `<type>(<scope>): <subject>`
- 타입 예: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`, `build`
- 본문에 변경 사유와 관련 이슈 번호(있다면) 명시 권장.
- 형식 강제는 아니나 자동 changelog 생성에 유용.

## Push 전 확인

- 변경 파일이 사용자 요청 범위 내인지 재확인
- 비밀 정보(`.env`, 키, 토큰, 인증서, 사설 IP/MAC, 운영 환경 endpoint)가 포함되지 않았는지 확인
- 대상 저장소가 의도한 저장소인지 (외부 vendored 원본 저장소가 아닌 본 워크스페이스 origin 인지) 확인
- read-only 가드 목록의 파일이 staged 되어 있지 않은지 확인
