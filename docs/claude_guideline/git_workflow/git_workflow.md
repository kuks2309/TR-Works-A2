# Git 커밋·푸시 워크플로 (Git Commit & Push Workflow)

> **본 파일은 지시용.** Git commit/push 와 다중 작업자 협업 규칙의 self-contained 단일 근원(SSOT / Single Source of Truth).

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill 의존 0.

## 설치

본 번들 폴더(`git_workflow/`)의 `install.sh` 로 설치한다:

```bash
cd git_workflow && ./install.sh <타깃-프로젝트-루트>
```

스크립트가 `git_workflow.md` 를 `docs/claude_guideline/git_workflow/` 로 복사하고 등록 스니펫을 타깃 `CLAUDE.md` 에 append 한다. **활성화 게이트**: 본 파일이 그 경로에 없으면 본 룰 비활성.

## 0. 모드 판정 (solo vs team)

push·리뷰 방식이 모드에 따라 다르므로 작업 전 먼저 판정한다. 우선순위:

1. **명시 선언 (최우선)** — 타깃 `CLAUDE.md` 의 `git 협업 모드: solo|team` 줄. 있으면 그대로 따른다.
2. **자동 감지 (선언 없을 때)** — 하나라도 해당하면 **team**:
   - **GitHub collaborator 2명 이상 (가장 권위 있음)**: `gh api repos/<owner>/<repo>/collaborators --jq 'length'`. collaborator 는 "작업 권한이 있는 사람" 전체라 초기 단계에서도 정확. **원격마다 다를 수 있으니 각 원격을 확인**한다.
   - `CODEOWNERS` 파일 존재, 또는 `main` 브랜치 보호 활성 (`gh api repos/<owner>/<repo>/branches/main/protection`)
   - (gh 불가 시 fallback) 최근 50 커밋 author 이메일 2개 이상: `git log -50 --format='%ae' | sort -u | wc -l`
   - 사용자가 "팀/회사/공유/여러 명" 맥락 언급
3. **기본값** — 위 신호 없으면 **solo**. 단 공유 저장소면 `CLAUDE.md` 에 `git 협업 모드: team` 명시 권장.

**원격별 모드 (미러 주의)** — 미러 원격마다 collaborator 가 다르면 각 원격은 그 모드를 따른다. 한 원격이라도 team 이면 그 원격 `main` 직접 push 가 팀 PR·리뷰·권한 정책을 우회하지 않는지 확인한다. **단, 관리자가 운영하는 단방향 미러 원격**(팀의 활성 개발 대상이 아님)은 직접 push 를 **문서화된 예외**로 허용할 수 있다 — 누가 관리자이고 어느 원격이 미러인지 명시한다. (예: `origin`=solo, `fito`=collaborator 다수지만 관리자 단방향 미러 → `main` 직접 push 예외.)

판정이 모호하면 사용자에게 1줄 확인.

## 1. 공통 규칙 (solo·team 둘 다)

- **명시 요청에만 commit/push** — "커밋"/"푸쉬"/"커밋 푸쉬" 자연어 트리거. 그 외 자동 commit/push 금지.
- **작업 단위 = 커밋 = push 단위** — 서로 다른 scope 변경을 한 커밋에 섞지 않는다. dirty tree 는 scope 별 분할 커밋.
- **명시 staging** — `git add <명시 경로>`. `git add -A` / `git add .` **금지**. commit 직전 `git diff --cached --name-only` 로 범위 검증.
- **커밋 메시지** — `type(scope): subject` (`feat`·`fix`·`docs`·`refactor`·`style`·`chore`·`test`). 한국어 본문 허용. `Co-Authored-By` 푸터.
- **파괴 명령 승인** — `git push --force`·`reset --hard`·`clean -f`·브랜치 삭제는 사용자 명시 승인 후에만.
- **push 전 확인** — secrets(`.env`·키·토큰·사설 IP(Internet Protocol)/MAC(Media Access Control)·운영 endpoint) 미포함, 대상 저장소 정확, vendored read-only 가드 파일 미staged.
- **다중 원격 미러** — 미러 원격이 여럿이면 **모두** push (예: 본 저장소 `origin` + `fito`).

## 2. solo 모드 (혼자)

- `main`(또는 현재 추적 분기) **직접 commit + push**. PR(Pull Request) 미사용.
- 다중 원격이면: `git push origin main && git push fito main`.

## 3. team 모드 (여럿)

- **`main` 직접 push 금지.** `<type>/<topic>` 브랜치에서 작업(예: `feat/git-workflow`), 짧게 유지.
- **작업 전 최신화** — `git fetch origin` 후 `git switch -c <branch> origin/main`. 진행 중 `git pull --rebase origin main` 로 main 흡수.
- **PR 워크플로** — 브랜치 push → PR 생성 → **리뷰 ≥1 승인 → merge**. 리뷰는 `code_review` 번들 SOP 적용.
- **작성자 self-approve / self-merge 금지** — 별도 작업자가 승인(code_review 의 self-`APPROVE` 금지와 동일 원칙).
- **충돌·force** — 로컬에서 rebase 로 해결. 공유 브랜치 force-push **금지**, 자기 feature 브랜치만 `--force-with-lease` 허용.
- **merge·정리** — squash 또는 merge commit(팀 합의), merge 후 브랜치 삭제.
- **다중 원격 + 팀** — `origin` 이 협업 기준(PR·리뷰), `fito` 는 미러 — merge 후 `git push fito main` 로 동기화.

## 4. GitHub 정책 강제 (team — 선택)

CLAUDE.md/README 규칙은 **권고**이고, GitHub 설정은 **강제**한다. 사람이 실수로 `main` 에 직접 push 하는 것까지 막으려면 team 저장소에 다음을 건다:

- **브랜치 보호** — `main` 에 PR 필수 + 리뷰 ≥1 승인 + 직접 push 차단. 설정/확인:
  ```bash
  gh api repos/<owner>/<repo>/branches/main/protection            # 현재 상태 확인
  gh api -X PUT repos/<owner>/<repo>/branches/main/protection ...  # PR·리뷰 필수 설정
  ```
- **CODEOWNERS** (`.github/CODEOWNERS`) — 경로별 자동 리뷰어 지정 → PR 에 리뷰 자동 요청.
- **PR 템플릿** (`.github/pull_request_template.md`) — 변경 요약·테스트·관련 이슈 체크리스트.

**관리자 단방향 미러 예외** — 미러 원격(예: `fito`)은 브랜치 보호의 "allow specified actors to bypass"(또는 admin bypass)로 **관리자 직접 push 는 허용**하면서 일반 팀원은 PR 을 강제한다 → §0 미러 예외와 양립.

> 브랜치 보호·CODEOWNERS 설정 변경은 저장소 **admin 권한 + 사용자 명시 승인** 후에만 (공유 팀 워크플로에 영향).

## 룰 (요약)

1. 명시 요청에만 commit/push (트리거 "커밋"/"푸쉬")
2. 작업 단위 = 커밋 단위, 명시 staging(`-A`/`.` 금지)
3. `type(scope): subject` + `Co-Authored-By`
4. 다중 원격이면 모두 push
5. 파괴 명령은 명시 승인
6. push 전 secrets·대상·vendored 확인
7. **team 모드: `main` 직접 push 금지 → 브랜치 + PR + 리뷰 승인 + merge**
8. 작성자 self-approve/self-merge 금지

## 자체 점검

```bash
# 모드 선언 확인 (team 공유 저장소 권장)
grep -E "git 협업 모드: (solo|team)" CLAUDE.md || echo "(모드 미선언 — 자동 감지)"

# 모드 자동 감지 — GitHub collaborator 수 (≥2 → team), 원격별 확인
for r in $(git remote); do
  url=$(git remote get-url "$r"); slug=$(echo "$url" | sed -E 's#.*github.com[:/]([^/]+/[^/.]+)(\.git)?#\1#')
  echo "$r: $(gh api "repos/$slug/collaborators" --jq 'length' 2>/dev/null || echo '?') collaborators"
done
# (gh 불가 시) 최근 author 수
git log -50 --format='%ae' 2>/dev/null | sort -u | wc -l

# 커밋 메시지 형식 (마지막 커밋)
git log -1 --format='%s' | grep -E "^(feat|fix|docs|refactor|style|chore|test)(\([^)]+\))?: "
```

---

**VERSION**: 1.0.0 (solo + team 모드, collaborator 자동 감지, 다중 원격 미러, GitHub 정책 강제(선택), code_review 리뷰 게이트 연계)
