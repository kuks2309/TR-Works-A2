# 프로젝트별 Override (gitignore 대상)

본 폴더는 프로젝트 / 워크스페이스 고유의 비공개 운영 정보를 담는다. **GitHub 에 push 되지 않는다** (`.gitignore` 에서 제외).

## 용도

- 하드웨어 IP / MAC / 시리얼 번호
- 사설 네트워크 인터페이스 이름
- 비공개 운영 endpoint
- 모듈별 read-only 경로 구체 목록
- 사내 도구 / 인증 정보 참조 (값 자체는 비밀 관리 시스템에 두고, 본 폴더에는 위치만 명시)

## 사용 규칙

- 본 폴더의 파일은 git 에 commit 하지 않는다 (`.gitignore` 에 `docs/claude_guideline/local/` 또는 본 폴더의 위치를 추가).
- 본 README 만 예외적으로 commit 되어 폴더 존재와 사용법을 설명한다.
- 비밀 (.env 의 값, 토큰, 키) 은 본 폴더에도 두지 않는다 — 비밀 관리 시스템 또는 환경변수만 사용.
- 일반화 가능한 규칙이 발견되면 상위 [../README.md](../README.md) 에 승격한다.

## 권장 파일 구조

```
local/
├── README.md           # 본 파일 (commit 됨)
├── network.md          # 하드웨어 IP, 인터페이스
├── readonly_paths.md   # vendored 외부 저장소 구체 경로
└── modules.md          # 모듈별 CLAUDE.md 위치 + 하드웨어 매핑
```

본 워크스페이스에서 어떤 override 가 작성되어 있는지는 워크스페이스 루트 `CLAUDE.md` 에서 진입 링크로 안내한다.
