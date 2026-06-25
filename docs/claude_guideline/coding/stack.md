# 언어·스택 구체 규칙 (Stack Conventions)

> **본 파일은 지시용.** coding.md §4 가 위임하는 **언어·프레임워크·도구별 구체 선택**. self-contained — 본문 외 의존 0.

언어 무관 원칙은 `conventions.md`, 본 파일은 "이 프로젝트는 어느 언어/도구로 무엇을 쓴다"는 **구체 선택**을 고정한다.

## 1. 코드 포맷터 — 선택은 여기, 강제는 도구

포맷은 글 규칙이 아니라 **포맷터 도구**로 강제한다. 프로젝트는 언어별 포맷터를 **선택**하고 설정 파일로 고정하며, 준수는 `checks/format.sh`(`⟦CI:format⟧`)가 기계 검사한다.

| 언어 | 포맷터 | 설정 파일 | 예시 선택 |
| --- | --- | --- | --- |
| C/C++ | `clang-format` | `.clang-format` | **Microsoft** / Google / LLVM |
| Python | `black` (+`isort`) | `pyproject.toml` | black 기본 |
| JavaScript/TypeScript | `prettier` | `.prettierrc` | prettier 기본 |
| 공통(들여쓰기·줄끝·인코딩) | EditorConfig | `.editorconfig` | LF·UTF-8·스페이스 |

- **선택은 프로젝트 1회 결정 후 고정**(드리프트 금지). 스타일 변경은 ADR(Architecture Decision Record, 설계 결정 기록) + 전체 재포맷 1커밋.
- 포맷터·설정이 없으면 `format.sh` 는 검사 생략(강제력 0) — 그 환경은 포맷 강제 없음을 정직히 인정.

예) C++ 프로젝트가 Microsoft 스타일을 쓰려면 루트에 `.clang-format` 한 줄:

```yaml
# .clang-format
BasedOnStyle: Microsoft
```

## 2. 언어·런타임 (프로젝트별 작성)

(프로젝트가 쓰는 언어·표준 버전·빌드 도구를 여기 명시. 예: C++17 / colcon, Python 3.10 / uv.)

## 3. 프레임워크·UI (프로젝트별 작성)

- UI 스택(예: Qt `.ui` 파일)은 별도 생성·갱신 규칙이 필요하면 여기 또는 `domains/` 로 둔다.
- (프레임워크별 관습은 프로젝트가 채운다.)

## 4. 언어 결합 (Python ↔ C/C++) — 트리거 시

서로 다른 언어를 결합하면(예: Python 에서 C/C++ 호출) 경계에서 사고가 난다. 결합을 쓰면 본 절 활성, 안 쓰면 면제.

**결합 방법 선택** (1회 고정, 변경은 ADR):

| 방법 | 용도 |
| --- | --- |
| `ctypes` | 순수 C `.so`/`.dll` 런타임 호출, 빌드 단계 없음 |
| `cffi` | C 선언 기반 FFI(Foreign Function Interface, 외부 함수 인터페이스), ctypes 보다 안전 |
| **pybind11** | C++ ↔ Python (모던, 헤더 온리) |
| Cython | Python 유사 코드를 C 로 컴파일 |
| Python C API | CPython 직접(최저수준, 최후 수단) |

(ROS2 의 rclpy↔rclcpp 는 '결합'이 아니라 DDS(Data Distribution Service)로 분리 통신 — 본 절 비대상.)

**경계 규약** (체크리스트):

- [ ] **메모리 소유권**: 경계를 넘는 버퍼는 **소유자 단일 지정** + 생명주기 명시 (C 가 먼저 `free` → use-after-free 금지; Python 은 `Py_buffer` 참조로 GC(Garbage Collection) 보호).
- [ ] **GIL 해제**: 오래 도는 C 호출은 `Py_BEGIN_ALLOW_THREADS` 로 GIL(Global Interpreter Lock) 해제 → `domains/concurrency-coding.md`(concurrency aspect) 참조.
- [ ] **타입/시그니처**: `ctypes` 는 런타임만 체크 → 시그니처 불일치 = segfault. `argtypes`/`restype` 명시 + 경계 테스트.
- [ ] **에러 전파**: C 반환코드/`errno` ↔ Python exception 변환 규약 고정 (C 는 예외 못 던짐).
- [ ] **데이터 마샬링**: numpy ↔ C 버퍼는 buffer protocol(zero-copy), struct 정렬·패킹·엔디안 양쪽 일치.
- [ ] **빌드·ABI**: `.so` 빌드·링크·크로스컴파일을 `setup.py`/CMake/colcon 으로 일원화.

**공유 메모리 규약** (race 주 진원지): 같은 프로세스(Python↔C zero-copy)든 프로세스 간(`mmap`·POSIX shm·`multiprocessing.shared_memory`·ROS2 zero-copy)이든 —

> ★ **공유 가변 메모리는 단일 writer + 생명주기 소유자 명시. 다중 writer 면 동기화(lock/atomic).**
> → concurrency aspect + coding.md §6 **전역변수표 "누가 바꾸나" 칸**에 그 writer 를 기록한다.

**경계 = 공개표면**: 언어 경계는 공개 인터페이스라 coding.md **§3 사전승인 트리거**(공개 API) 대상 — 소유권·에러규약·빌드를 ADR 에 기록.

## 자체 점검

```bash
# 포맷터 설정 존재 여부 (선택했는지)
ls .clang-format .editorconfig pyproject.toml .prettierrc* 2>/dev/null || echo "(포맷터 미선택 — format 강제 없음)"
# 포맷 준수 검사
bash docs/claude_guideline/coding/checks/format.sh .
```

---

**VERSION**: 1.0.0 (포맷터 선택 표 + EditorConfig, format-check 기계 강제 연계; 언어·프레임워크·UI 절은 프로젝트별 작성)
