# 메모리 안전 (Memory Safety) — 횡단 aspect

> coding.md 가 위임하는 메모리 관리 규칙. **aspect(횡단 관심사)** — 특정 플랫폼이 아니라 *수동 메모리를 다루는 모든 코드*에 적용. self-contained — 본문 외 의존 0.

## 트리거 (활성 조건)

수동 메모리 관리 언어·패턴을 쓰면 본 aspect 활성, 아니면 면제(0 발화 정상):

- C/C++ `new`·`delete`·`malloc`·`free`·raw 포인터
- 직접 버퍼/배열 수명 관리, 포인터 산술

## 1. 소유권·생명주기

- 모든 할당에 **단일 소유자**를 지정한다 — 소유자가 해제 책임을 진다.
- **raw 포인터 = 비소유(non-owning)** 관례. 소유는 스마트 포인터/RAII(Resource Acquisition Is Initialization, 자원 획득=초기화)가 표현.
- 사용처가 소유자보다 **오래 살지 않게** 한다(dangling 방지).

## 2. 할당·해제 짝

- `new`↔`delete`, `new[]`↔`delete[]`, `malloc`↔`free` **짝 보장**(같은 소유 경계 안에서).
- **RAII / 스마트 포인터 우선**: `unique_ptr`(단독 소유)·`shared_ptr`(공유 소유)·표준 컨테이너. 맨손 `new`/`delete` 최소화.

## 3. 금지 (메모리 버그)

- 누수(leak) · 이중 해제(double-free) · use-after-free · dangling 포인터 · 미초기화 포인터 · 버퍼 오버런

## 4. 다른 맥락과의 연결 (cross-ref)

- **동시성**: 공유 메모리를 thread 가 동시 접근하면 → `concurrency-coding.md`(mutex/lock/atomic + 단일 writer).
- **언어 경계**: Python↔C 등 경계를 넘는 버퍼는 → `stack.md` §4(소유권·zero-copy·생명주기).
- **전역 변수**: 공유 가변 메모리의 writer 는 coding.md §6 **전역변수표 "누가 바꾸나" 칸**에 기록.

## 5. 강제 (도구)

메모리 버그는 대부분 grep 으로 못 잡는다 → 기본 `⟦권고⟧`. 단 **도구는 잡는다**:

- 테스트를 **AddressSanitizer**(`-fsanitize=address`) 빌드로 실행 → 누수·use-after-free·오버런 탐지
- **valgrind** leak 검사 · **clang-tidy** 정적분석(소유권·초기화)
- → `checks/memory.sh`(`⟦CI:memory⟧`)가 **clang-tidy 정적분석**(누수·use-after-free·new/delete·미초기화) + **asan 런타임**(`MEMORY_TEST_CMD` 지정 시)으로 검사·차단. 도구 없으면 graceful 생략.

## 자체 점검

```bash
# 활성화 게이트 (수동 메모리 패턴 감지)
grep -rEl '\bnew\b|\bmalloc\b|\bfree\b|\bdelete\b' --include=*.c --include=*.cc --include=*.cpp --include=*.h . >/dev/null 2>&1 \
  && echo "수동 메모리 사용 — memory aspect 적용" || echo "(수동 메모리 없음 — 면제)"
# (이빨 구현 시) asan 빌드 테스트
# bash docs/claude_guideline/coding/checks/memory.sh
```

---

**VERSION**: 1.0.0 (소유권·생명주기 + 할당/해제 짝 + RAII + 메모리버그 금지; 동시성·언어경계·전역변수표 cross-ref; asan/valgrind 도구 강제 후보)
