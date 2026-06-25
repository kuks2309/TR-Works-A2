# 동시성 (Concurrency) — 횡단 aspect

> coding.md 가 위임하는 thread/async 코딩 규칙. **aspect(횡단 관심사)** — 특정 플랫폼이 아니라 *동시 실행을 쓰는 모든 코드*에 적용. self-contained — 본문 외 의존 0. (리뷰 관점은 자매 `code_review` 의 `concurrency.md`, 본 파일은 *작성* 관점.)

## 트리거 (활성 조건)

thread/async 를 쓰면 본 aspect 활성, 아니면 면제(0 발화 정상):

- `threading`·`asyncio`·`async def` (Python) · `std::thread`·`std::mutex`·`std::atomic` (C++)
- ROS2 callback group·executor · 인터럽트/신호 핸들러(ISR)

## 1. 공유 상태 보호

- 공유/전역 **가변** 데이터는 **mutex/lock 으로 보호** — 맨손 동시 접근 금지.
- 가능하면 **단일 writer**(공유 가변 대신 메시지 전달·복사)로 설계해 보호 자체를 줄인다.
- 보호 대상 writer 는 coding.md §6 **전역변수표 "누가 바꾸나" 칸**으로 식별.

## 2. 잠금 규율

- **잠금 순서 일관**(여러 lock 은 항상 같은 순서로 획득) — 안 그러면 서로 기다리다 멈춤(deadlock).
- **lock 범위 최소**(임계구역 짧게), lock 보유 중 **blocking 호출·긴 작업 금지**.

## 3. 원자성

- "읽고-수정-쓰기" compound 연산(`counter++`, check-then-act)은 **mutex 또는 `atomic`** — 단일 문장도 비원자적일 수 있다.
- 조건변수/신호 대기는 **spurious wakeup** 대비(`while(조건)` 재확인, `if` 금지).

## 4. async / GIL

- `async` 안에서 **blocking 호출 금지**(이벤트 루프 정지) — coding.md §4 금지패턴과 연계.
- Python 에서 C 호출이 오래 돌면 **GIL(Global Interpreter Lock) 해제** → `stack.md` §4 참조.

## 5. 다른 맥락과의 연결 (cross-ref)

- **메모리**: 공유 메모리의 소유권·생명주기는 → `memory-coding.md`(단일 소유자 + use-after-free 금지).
- **언어 경계**: GIL·경계 버퍼는 → `stack.md` §4.

## 6. 강제 (도구)

race condition 은 대부분 grep 으로 못 잡는다 → 기본 `⟦권고⟧`. 단 **도구는 잡는다**:

- **ThreadSanitizer**(`-fsanitize=thread`) — 데이터 race 런타임 탐지 · valgrind `helgrind`
- → ThreadSanitizer 검사는 **향후 이빨**(`checks/concurrency.sh`) 후보 — 현재 `⟦권고⟧`. 구현 시 `⟦CI⟧` 로 승격(tsan 빌드 테스트).

## 자체 점검

```bash
# 활성화 게이트 (동시성 패턴 감지)
grep -rEl 'threading|asyncio|async def|std::thread|std::mutex|std::atomic|callback_group' \
  --include=*.py --include=*.cc --include=*.cpp --include=*.h . >/dev/null 2>&1 \
  && echo "동시성 사용 — concurrency aspect 적용" || echo "(동시성 없음 — 면제)"
```

---

**VERSION**: 1.0.0 (공유상태 보호 + 잠금 규율(deadlock) + 원자성 + async/GIL; 메모리·언어경계 cross-ref; ThreadSanitizer 도구 강제 후보; code_review concurrency 와 write↔review 상보)
