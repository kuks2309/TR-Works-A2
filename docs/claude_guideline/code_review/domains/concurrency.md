# 코드 리뷰 — 동시성 Add-on (Code Review — Concurrency / Threading / async)

> 코어 [review.md](../review.md) 의 도메인 sub-file. **관점: 동시성 코드 리뷰** — 동기화 객체·공유 상태·실행 컨텍스트 인벤토리/평가.

**트리거**: `threading`, `asyncio`, `std::thread`, `std::mutex`, `MutuallyExclusiveCallbackGroup`, `ReentrantCallbackGroup`, multi-callback ROS 노드, `@asynccontextmanager`, `await`, `async def`.

## 1. 인벤토리 추가 표

**B-1. 동기화 객체** — 컬럼: `객체`, `종류(Mutex/Lock/Event/Semaphore/Atomic/Condvar)`, `보호 자원`, `획득 위치`, `해제 위치`

**B-2. 공유 상태** — 컬럼: `변수`, `읽기 위치`, `쓰기 위치`, `변경 방식`, `공유 방식`, `보호 객체`(보호 없으면 `"비보호"`)

- **변경 방식**: 단순 대입 / **복합 read-modify-write(비원자적 — `x++`·`x += 1`·check-then-act)** / 컬렉션 변형(순회 중 append·erase)
- **공유 방식**: 복사(값 전달 = 안전) / 참조·포인터·공유 객체(교차 변경 = 보호 필요)

**B-3. 실행 컨텍스트** — 컬럼: `이름`, `종류(thread/task/coroutine/callback group/executor)`, `우선순위·executor`, `생성 위치`

## 2. 평가 추가 카테고리 (인라인 태그)

- `[race]` — 공유 가변 상태의 비보호 **변경(mutation)**. **복합 연산(`x++`·`x += 1`·check-then-act)은 단일 문장이라도 비원자적**(read-modify-write) → lock/atomic 필요. 한 스레드가 컬렉션 변형 중 타 스레드 순회 = 무효화. 참조로 넘긴 가변 객체의 교차 변경.
- `[deadlock]` — 다중 lock 획득 순서 일관성, 순환 의존
- `[timing]` — 콜백 차단성, 시간 budget, jitter 원인
- `[reentrant]` — 재진입 가능 콜백에서 비-reentrant 호출

## 3. 자체 점검 grep

```bash
TARGET=docs/code_review/<주제>.md

# 동기화 객체 / 공유 상태 표 헤더
grep -E "^\| 객체 .*종류.*보호 자원|^\| 변수 .*읽기 위치.*쓰기 위치" $TARGET

# 동시성 평가 태그 등장
grep -oE "\[(race|deadlock|timing|reentrant)\]" $TARGET | sort -u

# 비보호 공유 상태 표기
grep -E "비보호" $TARGET

# mutation 원자성 점검 (복합 연산 비원자성)
grep -E "비원자적|read-modify-write|복합 연산|변경 방식" $TARGET
```

## 4. 다른 도메인과의 의존/충돌

- **ros2-review 와 상보** — multi-threaded executor·콜백 그룹 평가는 본 도메인의 race/deadlock 과 함께 적용.
- **embedded-review 와 상보** — ISR↔Task 공유 자원은 본 도메인의 동기화 객체 표 + embedded 의 `[prio]`/`[volatile]` 와 cross-reference.
