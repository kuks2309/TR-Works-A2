# ROS2 코딩 (ROS2 Coding) — 도메인

> coding.md 가 위임하는 ROS2 *작성* 규칙(리뷰 관점은 자매 `code_review` 의 `ros2-review.md`). self-contained — 본문 외 의존 0.

## 트리거 (활성 조건)

ROS2 코드를 쓰면 활성, 아니면 면제:

- `package.xml` · `rclpy`/`rclcpp` · `*.launch.py` · `rcl_interfaces` · `.msg`/`.srv`/`.action`

## 1. QoS (Quality of Service) 일치

- **pub↔sub QoS 호환 필수** — 불일치 시 통신 두절(메시지 0). reliability(reliable/best-effort)·durability·history depth 를 맞춘다.
- 센서 스트림 = best-effort, 명령/상태 = reliable + transient_local(latched) 관례.

## 2. 콜백·실행기 (executor)

- **콜백 안에서 blocking·긴 작업 금지** — 다른 콜백 굶김(starvation). 무거운 일은 별도 스레드/타이머로.
- 동시 콜백은 **callback group**(MutuallyExclusive vs Reentrant)으로 제어 → `domains/concurrency-coding.md` 참조.

## 3. 노드·파라미터·수명주기

- 파라미터는 `declare_parameter` 로 명시(타입·기본값·범위). 미선언 접근 금지.
- 수명주기(lifecycle) 노드는 상태 전이(configure/activate/...)에서 자원 획득·해제.

## 4. 좌표계·인터페이스

- `frame_id`·TF 변환은 **프레임·단위 명시** → 변환 수학은 `domains/numeric-coding.md`.
- **인터페이스(.msg/.srv/.action) 변경 = 공개표면** → coding.md §3 사전승인 + ADR(호환성·버전).

## 5. 강제

대부분 `⟦권고⟧`(런타임 동작은 정적 검출 한계). 연계:

- QoS·인터페이스 점검 → `code_review` 의 `ros2-review`(별도 패스)
- 콜백 동시성 → concurrency aspect

## 자체 점검

```bash
grep -rEl 'rclpy|rclcpp|package\.xml|\.launch\.py' . >/dev/null 2>&1 \
  && echo "ROS2 — 도메인 적용" || echo "(ROS2 없음 — 면제)"
```

---

**VERSION**: 1.0.0 (QoS 일치 + 콜백 starvation/callback group + 파라미터·수명주기 + frame/인터페이스 ADR; concurrency·numeric cross-ref; ros2-review 와 write↔review 상보)
