# 수치·기하 코딩 (Numeric & Geometry Coding) — 횡단 aspect

> coding.md 가 위임하는 수치·변환 수학 규칙. **aspect(횡단)** — 로봇/제어의 #1 결함군(좌표계·단위·행렬·부동소수)이 무주공산이 되지 않도록. self-contained — 본문 외 의존 0.

## 트리거 (활성 조건)

변환 수학·기하 연산을 쓰면 활성, 아니면 면제:

- 행렬/벡터 연산 · 좌표 변환(TF) · 쿼터니언/오일러/회전행렬 · 단위 변환(rad/deg, m/mm)

## 1. 좌표계·단위 명시

- **모든 기하량에 프레임·단위를 이름·주석으로 명시** (예: `pose_base_link_m`, `angle_rad`).
- 단위 변환(rad↔deg, m↔mm)은 경계에서 한 번, 내부는 단일 단위.

## 2. 회전·변환

- **행렬 곱 순서·규약**(row/column-major, 능동/수동)을 ADR 로 고정 — 섞으면 조용히 틀린다.
- 쿼터니언은 사용 전 **정규화**(normalize). 오일러는 **축 순서**(예: ZYX) 명시. 변환 **방향**(A→B vs B→A) 명시.
- 짐벌락·특이점 주의(오일러 대신 쿼터니언 권장).

## 3. 부동소수

- **`==` 비교 금지** — `abs(a-b) < epsilon`. epsilon 은 스케일에 맞게.
- NaN/Inf 발생 경로(0 나눗셈·acos 범위 초과) 방어. 0 나눗셈·`acos` 입력 clamp.
- 누적오차: 반복 변환은 주기적 재정규화. 큰 수·작은 수 혼합 합산 주의.

## 4. 다른 맥락과의 연결 (cross-ref)

- ROS2 TF 배관(frame_id·tf2)은 → `domains/ros2-coding.md` (수학은 본 파일, 배관은 ros2).
- 임베디드 고정소수점·정수 오버플로는 → `domains/embedded-coding.md` 와 공동.

## 5. 강제

대부분 `⟦권고⟧`(수학 정합성은 정적 검출 한계). 단위/프레임 규약은 명명·ADR 로, 변환 정확성은 테스트(왕복 변환 == 항등)로 검증 → `tests-ran` 이빨.

## 자체 점검

```bash
grep -rEl 'quaternion|Quaternion|euler|rotation|np\.dot|matmul|tf2|transform' . >/dev/null 2>&1 \
  && echo "수치/기하 — aspect 적용" || echo "(수치 연산 없음 — 면제)"
```

---

**VERSION**: 1.0.0 (좌표계·단위 명시 + 행렬순서/쿼터니언 정규화·방향 ADR + 부동소수 epsilon/NaN/누적오차; ros2·embedded cross-ref; 왕복변환 테스트 검증)
