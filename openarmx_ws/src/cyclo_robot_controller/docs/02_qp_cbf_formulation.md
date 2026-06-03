# 02. QP / CBF 정식화 (Formulation)

[← 문서 목차](README.md) · [← 01. 아키텍처](01_architecture.md)

이 컨트롤러의 핵심은 **매 제어 주기마다 QP 한 번을 풀어 관절 속도 `qdot` 를 구하는 것**입니다.
관절 한계·자기 충돌은 **CBF(Control Barrier Function) 부등식 제약**으로,
물리적 해 불능을 막기 위해 **슬랙(slack) 변수 + 큰 페널티**로 완화합니다.

> 근거 코드:
> [`qp_base.hpp`](../cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp),
> [`open_manipulator_movel_controller.cpp`](../cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movel_controller.cpp),
> [`open_manipulator_movej_controller.cpp`](../cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movej_controller.cpp)

---

## 1. 표준형 (QPBase)

`QPBase::solveQP` 는 OSQP 표준형을 풉니다:

```
minimize    ½ xᵀ P x + qᵀ x
subject to  l ≤ A x ≤ u
```

- `P` (Hessian), `q` (gradient), `A` (제약 행렬), `l`/`u` (제약 상·하한)
- 파생 클래스가 순수가상함수 4개를 채워 문제를 구성:
  `setCost()`, `setBoundConstraint()`, `setIneqConstraint()`, `setEqConstraint()`
- `setConstraint()` 가 **[bound; inequality; equality]** 순서로 `A`/`l`/`u` 를 쌓습니다
  (`qp_base.hpp:201-227`).

### 솔버 효율화

- **warm-start 활성** (`setWarmStart(true)`), `verbose=false`.
- **희소 패턴 캐시**: Hessian/제약 행렬의 희소 패턴이 직전과 동일하면
  `updateHessianMatrix/updateLinearConstraintsMatrix/updateGradient/updateBounds` 로 **in-place 갱신**,
  패턴이 바뀌었거나 갱신 실패 시에만 솔버를 **재초기화**합니다(`qp_base.hpp:148-166`).
- 등식 제약은 `l = u = b` 로 표현 (단, 본 컨트롤러들은 등식 제약 미사용, `neqc = 0`).

---

## 2. 결정 변수 (Decision Vector)

OpenManipulator MoveL/MoveJ 는 동일한 변수 레이아웃을 사용합니다
(`QPIndex`, 생성자에서 인덱스 계산). `n = joint_dof`, `m = 충돌쌍 개수`:

| 블록 | 기호 | 크기 | 의미 |
| --- | --- | --- | --- |
| `qdot` | `q̇` | `n` | **관절 속도 (실제 출력)** |
| `slack_q_min` | `s⁻` | `n` | 관절 하한 CBF 완화 슬랙 |
| `slack_q_max` | `s⁺` | `n` | 관절 상한 CBF 완화 슬랙 |
| `slack_sing` | `s_sing` | `1` | 특이점 회피 슬랙 (※ 예약만, 아래 5절) |
| `slack_sel_col` | `s_col` | `m` | 충돌 회피 CBF 완화 슬랙 (쌍별 1개) |

- 총 변수 수 `nx = 3n + 1 + m`.
- **bound 제약 개수 `nbc = nx`** (모든 변수에 상·하한). `qp_base.hpp` 의 `assert(nbc == nx || nbc == 0)` 충족.
- **부등식 제약 개수 `nineq = 2n + 1 + m`** (하한 n + 상한 n + 특이점 1 + 충돌 m).
- **등식 제약 `neqc = 0`.**

---

## 3. 비용 함수 (Cost)

### 3.1 MoveL (`setCost`, movel:141-173)

태스크 야코비안 `J = ∂x/∂q ∈ ℝ^{6×n}` (`getJacobian(controlled_link)`),
태스크 가중치 `W = diag(task_tracking_weight) ∈ ℝ^{6×6}`,
감쇠 가중치 `D = diag(damping_weight) ∈ ℝ^{n×n}`,
목표 태스크 속도 `ẋ_d`:

```
P[q̇,q̇] += 2 Jᵀ W J + 2 D
q[q̇]    += −2 Jᵀ W ẋ_d
```

이는 다음을 최소화하는 것과 동치(상수항 제외):

```
‖ J q̇ − ẋ_d ‖²_W  +  q̇ᵀ D q̇
   └─ 태스크 속도 추종 ─┘   └ 감쇠(특이점·과대속도 억제) ┘
```

### 3.2 MoveJ (`setCost`, movej:127-155)

관절 추종 가중치 `Wⱼ = diag(joint_tracking_weight)`, 목표 관절 속도 `q̇_d`:

```
P[q̇,q̇] += 2 Wⱼ + 2 D
q[q̇]    += −2 Wⱼ q̇_d
```

→ `‖ q̇ − q̇_d ‖²_{Wⱼ} + q̇ᵀ D q̇` 최소화. 야코비안이 없는 순수 관절 공간 추종입니다.

### 3.3 슬랙 페널티 (공통)

모든 슬랙 변수에 선형 페널티 `slack_penalty`(기본 1000) 를 부여하여
제약 위반을 강하게 억제합니다(`q[s⁻]=q[s⁺]=q[s_sing]=q[s_col]=slack_penalty`).
슬랙은 `≥ 0` (4절) 이므로 선형 페널티만으로 0 방향 압력이 걸립니다.

---

## 4. 제약 (Constraints)

### 4.1 Bound 제약 (`setBoundConstraint`)

```
qdot       :  q̇_lb  ≤ q̇ ≤ q̇_ub        (모델 속도 한계 ±velocityLimit)
모든 slack :  0      ≤ s  ≤ +∞           (비음수)
```

`q̇_lb/q̇_ub` 는 `KinematicsSolver::getJointVelocityLimit()` (= `(−velocityLimit, +velocityLimit)`).

### 4.2 관절 위치 한계 — CBF 부등식

장벽 함수를 하한 `h⁻ = q − q_min ≥ 0`, 상한 `h⁺ = q_max − q ≥ 0` 로 두고,
이산 CBF 조건 `ḣ ≥ −α h` (α = `cbf_alpha`) 를 슬랙으로 완화한 형태:

```
하한:   I·q̇ + I·s⁻  ≥  −α (q − q_min)         (movel:204-217)
상한:  −I·q̇ + I·s⁺  ≥  −α (q_max − q)         (movel:219-232)
```

코드상 `A_ineq` 에 `+I`(하한 행) / `−I`(상한 행) 과 각 슬랙의 `+I` 를 넣고,
하한(`l_ineq`)을 `−α(q−q_min)` / `−α(q_max−q)` 로, 상한(`u_ineq`)은 `+∞` 로 둡니다.
즉 `q̇` 가 관절 한계에 가까워질수록 한계 방향 속도가 `α·(여유거리)` 로 제한됩니다.

### 4.3 자기 충돌 회피 — CBF 부등식

각 충돌쌍 `i` 에 대해 거리 `dᵢ` 와 그라디언트 `∇dᵢ = ∂dᵢ/∂q` 를
`getCollisionPairDistances(with_grad=true, …)` 로 구해(아래 [03_controllers.md](03_controllers.md) §운동학):

```
∇dᵢᵀ·q̇ + s_col,i  ≥  −α (dᵢ − d_safe)      단,  dᵢ ≤ collision_buffer 일 때만 활성
```

- 활성 조건: **현재 거리 `dᵢ` 가 `collision_buffer` 이내일 때만** 하한을 설정합니다
  (`if (res.distance <= collision_buffer_)`, movel:249-252). 멀리 있으면 하한 `−∞`(비활성)이라 무비용.
- `d_safe = collision_safe_distance` 는 유지하려는 최소 안전거리.
- 의미: 두 링크가 가까워지면(`ḋ ≈ ∇dᵀq̇`) 접근 속도를 `−α(d−d_safe)` 이상으로 제한 → 충돌 직전 감속.

### 4.4 등식 제약

`setEqConstraint` 는 `A_eq`/`b_eq` 를 0 크기로 둡니다(`neqc = 0`). 사용되지 않습니다.

---

## 5. ⚠️ 특이점(singularity) 슬랙은 현재 "예약"만 됨

코드 사실을 정확히 기록합니다.

- 변수 `slack_sing`(크기 1)과 부등식 행 `con_sing`(크기 1)은 **할당**되어 있고,
  비용에 `slack_penalty` 도 부여됩니다.
- 그러나 `setIneqConstraint()` 는 **`con_sing` 행의 `A_ineq`/`l_ineq` 를 채우지 않습니다.**
  따라서 해당 제약 행은 `[−∞, +∞]` 로 남아 **항상 만족(비활성)** 이며,
  `slack_sing` 는 어떤 제약에도 등장하지 않으므로 페널티에 의해 0 으로 수렴합니다.
- 결론: **특이점 회피는 자리만 마련된 미구현(placeholder) 기능**입니다. 실제 특이점 억제는
  비용 함수의 감쇠항(`2D`)이 간접적으로 담당합니다. (MoveL/MoveJ 동일)

---

## 6. 한 주기 풀이 요약

```
setCost()           → P, q  채움 (태스크/관절 추종 + 감쇠 + 슬랙 페널티)
setBoundConstraint()→ qdot 속도한계, slack ≥ 0
setIneqConstraint() → 관절한계 CBF(2n) + 특이점(1, 비활성) + 충돌 CBF(m)
setConstraint()     → A,l,u 조립 [bound; ineq; eq]
solveProblem()      → OSQP (warm-start)
getSolution()       → x,  그중 x[0:n] = q̇*  (getOptJointVel)
```

해 `q̇*` 는 ROS 노드에서 `q_cmd += q̇* · dt` 로 적분되어 위치 명령이 됩니다.
QP 가 풀리지 않으면(`Status != Solved`) `getOptJointVel` 은 `false` 와 영(0) 속도를 반환하고,
노드는 `controller_error` 토픽으로 실패를 보고합니다.

---

## 7. CBF / 파라미터 직관

| 파라미터 | 의미 | 키우면 | 줄이면 |
| --- | --- | --- | --- |
| `cbf_alpha` (α) | 한계/충돌 경계 접근 허용 공격성 | 경계까지 빠르게 접근(덜 보수적) | 일찍 감속(더 보수적·안전) |
| `collision_buffer` | 충돌 CBF 활성 거리 | 더 일찍 충돌제약 개입 | 더 늦게 개입 |
| `collision_safe_distance` (d_safe) | 유지 최소 거리 | 더 멀리 떨어져 정지 | 더 가깝게 허용 |
| `slack_penalty` | 제약 위반(슬랙) 비용 | 제약 거의 경성(hard)화 | 제약 더 잘 양보(연성) |
| `weight_damping` | 감쇠(속도/특이점 억제) | 더 부드럽지만 추종 느려짐 | 빠르지만 거칠어짐 |
| `weight_task_*` / `weight_joint_tracking` | 추종 충실도 | 정밀 추종 | 더 여유롭게 양보 |

다음: [03. 컨트롤러 & 운동학 →](03_controllers.md)
