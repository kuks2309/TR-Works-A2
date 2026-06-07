# 작업 지시서 — 팔 중력보상 자유 구동(핸드 가이드)

> 대상 워크스페이스: **China 전용** (`/home/openarmx/TR-Works/kkw/China`).
> 작성: 2026-06-07 세션. 실행: 다른 세션에서도 그대로 사용 가능.
> 목적: 한 팔(좌/우)을 **손으로 자유롭게 포즈**할 수 있게 만들고(자세 시연·티칭용), 끝나면 안전하게 위치 제어로 **복원**한다.
>
> **⚠️ 안전 정책(필수): 자유 구동은 반드시 홈(INIT)에서 시작하고, 해제할 때도 반드시 홈으로 복귀한 뒤 끝낸다.** 임의 자세에서 kp 를 0 으로 풀면 큰 처짐/충돌 위험이 있으므로, ON 전에 `--home`, OFF(`--restore`) 시 자동 홈 복귀를 강제한다.

---

## 1. 원리

OpenArmX 하드웨어는 **MIT 모드**(모터 임피던스 토크 제어)로 동작한다. 각 모터의 명령 토크는:

```
tau = kp*(pos_cmd - pos) + kd*(vel_cmd - vel) + tau_ff
```

- `tau_ff` = **중력보상** 피드포워드. `{side}_forward_effort_controller` 가 `gravity_comp_node` 의 g(q) 를 발행 → 팔이 중력에 안 떨어진다.
- `kp*(pos_cmd - pos)` = **위치 강성**. 이게 살아 있으면 팔이 명령 위치를 잡으려 해서 손으로 못 움직인다.
- 따라서 **핸드 가이드 = kp 를 0 으로 + 중력보상(effort)은 유지 + 위치 컨트롤러(JTC) 비활성**.

핵심 컴포넌트:
- `{side}_joint_trajectory_controller` — JTC(Joint Trajectory Controller). 위치 제어. 자유 구동 시 **비활성**.
- `{side}_forward_effort_controller` — 중력보상 피드포워드. 자유 구동 중에도 **계속 active 유지**(언로드/비활성 시 팔이 떨어짐).
- `/openarmx_{side}_hardware_params` — kp/kd 동적 파라미터 노드. `kp_joint1..7` 을 0↔복원.
  - 공장값(복원): `kp_joint1..4 = 50`, `kp_joint5..7 = 10` (kd 는 건드리지 않음).

---

## 2. 사용법 (재사용 스크립트 — 권장)

스크립트: [`experiments/arm_freedrive.py`](../../experiments/arm_freedrive.py) — 홈 정책과 안전 순서를 코드로 고정. 모드 `--home / --enable / --restore`.

```bash
cd /home/openarmx/TR-Works/kkw/China
source /opt/ros/humble/setup.bash
source openarmx_ws/install/setup.bash

# 0) 홈(INIT) 이동 — 자유 구동 시작 전 필수
python3 experiments/arm_freedrive.py --side right --home
#   (필요 시 JTC 활성+kp 복원) -> INIT 궤적 -> 홈 도달 확인

# 1) 자유 구동 ON  ** 홈에 있을 때만 허용(아니면 중단) **
python3 experiments/arm_freedrive.py --side right --enable
#   - 중력보상(effort) active + 홈 여부(오차<=10°) 검사 -> 통과 시
#   - right_joint_trajectory_controller 비활성 -> kp_joint1..7 = 0
#   -> 손으로 자유 이동 가능

#   ... 손으로 원하는 자세를 잡고, read_arm_pose.py / save_grasp_reference.py 로 즉시 캡처 ...

# 2) 자유 구동 OFF + 홈 복귀
python3 experiments/arm_freedrive.py --side right --restore
#   - JTC 활성(현재자세 캡처) -> kp 복원(50/10) -> INIT 궤적 -> 홈 복귀
```

좌/우 독립. 한쪽만 자유 구동하고 다른쪽은 위치 제어 유지 가능. 홈(INIT) 정의는 `ptp_pick_seq_v2_left.py` 의 `INIT_DEG`(측면화) 를 그대로 사용한다(SSOT).

---

## 3. 사용법 (수동 CLI — 스크립트 없이)

```bash
# === ON: 오른팔 자유 구동 ===
# 1) JTC 비활성 (effort 는 유지)
ros2 control switch_controllers --deactivate right_joint_trajectory_controller
# 2) kp 0 으로
for i in 1 2 3 4 5 6 7; do ros2 param set /openarmx_right_hardware_params kp_joint$i 0.0; done

# === OFF: 복원 ===
# 1) JTC 먼저 활성 (현재자세 캡처)
ros2 control switch_controllers --activate right_joint_trajectory_controller
# 2) kp 복원
for i in 1 2 3 4; do ros2 param set /openarmx_right_hardware_params kp_joint$i 50.0; done
for i in 5 6 7;   do ros2 param set /openarmx_right_hardware_params kp_joint$i 10.0; done
```

좌팔은 `right`→`left`, `/openarmx_right_hardware_params`→`/openarmx_left_hardware_params`.

---

## 4. 순서가 중요한 이유 (안전)

- **홈에서만 ON / 홈으로 OFF (필수 정책).** 자유 구동은 반드시 홈(INIT)에서 시작한다(임의 자세 kp=0 은 큰 처짐·충돌 위험). `--enable` 은 홈 여부(오차≤10°)를 검사해 아니면 중단한다. 해제(`--restore`)는 위치 제어 복원 후 **자동으로 홈으로 이동**해 끝낸다. 별도 `--home` 으로 언제든 홈 복귀 가능.
- **ON: JTC 비활성 먼저 → kp=0 나중.** JTC 가 살아 있는데 kp 만 0 으로 두면, 복원 시 JTC 가 **이전(stale) setpoint** 로 kp 를 적용해 팔이 그쪽으로 **튄다(스냅백)**.
- **OFF(복원): JTC 활성 먼저 → kp 복원 나중.** JTC 는 활성 순간 **현재(손으로 잡아 둔) 자세를 setpoint 로 캡처**한다. 그 뒤 kp 를 올리면 현재자세에서 잡으므로 스냅백이 없다.
- **중력보상(effort)은 절대 언로드/비활성 금지.** kp=0 상태에서 effort 마저 끄면 팔이 **즉시 떨어진다**. (2026-06-07: effort 스포너 `--unload-on-kill` 결함으로 무력화된 전례 있음.)

---

## 5. 주의 / 한계

- **약간의 처짐**: 중력보상 `g_scale=0.95`(5% 부족보상)라 kp=0 에서 위치 피드백이 없어 자세가 서서히 처질 수 있다. **자세는 손으로 잡고 있는 순간에 즉시 읽어/저장**할 것(분 단위 방치 시 크게 흘러내림 — 실측: ~20cm 드리프트).
- **무적분 droop**: 복원 후에도 중력 부하가 큰 자세(어깨 들림)는 MIT PD(무적분)라 명령 대비 1~2°(때로 더) 정상상태 오차가 남는다.
- **선행 조건**: L1 컨트롤러 스택(controller_manager + JTC) + 중력보상(`gravity_comp_node` + `{side}_forward_effort_controller`)이 이미 떠 있어야 한다. Launch Manager 의 Gravity Comp 체크박스로 켤 수 있다.

---

## 6. 자세 읽기 / 저장 (티칭 보조 도구)

- [`experiments/read_arm_pose.py`](../../experiments/read_arm_pose.py) `--side right` — 현재 관절각 + hand_tcp 위치/RPY(Roll Pitch Yaw)/tilt 출력.
- [`experiments/save_grasp_reference.py`](../../experiments/save_grasp_reference.py) `--side right --note "..."` — 현재 자세 + 최근 검출 박스를 데이터셋(`<side>_grasp_reference_dataset.yaml`)에 **회전행렬까지** 누적 저장(읽기+저장 원자적 → 드리프트 최소).

> 팁: 자유 구동으로 잡은 직후 `save_grasp_reference.py` 를 한 번에 실행해야 드리프트 없이 캡처된다.

---

## 7. 검증된 사용 사례 (2026-06-07)

- 오른팔을 자유 구동으로 여러 박스 위치에 대한 grasp 자세를 시연·캡처 → `experiments/right_grasp_reference_dataset.yaml` 데이터셋 구축.
- 그 자세들로 INIT 등록 + 좌우 미러(규칙: **j4 만 동일, 나머지 부호반전** `s=(-1,-1,-1,+1,-1,-1,-1)`) → 양팔 INIT 복귀 검증.
