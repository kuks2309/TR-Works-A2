# 좌표 프레임 & TF 체계 분석

분석일: 2026-06-03  
패키지: openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지의 TF(Transform Framework, 변환 프레임워크) 체계를 다룬다. 구체적으로는 파이프라인에 등장하는 모든 좌표 프레임의 역할과 상호 관계, `_on_cloud` 콜백 내 `tf_buffer.lookup_transform` 호출의 의미, `_tf_to_Rt`의 quaternion(사원수) → rotation matrix(회전행렬) 변환 수학, `_quat_from_matrix`의 역방향 변환, world → `openarmx_body_link0` identity(항등) 조인트가 perception(인식)↔control(제어) 경계를 어떻게 단순화하는지, 캘리브레이션 의존성이 노드 정확도에 미치는 영향, 그리고 tool-frame convention(도구 프레임 규약)을 다룬다.

**제외 범위**: PCA(Principal Component Analysis, 주성분 분석)를 이용한 yaw 결정 수학, Gram-Schmidt 직교화와 `B @ Tᵀ` 유도, `_quat_from_matrix`의 4-case Shepperd 분기 이론 — 이들은 [03_grasp_synthesis_theory.md](03_grasp_synthesis_theory.md)에서 상세히 다루므로 본 문서에서는 중복 없이 TF 체계의 맥락 안에서만 참조한다.

---

## 1. 파이프라인에 등장하는 좌표 프레임 목록

아래 프레임들이 `grasp_pose_node.py` 실행 경로에서 명시적으로 사용된다.

| 프레임 이름 | 종류 | 역할 |
|---|---|---|
| `world` | 정적 | URDF 최상위 앵커. solver URDF에서 `openarmx_body_link0`의 부모 |
| `openarmx_body_link0` | 정적 | **solver root이자 grasp 출력 frame**. `base_frame` 파라미터 기본값 |
| `camera_color_optical_frame` | 동적(카메라 TF) | `/box_plane/cloud`의 `header.frame_id`. 카메라 센서 광학 좌표계 |
| `d435_center_link` | 정적(캘리브 결과) | 로봇 URDF에 박혀있는 카메라 링크. 외부 파라미터 캘리브 결과가 이 링크에 연결됨 |
| `openarmx_left_hand_tcp` | kinematic chain | TCP(Tool Center Point, 도구 중심점). tool-frame convention의 기준 링크. URDF에 존재하나 `grasp_pose_node`가 직접 lookup하지는 않음 |

`grasp_pose_node.py:106`에서 `base_frame`의 기본값이 `"openarmx_body_link0"`으로 선언되며, `launch/openarmx_pick.launch.py:49`에서도 명시적으로 `"openarmx_body_link0"`으로 고정된다.

`cloud.header.frame_id`는 `/box_plane/cloud` 메시지가 발행될 때 발행자(fit_box_plane 노드)가 채우는 값으로, 실제 D435 운용 시에는 `camera_color_optical_frame`이다. `README.md`의 Topics 표에 이 값이 명시되어 있으며, `scripts/verify_grasp.py:35`에서 합성 검증 시에는 TF lookup이 identity가 되도록 의도적으로 `BASE = "openarmx_body_link0"`을 `frame_id`로 지정한다.

---

## 2. TF 트리 구조

파이프라인 전체에서 관련 TF 트리는 다음과 같다.

```
world
└── openarmx_body_link0   (joint: openarmx_body_world_joint, fixed, origin xyz="0 0 0" rpy="0 0 0")
    ├── openarmx_left_link1
    │   └── ... (7-DOF kinematic chain)
    │       └── openarmx_left_hand_tcp   (TCP, tool-frame convention 기준)
    ├── openarmx_right_link1  (solver URDF에서 fixed로 동결)
    │   └── ...
    └── d435_center_link      (캘리브 결과 정적 TF로 부착)
        └── d435_center_color_optical_frame
            └── camera_color_optical_frame   (RealSense2 드라이버 내부 TF)
```

**중요 지점 1 — world → openarmx_body_link0 identity**

`urdf/openarmx_left_solver.urdf:7-10`에서 해당 joint가 다음과 같이 선언된다.

```xml
<joint name="openarmx_body_world_joint" type="fixed">
  <parent link="world" />
  <child link="openarmx_body_link0" />
  <origin rpy="0 0 0" xyz="0 0 0" />
</joint>
```

`xyz="0 0 0" rpy="0 0 0"`은 완전한 identity transform이다. world 프레임과 `openarmx_body_link0` 프레임은 수치적으로 동일하다. 이 사실이 perception↔control TF 단순화의 핵심이며, §3에서 상세히 설명한다.

**중요 지점 2 — d435_center_link 정적 TF**

캘리브레이션으로 얻은 `openarmx_body_link0 → d435_center_link`의 정적 변환값은 다음과 같다(2026-06-01 측정, 60° 마운트 기준).

- translation: x = 0.065430 m, y = 0.000987 m, z = 0.641921 m  
- rotation RPY: roll = +0.6846°, pitch = +59.7350°, yaw = +0.4038°

이 값은 로봇 URDF(`openarmx_description/urdf/robot/v10.urdf.xacro`)의 bimanual 블록 안에 fixed joint로 베이크되어, `robot_state_publisher`가 TF를 자동 발행한다. `grasp_pose_node`가 필요로 하는 `camera_color_optical_frame → openarmx_body_link0` 변환은 이 정적 체인을 통해 TF 트리에 항상 존재한다.

---

## 3. world → body_link0 identity가 perception↔control TF를 단순화하는 방법

`grasp_pose_node`의 최종 출력인 `PoseStamped`와 `MoveL`의 `frame_id`는 모두 `self.base_frame = "openarmx_body_link0"`이다(`grasp_pose_node.py:219, 242`).

QP+CBF solver(`omx_movel_controller_node`)의 base_frame 역시 `openarmx_body_link0`이다(`launch/openarmx_movel.launch.py`에서 solver에 전달되는 URDF의 root). solver는 이 프레임에서 Pinocchio FK(Forward Kinematics, 순기구학)/Jacobian을 계산하고, 동일 프레임에서 표현된 goal pose와 비교해 QP를 구성한다.

결과적으로, grasp 합성 노드와 QP solver 사이의 경계에서 **추가적인 TF 변환이 전혀 필요 없다.** 발행된 grasp pose를 solver가 그대로 사용할 수 있다. `README.md`는 이를 명시한다.

> "The grasp pose is expressed in `openarmx_body_link0`, which is also the solver URDF root (the `world → body_link0` joint is identity), so no extra TF is needed between perception and control."

만약 world → body_link0 joint가 identity가 아니었다면(예: 로봇이 맵 상의 특정 위치에 존재하는 mobile manipulation 시나리오), grasp 출력 frame과 solver root frame이 달라지므로 perception 노드가 goal pose를 solver root frame으로 변환하는 추가 단계가 필요했을 것이다. 현재 설계는 고정 플랫폼을 가정하고 이 변환 단계를 제거한 것이다.

---

## 4. TF lookup: camera_color_optical_frame → openarmx_body_link0

`_on_cloud` 콜백은 포인트 클라우드 메시지를 수신할 때마다 TF를 조회한다(`grasp_pose_node.py:171-176`).

```python
tf = self.tf_buffer.lookup_transform(
    self.base_frame, cloud.header.frame_id, rclpy.time.Time())
```

세 인자의 의미는 다음과 같다.

- **첫 번째 인자 `self.base_frame`**: target frame. 변환 결과가 이 프레임 기준으로 표현된다. 즉 "cloud.header.frame_id에서 본 점을 `openarmx_body_link0`에서 어떻게 표현하느냐"를 묻는다.
- **두 번째 인자 `cloud.header.frame_id`**: source frame. 실제로는 `/box_plane/cloud` 발행자가 채우는 값이며, 라이브 운용 시 `camera_color_optical_frame`.
- **세 번째 인자 `rclpy.time.Time()`**: 시간 = 0. TF 버퍼에서 가장 최근의 사용 가능한 변환을 반환한다. 정적 TF는 시간에 무관하므로 타임스탬프 불일치 문제가 발생하지 않는다.

lookup 실패 시 `except Exception`으로 경고 로그를 출력하고 조기 반환하며, 다음 프레임에서 재시도한다(`grasp_pose_node.py:173-176`).

---

## 5. `_tf_to_Rt`: quaternion → rotation matrix + translation

lookup 결과인 `TransformStamped`에서 수치 변환을 추출하는 정적 메서드다(`grasp_pose_node.py:205-215`).

```python
@staticmethod
def _tf_to_Rt(tf):
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),    2*(x*z + y*w)  ],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z), 2*(y*z - x*w)  ],
        [2*(x*z - y*w),      2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])
    tr = tf.transform.translation
    return R, np.array([tr.x, tr.y, tr.z])
```

### 5.1 수학적 전개

unit quaternion `q = (x, y, z, w)`에서 rotation matrix로의 표준 변환이다. ROS2의 `geometry_msgs/Quaternion`은 Hamilton 규약을 따르므로, 행렬의 각 원소는 다음 관계식으로 유도된다.

임의의 3D 벡터 **v**에 대해 quaternion 회전은 `q ⊗ (0, **v**) ⊗ q*`로 정의된다. 이를 행렬 형태로 전개하면 위 9개 원소가 얻어진다. 핵심 부호 규약:

- `R[0][1] = 2(xy - zw)`, `R[1][0] = 2(xy + zw)`: 반대칭 항의 부호가 서로 반대이며, `±zw`는 Hamilton 규약에서 cross term의 방향을 결정한다.
- 대각 원소 `1 - 2(y² + z²)` 등은 `2(x² + y² + z² + w²) = 2`라는 unit quaternion 조건에서 나온다.

이 행렬 R은 **source frame의 벡터를 target frame으로 표현하는 rotation**이다. 즉 `R` = R_base←cam, `t` = t_base←cam(base 프레임 원점으로부터 cam 원점까지의 벡터, base 좌표계 표현).

### 5.2 포인트 클라우드 변환 적용

`_on_cloud` 내에서 실제 좌표 변환은 다음과 같이 이루어진다(`grasp_pose_node.py:177-178`).

```python
R, t = self._tf_to_Rt(tf)
pts = (R @ pts_cam.T).T + t   # Nx3 in base frame
```

`pts_cam`이 shape `(N, 3)`이므로 `pts_cam.T`는 `(3, N)`. `R @ pts_cam.T`는 `(3, N)`, 다시 `.T`로 `(N, 3)`을 만든다. `t`는 shape `(3,)`으로 numpy broadcasting에 의해 각 행에 더해진다. 이 연산은 affine transformation `pts_base = R · pts_cam + t`의 행렬 버전이다.

### 5.3 외부 의존성 없는 순수 numpy 구현

`scipy.spatial.transform.Rotation`이나 `tf2_geometry_msgs`를 사용하지 않고 직접 전개한 이유는, 추가 패키지 의존성 없이 `openarmx_ws` 단독 빌드가 가능하도록 하기 위함이다. 계산 복잡도는 O(1)이므로 성능 차이는 무시 가능하다.

---

## 6. `_quat_from_matrix`: rotation matrix → quaternion

`_grasp_rotation`에서 구한 `R_base_tool`(3×3 행렬)을 ROS2 메시지의 `geometry_msgs/Quaternion` 형식으로 변환하기 위해 사용한다(`grasp_pose_node.py:65-80`). 반환 형식은 `[x, y, z, w]`로 `geometry_msgs/Quaternion`의 필드 순서와 일치한다.

변환 수식과 Shepperd 4-case 분기의 이론적 유도는 [03_grasp_synthesis_theory.md §6](03_grasp_synthesis_theory.md)에서 상세히 다룬다. 본 문서에서는 TF 체계 맥락에서 이 함수가 수행하는 역할만 명시한다.

**부호 일관성**: `_tf_to_Rt`가 ROS2 TF 메시지의 quaternion을 읽을 때와 동일한 Hamilton 규약을 따른다. `_quat_from_matrix`가 생성하는 quaternion 역시 Hamilton 규약이다. 따라서 `_quat_from_matrix` 출력을 `PoseStamped.pose.orientation`에 넣으면, 해당 메시지를 받은 solver가 `_tf_to_Rt`와 같은 역변환을 적용해 R을 복원했을 때 원본 `R_base_tool`이 정확히 재현된다.

---

## 7. 출력 프레임 일관성: 모든 출력이 openarmx_body_link0 기준

`grasp_pose_node`가 발행하는 모든 출력 메시지의 `header.frame_id`는 `self.base_frame`, 즉 `"openarmx_body_link0"`로 통일된다.

| 출력 토픽 | frame_id 설정 위치 |
|---|---|
| `/openarmx/grasp_pose` | `grasp_pose_node.py:219` |
| `/openarmx/grasp_markers` | `grasp_pose_node.py:253` |
| `/openarmx/left/movel` (내부 PoseStamped) | `grasp_pose_node.py:242` |

RViz Marker의 `header.frame_id`도 동일하게 `self.base_frame`으로 설정되므로, RViz에서 fixed frame을 `openarmx_body_link0` 또는 `world`(identity이므로 동일)로 설정하면 화살표 마커가 정확한 위치에 표시된다.

---

## 8. tool-frame convention: openarmx_left_hand_tcp 기준

TCP의 로컬 좌표 규약은 노드 docstring에 명시되어 있다(`grasp_pose_node.py:19-21`).

> "the controlled link `openarmx_left_hand_tcp` is assumed to grasp along its local +z (approach) with the fingers opening along local +x."

이를 파라미터로 표현하면 다음과 같다(`grasp_pose_node.py:120-121`).

```python
self.declare_parameter("tool_approach_axis", [0.0, 0.0, 1.0])
self.declare_parameter("tool_opening_axis",  [1.0, 0.0, 0.0])
```

즉 TCP 로컬 프레임에서:

- **+z 축**: approach axis(접근 축) — 그리퍼가 물체에 다가가는 방향. grasp 합성에서 이 축이 base frame의 `-z` 방향(수직 하강)으로 정렬되어야 한다.
- **+x 축**: opening axis(개방 축) — 두 손가락이 벌어지는 방향. grasp 합성에서 이 축이 박스의 short axis(짧은 변 방향)로 정렬되어야 한다.

`_grasp_rotation` 함수는 이 tool-frame 규약을 기반으로 **base frame에서 desired(원하는) approach/opening 방향을 TCP의 +z/+x에 정확히 매핑하는 회전행렬 `R_base_tool`**을 구성한다.

### 8.1 tool-frame 기준의 desired 방향 매핑 정리

| tool 로컬 축 | 파라미터 | base frame에서의 desired 방향 | 결정 근거 |
|---|---|---|---|
| +z (approach) | `tool_approach_axis` | `(0, 0, -1)` (수직 하강) | 박스 윗면 법선 = base `+z` → approach = `-z` |
| +x (opening) | `tool_opening_axis` | PCA 짧은 변 방향 | 그리퍼가 짧은 변을 감싸야 함 |
| +y (binormal) | 자동 계산 | `approach × opening` | right-handed 직교계 완성 |

`tool_approach_axis`와 `tool_opening_axis`는 ROS2 파라미터이므로, 다른 그리퍼 모델 또는 다른 손 링크를 사용하는 경우 launch 인수로 재정의 가능하다. 예를 들어 오른팔용 solver를 사용할 때 손가락 방향이 반전되어 있다면 `tool_opening_axis:=[−1,0,0]`으로 조정할 수 있다(추정 — 오른팔 검증은 완료되지 않았으므로 실제 TCP 방향은 URDF FK로 별도 확인 필요).

### 8.2 `_grasp_rotation`에서 tool-frame이 사용되는 방식

`grasp_pose_node.py:83-97`의 `_grasp_rotation(approach, opening, tool_a, tool_o)`:

1. desired frame의 orthonormal basis `B = [o, a×o, a]`를 구성한다. 열 순서: [opening 방향, binormal, approach 방향].
2. tool frame의 동일한 구조 basis `T = [to, ta×to, ta]`를 구성한다.
3. `R_base_tool = B @ T.T`를 반환한다.

이 행렬은 **TCP 로컬 좌표의 벡터를 base 좌표로 표현하는 회전**이다. 구체적으로 `R_base_tool @ [0,0,1]ᵀ = approach = [0,0,-1]ᵀ`이고, `R_base_tool @ [1,0,0]ᵀ = opening`이다. 수학적 유도는 [03_grasp_synthesis_theory.md §5.2](03_grasp_synthesis_theory.md)를 참조한다.

---

## 9. 캘리브레이션 의존성과 grasp 정확도에 미치는 영향

### 9.1 TF 체인과 캘리브레이션 의존성

라이브 운용에서 `lookup_transform(openarmx_body_link0 ← camera_color_optical_frame)` 결과의 정확도는 전적으로 캘리브레이션으로 얻은 `body_link0 → d435_center_link` 정적 TF에 의존한다. TF 체인은 다음과 같다.

```
camera_color_optical_frame
  ← d435_center_color_optical_frame   (RealSense2 드라이버 내부 정적 TF, 고정)
  ← d435_center_link                   (캘리브 외부 파라미터 정적 TF  ← 이 링크의 정확도가 관건)
  ← openarmx_body_link0
```

즉 grasp 위치의 base frame 오차는 캘리브레이션 오차와 직결된다.

### 9.2 캘리브레이션 오차의 grasp 위치 오차로의 전파

캘리브레이션으로 얻은 외부 파라미터를 `T̂_base←cam`이라 하고, 실제(이상적) 변환을 `T_base←cam`이라 할 때, 오차 `δT = T̂ · T⁻¹`가 포인트 클라우드에 다음과 같이 전파된다(추정 — 오차 선형화 기반).

- **회전 오차 δR**: 카메라에서 거리 r에 있는 점이 `r · sin(δθ) ≈ r · δθ`만큼 이동한다. 카메라로부터 박스 중심까지의 거리가 약 0.5~0.8 m 수준이므로, 회전 오차 1°(≈ 0.017 rad)는 grasp 위치에 약 8~14 mm의 오차를 유발한다.
- **평행이동 오차 δt**: 모든 점에 동일하게 더해지므로, centroid 오차와 직접 대응된다. 1 mm 평행이동 오차 → 1 mm grasp 위치 오차.

### 9.3 실측 캘리브레이션 정밀도

메모리 기록(d435-camera-extrinsic)에 따르면 30 프레임 평균 기준 위치 표준편차가 0.1 mm 미만이다. Stage B 검증 결과(README.md:126-127)에서 합성 클라우드를 사용했을 때(TF = identity) position 오차가 ≈ 2 mm이므로, 이 2 mm는 알고리즘 자체(sampling, PCA 수치 오차)에 기인하며 캘리브레이션 오차는 별도로 더해진다.

라이브 카메라 검증(README.md:129)에서 D435 + 골판지 박스(top at body z ≈ 0.204 m)에 대해 안정적인 top-down grasp pose가 base frame에서 출력되었다고 보고되어 있으나, grasp 위치의 절대 오차에 대한 수치는 기록되지 않았다.

### 9.4 캘리브레이션 유효성 조건

캘리브레이션은 다음 조건이 유지될 때만 유효하다.

1. 카메라 마운트의 물리적 위치가 변하지 않아야 한다. 마운트 틸트가 바뀌면 pitch뿐 아니라 translation도 이동하므로 재캘리브레이션이 필수다(d435-camera-extrinsic 메모리 교훈 항목).
2. 로봇 베이스(`openarmx_body_link0`)가 작업 공간 내 고정 위치를 유지해야 한다. 베이스가 이동하는 경우 world → body_link0 identity 가정이 깨져 perception과 control의 좌표계가 분리된다(현재 설계에서 이 시나리오는 고려하지 않는다).

---

## 10. 합성 검증에서의 TF 단순화

`scripts/verify_grasp.py`는 카메라·TF 브로드캐스터 없이 grasp 합성만 독립적으로 검증하기 위해 `cloud.header.frame_id = "openarmx_body_link0"`(BASE)로 설정한다(`verify_grasp.py:24, 35`).

```python
BASE = "openarmx_body_link0"
...
msg.header = Header(frame_id=BASE)
```

이 경우 `lookup_transform(openarmx_body_link0 ← openarmx_body_link0)`은 identity transform을 반환하므로, `_tf_to_Rt`의 결과는 `R = I₃`, `t = (0, 0, 0)`. 포인트 클라우드는 변환 없이 그대로 base frame의 좌표로 처리된다.

이 방법으로 TF 체인의 정확도와 grasp 합성 알고리즘의 정확도를 분리 검증한다는 점이 설계의 장점이다.

---

## 11. 프레임 선택의 전체 요약

```
D435 depth 카메라
  │ PointCloud2 (frame_id = camera_color_optical_frame)
  ▼
[lookup_transform: openarmx_body_link0 ← camera_color_optical_frame]
  │  캘리브레이션 정적 TF 사용 (body_link0 → d435_center_link, pitch ≈ 60°)
  │  _tf_to_Rt: quaternion→R, translation→t
  ▼
[pts_base = R · pts_cam + t]   — Nx3, base frame
  │
  ▼ centroid, PCA yaw, approach=(0,0,-1)
  │  _grasp_rotation: tool +z → (0,0,-1),  tool +x → short axis
  │  _quat_from_matrix: R_base_tool → quaternion [x,y,z,w]
  ▼
PoseStamped / MoveL  (frame_id = openarmx_body_link0)
  │
  │  [frame 변환 없음 — world→body_link0 identity이므로]
  ▼
QP+CBF solver (base = openarmx_body_link0)
  → joint trajectory → arm
```

---

## 부록. 참조 코드 위치 (패키지 루트 기준)

| 주제 | 위치 |
|---|---|
| `base_frame` 파라미터 선언 | `openarmx_pick/grasp_pose_node.py:106` |
| `tf_buffer` / `tf_listener` 초기화 | `openarmx_pick/grasp_pose_node.py:137-138` |
| `lookup_transform` 호출 | `openarmx_pick/grasp_pose_node.py:171-172` |
| `_tf_to_Rt` (quaternion→R+t) | `openarmx_pick/grasp_pose_node.py:205-215` |
| 포인트 클라우드 기저 변환 `pts = (R @ pts_cam.T).T + t` | `openarmx_pick/grasp_pose_node.py:177-178` |
| `_quat_from_matrix` (R→quaternion) | `openarmx_pick/grasp_pose_node.py:65-80` |
| 출력 `PoseStamped` frame_id 설정 | `openarmx_pick/grasp_pose_node.py:219` |
| 출력 `MoveL` frame_id 설정 | `openarmx_pick/grasp_pose_node.py:242` |
| Marker frame_id 설정 | `openarmx_pick/grasp_pose_node.py:253` |
| tool_approach_axis / tool_opening_axis 파라미터 | `openarmx_pick/grasp_pose_node.py:120-121` |
| tool-frame convention docstring | `openarmx_pick/grasp_pose_node.py:19-21` |
| `_grasp_rotation` (tool frame → base frame R) | `openarmx_pick/grasp_pose_node.py:83-97` |
| world→body_link0 joint (identity) | `urdf/openarmx_left_solver.urdf:7-10` |
| launch에서 base_frame 고정 | `launch/openarmx_pick.launch.py:49` |
| 합성 검증에서 frame_id=BASE (TF identity) | `scripts/verify_grasp.py:24, 35` |
| 캘리브레이션 외부 파라미터 메모 | memory: d435-camera-extrinsic |
