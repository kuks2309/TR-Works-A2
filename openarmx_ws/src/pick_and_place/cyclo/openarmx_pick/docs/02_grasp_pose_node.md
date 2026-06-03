# grasp_pose_node.py 코드 심층 분석

분석일: 2026-06-03
패키지: openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick/grasp_pose_node.py`(278줄) 한 파일을 메서드 단위로 정밀 분석한다. 대상은 모듈 수준 docstring 및 tool-frame(도구 좌표계) convention, 선택적 import guard, `__init__` 파라미터 선언 전체, 그리고 모든 메서드의 제어 흐름·입출력·예외 처리·로깅이다. 수학적 유도(회전행렬 → quaternion 변환 공식, PCA(Principal Component Analysis, 주성분 분석) 공분산 분해 등)는 03 문서에서 다루므로, 본 문서에서는 코드 구조와 구현에만 집중한다. 교차 확인에 사용된 파일은 `launch/openarmx_pick.launch.py`, `README.md`, `package.xml`이다.

---

## 1. 모듈 개요와 docstring

`grasp_pose_node.py:2-22`의 모듈 docstring은 이 노드를 "Stage B: box-top detection → top-down 6-DoF(Degree of Freedom, 6자유도) grasp pose → (optional) MoveL"로 정의한다.

설계 철학이 docstring 안에 명시적으로 서술돼 있다.

- **입력**: `box_plane` 비전 노드가 발행하는 박스 상면(box-top) inlier PointCloud2(포인트 클라우드)
- **좌표계 변환**: TF2(Transform Framework 2)를 통해 카메라 광학 프레임 → 로봇 기저 프레임(`openarmx_body_link0`)
- **grasp 합성 전략**: 학습 기반 grasp 네트워크 없이, 다음 세 요소로 grasp를 완전히 결정한다.
  - grasp point = 박스 상면 centroid(무게중심), 옵션으로 `grasp_depth`만큼 아래로 이동
  - approach(접근 방향) = 기저 프레임 -z 방향(수직 하강)
  - yaw(회전각) = 박스 상면의 XY 평면 PCA 주축(long axis)에서 유도한 opening(그리퍼 개방 방향)

### 1.1 Tool-frame convention

`grasp_pose_node.py:19-22`에 명시된 규칙:

> "the controlled link `openarmx_left_hand_tcp` is assumed to grasp along its local +z (approach) with the fingers opening along local +x."

즉, tool frame에서 +z = approach axis(접근 축), +x = opening axis(개방 축)이며, 이 두 축은 파라미터 `tool_approach_axis`, `tool_opening_axis`로 재정의 가능하다. 코드의 rotation 합성은 이 tool convention을 기저 프레임의 원하는 approach/opening 방향으로 매핑하는 방식으로 설계됐다.

---

## 2. 모듈 수준 선언과 import

### 2.1 표준 의존성

`grasp_pose_node.py:23-36`에서 임포트하는 패키지:

| 모듈 | 용도 |
|---|---|
| `json` | `/box_plane/info` 토픽 JSON 파싱 |
| `math` | `isfinite`, `sqrt` 등 스칼라 연산 |
| `struct` | PointCloud2 raw bytes에서 float32 언팩 |
| `numpy` | 행렬/벡터 연산 (PCA, 좌표 변환) |
| `rclpy` | ROS2(Robot Operating System 2) Python 클라이언트 |
| `geometry_msgs`, `std_msgs`, `visualization_msgs` | 메시지 타입 |
| `sensor_msgs.PointCloud2` | 입력 포인트 클라우드 |
| `tf2_ros` | TF2 버퍼·리스너 |

### 2.2 선택적 import guard: `_HAVE_MOVEL`

`grasp_pose_node.py:38-42`:

```python
try:
    from openarmx_scenario_player_msgs.msg import MoveL
    _HAVE_MOVEL = True
except Exception:  # MoveL only needed when auto_send=True
    _HAVE_MOVEL = False
```

`openarmx_scenario_player_msgs` 패키지가 없는 환경(예: `cyclo_ws`만 빌드된 상태)에서도 노드가 임포트 오류 없이 기동되도록 하는 방어 코드다. `except Exception`으로 `ImportError` 외의 예외도 흡수한다. 이 guard의 실제 동작은 `__init__`에서 `auto_send and _HAVE_MOVEL` 조건으로 연결된다.

---

## 3. 모듈 수준 헬퍼 함수

클래스 외부에 세 개의 순수 함수(pure function)가 정의돼 있다. 클래스 메서드가 아니라 모듈 함수로 분리된 이유는 단위 테스트 용이성과 재사용성이다.

### 3.1 `_read_xyz`

`grasp_pose_node.py:46-62`

**시그니처**: `(cloud: PointCloud2, stride: int = 1) -> np.ndarray`

**목적**: `PointCloud2` 메시지에서 유한한(finite) XYZ 좌표만 추출해 `(N, 3)` float64 배열로 반환한다.

**제어 흐름**:

1. `cloud.fields`를 순회해 필드 이름 → 바이트 오프셋 딕셔너리 `off`를 구성한다 (`grasp_pose_node.py:48`).
2. x, y, z 필드 중 하나라도 없으면 `np.empty((0, 3))`을 즉시 반환한다 (`grasp_pose_node.py:49-50`). 잘못된 메시지에 대한 조기 반환이다.
3. `cloud.point_step`(포인트 하나의 바이트 크기)과 `cloud.data`(raw bytes)를 추출한다.
4. `range(0, n, stride)`로 서브샘플링하며, 각 포인트에 대해 `struct.unpack_from("<f", data, base + offset)`으로 little-endian float32를 읽는다 (`grasp_pose_node.py:55-60`).
5. `math.isfinite(x) and math.isfinite(y) and math.isfinite(z)` 조건으로 NaN·Inf를 걸러낸다 (`grasp_pose_node.py:60`).
6. 유효한 포인트가 하나라도 있으면 `np.asarray(..., dtype=np.float64)`, 없으면 `np.empty((0, 3))`을 반환한다 (`grasp_pose_node.py:62`).

**입출력**:
- 입력: `PointCloud2` 메시지, 정수형 stride(기본 1, `__init__`에서 `cloud_stride` 파라미터로 전달)
- 출력: `(N, 3)` float64 배열 (카메라 좌표계, 단위: m)

**예외 처리**: 명시적 try-except 없이 조기 반환으로 처리한다. `struct.unpack_from`이 바이트 오프셋 오류를 낼 경우 호출자(`_on_cloud`)로 예외가 전파된다. 이 경우에 대한 방어 코드는 없다(추정: 정상적인 `box_plane` 노드가 항상 표준 PointCloud2를 발행한다고 가정).

### 3.2 `_quat_from_matrix`

`grasp_pose_node.py:65-80`

**시그니처**: `(m: np.ndarray) -> np.ndarray`

**목적**: 3×3 회전행렬을 quaternion(사원수) `[x, y, z, w]` 형식으로 변환한다.

**제어 흐름**: Shepperd's method(수치 안정성을 위한 분기 방식)의 4-case 구현이다.

1. `tr = m[0,0] + m[1,1] + m[2,2]` (trace) > 0: 표준 경로 (`grasp_pose_node.py:68-70`)
2. `m[0,0] > m[1,1] and m[0,0] > m[2,2]`: x 성분이 가장 큰 경우 (`grasp_pose_node.py:71-73`)
3. `m[1,1] > m[2,2]`: y 성분이 가장 큰 경우 (`grasp_pose_node.py:74-76`)
4. 나머지: z 성분이 가장 큰 경우 (`grasp_pose_node.py:77-79`)

각 case에서 스칼라 `s`로 나누는 방식은 0으로 나누는 것을 방지한다(diagonal 요소 중 최댓값을 분모로 사용). 반환 형식은 `[x, y, z, w]`이며, `geometry_msgs/Quaternion`의 필드 순서와 일치한다.

**입출력**:
- 입력: `(3, 3)` float64 회전행렬
- 출력: `(4,)` float64 배열 `[x, y, z, w]`

### 3.3 `_grasp_rotation`

`grasp_pose_node.py:83-97`

**시그니처**: `(approach, opening, tool_approach, tool_opening) -> np.ndarray`

**목적**: tool frame의 approach/opening 축을 기저 프레임(base frame)에서 원하는 approach/opening 방향으로 정렬하는 rotation matrix R_base_tool을 반환한다.

**제어 흐름**:

1. `approach`를 단위벡터로 정규화 (`grasp_pose_node.py:87`).
2. `opening`에서 approach 방향 성분을 제거(Gram-Schmidt)해 직교 성분만 남긴다 (`grasp_pose_node.py:88`).
3. `opening`의 노름이 1e-6 미만이면(approach와 거의 평행) 대체 벡터 `[1,0,0]`의 직교 성분을 사용한다 (`grasp_pose_node.py:89-90`). 수치적 degenerate case 방어 코드다.
4. 정규화 후 `B = [o, cross(a,o), a]` 열행렬 구성 → 원하는 방향의 frame basis (`grasp_pose_node.py:92`).
5. 동일한 방식으로 tool frame basis `T = [to, cross(ta,to), ta]` 구성 (`grasp_pose_node.py:93-96`).
6. `R = B @ T.T`를 반환한다 (`grasp_pose_node.py:97`).

**입출력**:
- 입력: 모두 `(3,)` float64 벡터. 단위벡터일 필요 없음(내부에서 정규화)
- 출력: `(3, 3)` float64 회전행렬

**예외 처리**: step 3의 degenerate case가 유일한 방어 분기다. tool_approach와 tool_opening이 평행한 경우에 대한 방어 코드는 없다(추정: 파라미터 유효성을 호출자가 보장한다고 가정).

---

## 4. `GraspPoseNode` 클래스

`grasp_pose_node.py:101-262`

### 4.1 `__init__`

`grasp_pose_node.py:102-156`

#### 파라미터 선언

`super().__init__("grasp_pose_node")` 호출 후 `declare_parameter`로 14개의 ROS2 파라미터를 선언한다.

| 파라미터 이름 | 기본값 | 단위 | 설명 |
|---|---|---|---|
| `cloud_topic` | `/box_plane/cloud` | — | 입력 PointCloud2 토픽 |
| `info_topic` | `/box_plane/info` | — | 박스 정보 JSON 토픽 |
| `base_frame` | `openarmx_body_link0` | — | 기저 프레임 ID |
| `movel_topic` | `/openarmx/left/movel` | — | MoveL(직선 이동 명령) 발행 토픽 |
| `grasp_pose_topic` | `/openarmx/grasp_pose` | — | grasp pose PoseStamped 발행 토픽 |
| `pregrasp_height` | 0.10 | m | 박스 상면 위 pre-grasp(사전 접근) 높이 |
| `grasp_depth` | 0.005 | m | 박스 상면 아래 grasp 포인트 오프셋 |
| `cloud_stride` | 4 | — | PointCloud2 서브샘플링 간격 |
| `auto_send` | `False` | — | pre-grasp MoveL 자동 발행 여부 |
| `move_time` | 4.0 | s | MoveL 동작 완료 목표 시간 |
| `send_min_interval` | 5.0 | s | MoveL 재발행 최소 쿨다운 |
| `send_min_delta` | 0.02 | m | MoveL 재발행을 트리거하는 목표 이동 거리 |
| `tool_approach_axis` | `[0,0,1]` | — | TCP(Tool Center Point, 도구 중심점) approach 축 (tool frame) |
| `tool_opening_axis` | `[1,0,0]` | — | TCP opening(그리퍼 개방) 축 (tool frame) |

`grasp_pose_node.py:123-135`에서 파라미터 값을 멤버 변수로 캐싱한다.

#### debounce 상태 초기화

`grasp_pose_node.py:132-133`:

```python
self._last_sent_xyz = None
self._last_sent_t = None
```

`_should_send` 메서드가 사용하는 이전 발행 위치와 시각이다. `None`은 "첫 번째 발행이 아직 이루어지지 않은 상태"를 의미한다.

#### TF2 버퍼 및 리스너

`grasp_pose_node.py:137-138`:

```python
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
```

`tf2_ros.TransformListener`는 `/tf`, `/tf_static` 토픽을 구독해 `tf_buffer`를 지속적으로 갱신한다. 별도의 실행자(executor)나 스핀 스레드는 필요 없이 `rclpy.spin`의 콜백 루프 안에서 동작한다.

`_box_height` 멤버는 `None`으로 초기화되며(`grasp_pose_node.py:139`), `_on_info` 콜백이 수신되기 전까지 로깅에서 `None`으로 표시된다.

#### QoS(Quality of Service, 서비스 품질) 설정

`grasp_pose_node.py:141-143`:

```python
latched = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
```

`TRANSIENT_LOCAL` durability는 ROS1의 "latched topic"에 해당한다. 새로운 구독자가 연결되면 마지막으로 발행된 메시지를 즉시 수신한다. `depth=1`은 큐에 최신 메시지 하나만 유지한다는 의미다. `pose_pub`과 `marker_pub` 모두 이 QoS를 사용한다(`grasp_pose_node.py:144-145`).

#### Publisher 구성

| Publisher | 토픽 | 메시지 타입 | QoS |
|---|---|---|---|
| `pose_pub` | `grasp_pose_topic` | `PoseStamped` | latched (depth=1, RELIABLE, TRANSIENT_LOCAL) |
| `marker_pub` | `/openarmx/grasp_markers` | `MarkerArray` | latched (depth=1, RELIABLE, TRANSIENT_LOCAL) |
| `movel_pub` | `movel_topic` | `MoveL` | depth=10 (기본 RELIABLE) |

`movel_pub`은 `auto_send=True and _HAVE_MOVEL`인 경우에만 생성된다(`grasp_pose_node.py:146-148`). `auto_send=True`이지만 `_HAVE_MOVEL=False`이면 경고 로그를 남기고 `movel_pub`은 `None`으로 유지된다(`grasp_pose_node.py:149-150`).

#### Subscriber 구성

`grasp_pose_node.py:152-153`:

```python
self.create_subscription(String, gp("info_topic").value, self._on_info, 10)
self.create_subscription(PointCloud2, gp("cloud_topic").value, self._on_cloud, 5)
```

- info 구독: depth=10, 콜백 `_on_info`
- cloud 구독: depth=5, 콜백 `_on_cloud`

cloud 구독의 depth가 info보다 작은 이유는 PointCloud2 메시지의 크기 때문이다(추정). 큐 깊이를 5로 제한해 오래된 클라우드가 쌓이는 것을 방지한다.

#### 초기화 완료 로그

`grasp_pose_node.py:154-156`에서 `info`-레벨 로그를 출력한다:

```
grasp_pose_node up: cloud='...' -> base='...', auto_send=...
```

---

### 4.2 `_on_info`

`grasp_pose_node.py:158-162`

**시그니처**: `(self, msg: String) -> None`

**목적**: `/box_plane/info` 토픽에서 JSON을 파싱해 `box_height_m` 값을 `self._box_height`에 저장한다.

**제어 흐름**:

```
msg.data (JSON str)
  └─ json.loads()
       └─ .get("box_height_m")  →  self._box_height
```

`try-except Exception: pass` 패턴으로 JSON 파싱 오류를 완전히 무시한다. 파싱에 실패해도 `self._box_height`는 이전 값을 유지한다(초기에는 `None`).

**입출력**:
- 입력: `std_msgs/String`, `data` 필드가 JSON 문자열
- 출력: 없음 (side-effect: `self._box_height` 갱신)

**참고**: `self._box_height`는 현재 `_on_cloud`의 로깅에서만 사용된다(`grasp_pose_node.py:203`). grasp 알고리즘 자체에는 사용되지 않는다. 박스 높이 정보를 활용한 grasp depth 자동 조정 기능은 구현되지 않은 상태다(추정: 미래 확장 포인트).

---

### 4.3 `_on_cloud` — 메인 알고리즘

`grasp_pose_node.py:164-203`

이 노드의 핵심 콜백이다. PointCloud2 메시지 수신마다 전체 grasp 합성 파이프라인을 실행한다.

#### 전체 흐름 개요

```
PointCloud2 수신
 ├─ 1. _read_xyz() → pts_cam (카메라 좌표계)
 ├─ 2. 포인트 수 검사 (< 30이면 조기 반환)
 ├─ 3. TF2 lookup: base_frame ← cloud.header.frame_id
 ├─ 4. _tf_to_Rt() → (R, t)
 ├─ 5. 좌표 변환: pts = (R @ pts_cam.T).T + t  (기저 프레임)
 ├─ 6. centroid 계산
 ├─ 7. XY PCA → long_axis → opening 벡터
 ├─ 8. approach = [0,0,-1]  (수직 하강)
 ├─ 9. _grasp_rotation() → R_bt → _quat_from_matrix() → quat
 ├─ 10. grasp_xyz = centroid - grasp_depth * z
 │       pre_xyz  = centroid + pregrasp_h * z
 ├─ 11. _publish_pose(grasp_xyz, quat)
 ├─ 12. _publish_marker(grasp_xyz, pre_xyz)
 └─ 13. [auto_send] _should_send(pre_xyz) → _send_movel(pre_xyz, quat)
```

#### 단계별 설명

**Step 1-2: 포인트 추출 및 최소 개수 검사**

`grasp_pose_node.py:165-169`:

```python
pts_cam = _read_xyz(cloud, self.stride)
if pts_cam.shape[0] < 30:
    self.get_logger().warn(f"box-top cloud too small ({pts_cam.shape[0]} pts).",
                           throttle_duration_sec=2.0)
    return
```

30개 미만이면 PCA가 의미 없으므로 조기 반환한다. `throttle_duration_sec=2.0`으로 경고 로그를 최대 0.5Hz로 제한한다.

**Step 3: TF2 transform lookup**

`grasp_pose_node.py:170-176`:

```python
tf = self.tf_buffer.lookup_transform(
    self.base_frame, cloud.header.frame_id, rclpy.time.Time())
```

`rclpy.time.Time()`은 "가장 최근에 사용 가능한 transform"을 의미한다(타임스탬프 0). TF2 조회 실패 시 `except Exception`으로 경고 로그 후 조기 반환한다. 이 경우에도 `throttle_duration_sec=2.0`이 적용된다(`grasp_pose_node.py:174-176`).

**Step 4-5: 좌표 변환**

`grasp_pose_node.py:177-178`:

```python
R, t = self._tf_to_Rt(tf)
pts = (R @ pts_cam.T).T + t
```

`pts_cam`이 `(N, 3)` 행렬이므로 `pts_cam.T`는 `(3, N)`, `R @ pts_cam.T`는 `(3, N)`, `.T`로 다시 `(N, 3)`이 된다. `t`는 `(3,)` 벡터로 브로드캐스팅된다.

**Step 6-7: Centroid 및 PCA**

`grasp_pose_node.py:180-187`:

```python
centroid = pts.mean(axis=0)
xy = pts[:, :2] - centroid[:2]
cov = xy.T @ xy / max(len(xy) - 1, 1)
evals, evecs = np.linalg.eigh(cov)
long_axis = evecs[:, int(np.argmax(evals))]
long_axis = np.array([long_axis[0], long_axis[1], 0.0])
opening = np.cross(np.array([0.0, 0.0, 1.0]), long_axis)
opening /= np.linalg.norm(opening)
```

XY 평면에서 2×2 공분산 행렬을 구성하고 `np.linalg.eigh`(대칭 행렬 고유값 분해)로 주성분 방향을 찾는다. `eigh`는 `eig`와 달리 대칭 행렬에 특화돼 고유벡터가 항상 실수이고 정규화된다. 최대 고유값에 대응하는 고유벡터가 박스의 long axis(긴 방향)다.

opening 방향은 `cross([0,0,1], long_axis)`로 구한다. 이는 long axis에 직교하면서 XY 평면 내에 있는 벡터다. 즉 박스의 short axis(짧은 방향)로, 그리퍼가 이 방향으로 벌어져 박스를 감싸게 된다.

`long_axis`의 z 성분을 0으로 강제 설정하는 것(`grasp_pose_node.py:185`)은 박스 상면이 기저 프레임 XY 평면과 평행하다는 가정에 기반한다.

**Step 8-9: Approach 방향 및 회전행렬**

`grasp_pose_node.py:189-191`:

```python
approach = np.array([0.0, 0.0, -1.0])
R_bt = _grasp_rotation(approach, opening, self.tool_a, self.tool_o)
quat = _quat_from_matrix(R_bt)
```

approach는 항상 기저 프레임 -z(수직 하강)로 고정돼 있다. 테이블 위 박스는 항상 수평면에 놓인다는 전제다.

**Step 10: Grasp 및 Pre-grasp 위치 계산**

`grasp_pose_node.py:193-194`:

```python
grasp_xyz = centroid.copy(); grasp_xyz[2] -= self.grasp_depth
pre_xyz = centroid.copy(); pre_xyz[2] += self.pregrasp_h
```

`grasp_xyz`는 centroid에서 `grasp_depth`(기본 0.005 m)만큼 z 감소. 손가락이 박스 상면 아래로 미세하게 진입하는 위치다.

`pre_xyz`는 centroid에서 `pregrasp_h`(기본 0.10 m)만큼 z 증가. 팔이 박스 위에 안전하게 정렬되는 위치다.

**Step 11-13: 발행**

- `_publish_pose(grasp_xyz, quat, stamp)`: grasp 위치를 PoseStamped로 발행
- `_publish_marker(grasp_xyz, pre_xyz, stamp)`: RViz 시각화 화살표 발행
- `movel_pub is not None and _should_send(pre_xyz)` 조건 충족 시 `_send_movel(pre_xyz, quat, stamp)` 호출

MoveL 목표가 `grasp_xyz`가 아닌 `pre_xyz`인 점에 주목한다. 즉 팔은 박스 상면 10 cm 위까지만 자동으로 이동하며, 실제 하강·그리퍼 닫기·리프트는 별도 FSM(Finite State Machine, 유한 상태 기계)에서 담당하게 설계돼 있다.

**로깅**

`grasp_pose_node.py:201-203`:

```python
self.get_logger().info(
    f"grasp @ ({grasp_xyz[0]:+.3f},{grasp_xyz[1]:+.3f},{grasp_xyz[2]:+.3f}) base, "
    f"{pts.shape[0]} pts, h={self._box_height}", throttle_duration_sec=1.0)
```

`throttle_duration_sec=1.0`으로 최대 1Hz 로깅. 알고리즘 성공 경로의 유일한 로그이며, grasp 좌표·포인트 수·박스 높이를 함께 출력한다. `pts.shape[0]`은 stride 적용 후 실제 변환된 포인트 수다.

---

### 4.4 `_tf_to_Rt`

`grasp_pose_node.py:205-215`

**시그니처**: `@staticmethod (tf) -> (np.ndarray, np.ndarray)`

**목적**: `tf2_ros`의 `TransformStamped` 객체에서 3×3 회전행렬 R과 평행이동 벡터 t를 추출한다.

**제어 흐름**: quaternion `(x, y, z, w)` → 3×3 회전행렬 직접 전개. `scipy.spatial.transform`이나 별도 tf2_geometry_msgs 의존성 없이 순수 numpy로 처리한다. 회전행렬의 수학적 유도(quaternion → matrix 전개 공식)는 03 문서에서 다룬다.

```python
tr = tf.transform.translation
return R, np.array([tr.x, tr.y, tr.z])
```

**입출력**:
- 입력: `geometry_msgs/TransformStamped` (tf2_ros lookup 결과)
- 출력: `(R: (3,3) float64, t: (3,) float64)`

`@staticmethod` 데코레이터로 인스턴스 상태에 의존하지 않음을 명시한다.

---

### 4.5 `_publish_pose`

`grasp_pose_node.py:217-224`

**시그니처**: `(self, xyz, quat, stamp) -> None`

**목적**: `PoseStamped` 메시지를 조립해 `pose_pub`으로 발행한다.

**제어 흐름**: 단순한 메시지 조립 메서드다. `frame_id = self.base_frame`, `stamp = cloud.header.stamp`(카메라 타임스탬프 그대로 전달). `map(float, xyz)`와 `map(float, quat)`으로 numpy scalar를 Python float으로 변환한다. numpy scalar를 그대로 전달하면 일부 ROS2 메시지 직렬화에서 타입 오류가 발생할 수 있으므로 명시적 변환이 필요하다.

**입출력**:
- 입력: xyz `(3,)`, quat `[x,y,z,w] (4,)`, stamp `rclpy.time.Time`
- 출력: 없음 (side-effect: `pose_pub` 발행)

---

### 4.6 `_should_send`

`grasp_pose_node.py:226-238`

**시그니처**: `(self, pre_xyz) -> bool`

**목적**: MoveL 재발행 debounce(디바운스, 중복 발행 방지) 로직. 매 카메라 프레임마다 MoveL을 발행하면 QP 솔버가 trajectory를 매 사이클 재시작해 팔이 목표 방향으로 "기어가는" 현상이 발생한다. 이를 방지하기 위해 두 조건 중 하나가 충족될 때만 발행을 허용한다.

**제어 흐름**:

```
now = 현재 시각 (nanoseconds * 1e-9 → float seconds)

if _last_sent_xyz is None:         # 최초 발행
    저장 + return True

moved = |pre_xyz - _last_sent_xyz|
elapsed = now - _last_sent_t

if moved > send_min_delta (0.02 m):  # 목표 위치가 2 cm 이상 이동
    저장 + return True
if elapsed > send_min_interval (5.0 s):  # 5초 쿨다운 경과
    저장 + return True

return False
```

두 조건 `moved > send_min_delta OR elapsed > send_min_interval`이 OR 관계임에 주목한다(`grasp_pose_node.py:235`). 목표가 고정됐더라도 5초마다 한 번은 재발행되며, 목표가 크게 움직이면 쿨다운과 무관하게 즉시 재발행된다.

`send_min_interval`의 기본값 5.0 s는 `move_time`(기본 4.0 s)보다 크게 설정돼 있다. 이는 첫 번째 MoveL 동작이 완료된 후에야 다음 MoveL이 발행되도록 의도한 설계다(`grasp_pose_node.py:115-119` 주석 참고).

**입출력**:
- 입력: `pre_xyz (3,)` ndarray
- 출력: `bool` (발행 허용 여부)
- side-effect: `_last_sent_xyz`, `_last_sent_t` 갱신

---

### 4.7 `_send_movel`

`grasp_pose_node.py:240-249`

**시그니처**: `(self, xyz, quat, stamp) -> None`

**목적**: `openarmx_scenario_player_msgs/MoveL` 메시지를 조립해 `movel_pub`으로 발행한다.

**제어 흐름**: `MoveL` 메시지는 `PoseStamped` 형식의 목표 포즈와 `Duration` 형식의 `time_from_start`를 포함한다. `frame_id`와 `stamp`는 `_publish_pose`와 동일한 방식으로 설정된다. `time_from_start`는 `move_time` float을 정수 초(sec)와 나노초(nanosec)로 분리해 설정한다(`grasp_pose_node.py:247-248`):

```python
m.time_from_start.sec = int(self.move_time)
m.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
```

**입출력**:
- 입력: xyz `(3,)`, quat `[x,y,z,w] (4,)`, stamp
- 출력: 없음 (side-effect: `movel_pub` 발행)

**참고**: 목표 위치가 `grasp_xyz`(박스 상면 아래)가 아닌 `pre_xyz`(박스 상면 10 cm 위)임을 상기한다. MoveL은 pre-grasp hover 위치로의 이동만 담당한다.

---

### 4.8 `_publish_marker`

`grasp_pose_node.py:251-262`

**시그니처**: `(self, grasp_xyz, pre_xyz, stamp) -> None`

**목적**: RViz(ROS Visualization, ROS 시각화 도구) 화살표 마커를 발행한다.

**제어 흐름**: `Marker.ARROW` 타입을 사용한다. RViz에서 ARROW 타입에 `points` 필드를 설정하면 두 점을 잇는 화살표가 그려진다.

```python
a.points = [Point(x=float(pre_xyz[0]), ...),   # 화살표 꼬리 (pre-grasp)
            Point(x=float(grasp_xyz[0]), ...)]  # 화살표 머리 (grasp point)
```

`grasp_pose_node.py:259-260`: 화살표는 pre-grasp → grasp 방향, 즉 위에서 아래로 접근하는 방향을 시각화한다.

**마커 속성**:
- `ns="grasp"`, `id=0`: 동일한 ns+id는 매 발행마다 덮어쓰인다(LIFO)
- `scale`: x=0.01(샤프트 직경), y=0.02(헤드 직경), z=0.0(헤드 길이 자동)
- `color`: RGBA = (0.1, 1.0, 0.2, 0.9) — 연두색 불투명 화살표

`MarkerArray`로 감싸서 발행하는 것은(`grasp_pose_node.py:261`) 향후 여러 마커(예: opening 방향 축, 포인트 클라우드 centroid sphere 등)를 추가하기 위한 확장성 있는 설계다(추정).

---

### 4.9 `main`

`grasp_pose_node.py:265-278`

**시그니처**: `() -> None`

**목적**: 노드 진입점. `rclpy.init()`, 노드 생성, `rclpy.spin()`, 정리 순서로 동작한다.

**제어 흐름**:

```python
rclpy.init()
node = GraspPoseNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

`KeyboardInterrupt`를 `except`로 잡아 Ctrl-C 시 정상 종료를 보장한다. `finally` 블록에서 `destroy_node()`와 `rclpy.shutdown()`을 호출해 ROS2 리소스를 정리한다. `setup.cfg`의 `console_scripts`에 `grasp_pose_node = openarmx_pick.grasp_pose_node:main`으로 등록돼 `ros2 run openarmx_pick grasp_pose_node`로 실행된다.

---

## 5. 토픽 인터페이스 요약

| 방향 | 토픽 | 타입 | QoS | 목적 |
|---|---|---|---|---|
| IN | `/box_plane/cloud` | `sensor_msgs/PointCloud2` | depth=5 | 박스 상면 inlier 포인트 클라우드 |
| IN | `/box_plane/info` | `std_msgs/String` | depth=10 | 박스 메타정보 JSON (`box_height_m`) |
| OUT | `/openarmx/grasp_pose` | `geometry_msgs/PoseStamped` | TRANSIENT_LOCAL, depth=1 | top-down grasp 포즈 |
| OUT | `/openarmx/grasp_markers` | `visualization_msgs/MarkerArray` | TRANSIENT_LOCAL, depth=1 | RViz 접근 화살표 |
| OUT | `/openarmx/left/movel` | `openarmx_scenario_player_msgs/MoveL` | depth=10 | pre-grasp MoveL 명령 (auto_send=True 시) |

---

## 6. 예외 처리 패턴 정리

노드 전반에서 사용되는 예외 처리 전략은 두 가지다.

1. **조기 반환 + 경고 로그**: `_on_cloud` 내 포인트 수 부족 및 TF2 실패 시. `throttle_duration_sec`으로 로그 폭주를 방지한다.
2. **`except Exception: pass`**: `_on_info`의 JSON 파싱 실패 시. 비필수 정보이므로 무시한다.

`_read_xyz`에서 `struct.unpack_from` 실패, `_grasp_rotation`에서 tool_approach/tool_opening 평행, `np.linalg.eigh` 실패(포인트가 모두 동일한 경우) 등에 대한 방어 코드는 없다. 이는 `box_plane` 노드가 항상 유효한 형식의 PointCloud2를 발행한다는 가정 아래 설계된 것으로 보인다(추정).

---

## 7. 설계상 주목할 점

### 7.1 `scipy` 미사용

회전행렬·quaternion 변환(`_quat_from_matrix`, `_tf_to_Rt`), 포인트 파싱(`struct.unpack_from`)을 `scipy`나 `transforms3d` 없이 순수 `math`/`numpy`/`struct`로 구현했다. 외부 의존성을 최소화해 빌드 환경을 단순하게 유지하는 의도다.

### 7.2 TRANSIENT_LOCAL latched QoS

`pose_pub`과 `marker_pub`이 latched로 설정돼 있어, RViz를 나중에 띄워도 마지막 grasp 포즈와 마커를 즉시 볼 수 있다.

### 7.3 debounce의 `send_min_interval >= move_time` 설계 의도

`send_min_interval`(기본 5.0 s)이 `move_time`(기본 4.0 s)보다 크게 설정된 것은 의도적이다(`grasp_pose_node.py:117-118` 주석): "≥ move_time lets a motion finish first". MoveL 동작이 완료되기 전에 새 MoveL이 발행되면 QP 솔버가 현재 trajectory를 버리고 새 trajectory를 시작하므로, 팔이 목표에 도달하지 못하고 진동하는 현상이 생긴다.

### 7.4 grasp 포즈와 MoveL 목표의 분리

`_publish_pose`는 `grasp_xyz`(박스 상면 약간 아래)를, `_send_movel`은 `pre_xyz`(박스 상면 위 10 cm)를 목표로 한다. grasp 포즈는 최종 descend 목표로, MoveL은 pre-grasp hover 목표로 역할이 분리돼 있다. 상위 FSM이 pre-grasp 완료 후 grasp 포즈를 사용해 하강 단계를 실행하는 구조를 상정한 설계다.

---

## 8. 콜백 간 데이터 공유

```
_on_info ─────────────────────────── self._box_height (로깅 전용)
_on_cloud ─┬─ _read_xyz
           ├─ TF2 lookup
           ├─ PCA → centroid, quat
           ├─ _publish_pose
           ├─ _publish_marker
           └─ _should_send → _send_movel
               └─ self._last_sent_xyz
                  self._last_sent_t
```

두 콜백은 `rclpy.spin`의 단일 스레드 실행자 내에서 순차적으로 실행되므로 별도의 mutex 없이 `self._box_height`, `self._last_sent_xyz`, `self._last_sent_t`를 공유해도 race condition이 발생하지 않는다. ROS2 기본 실행자는 단일 스레드다(추정: `MultiThreadedExecutor`를 명시적으로 사용하지 않는 한).
