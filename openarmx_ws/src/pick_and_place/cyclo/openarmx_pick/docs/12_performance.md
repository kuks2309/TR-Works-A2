# openarmx_pick 성능 분석

**분석일:** 2026-06-03
**패키지:** openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지의 런타임 성능 특성을 분석한다. 구체적으로 다음 네 가지 주제를 다룬다.

1. `_read_xyz`의 순수 Python 루프 비용과 `cloud_stride` 서브샘플링이 이를 완화하는 방식
2. PCA(Principal Component Analysis, 주성분 분석) 공분산 고유값 분해의 연산 비용
3. latched QoS(Quality of Service, 서비스 품질) — `TRANSIENT_LOCAL depth=1`의 의미와 성능 영향
4. MoveL debounce 메커니즘(`send_min_delta` / `send_min_interval`)이 solver trajectory 재시작을 방지하는 방식

알고리즘의 수학적 유도는 `03_grasp_synthesis_theory.md`에서 다루므로 본 문서에서는 다루지 않는다. 코드 구조의 메서드 수준 설명은 `02_grasp_pose_node.md`에서 다룬다.

분석 대상 파일: `openarmx_pick/grasp_pose_node.py` (278줄 전체). 교차 확인에 사용한 파일: `launch/openarmx_pick.launch.py`.

---

## 1. `_read_xyz` — 순수 Python struct 언팩 루프

### 1.1 구현

`openarmx_pick/grasp_pose_node.py:46-62`:

```python
def _read_xyz(cloud: PointCloud2, stride: int = 1) -> np.ndarray:
    off = {f.name: f.offset for f in cloud.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return np.empty((0, 3))
    ox, oy, oz = off["x"], off["y"], off["z"]
    step, data = cloud.point_step, cloud.data
    n = cloud.width * cloud.height
    out = []
    for i in range(0, n, stride):
        base = i * step
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append((x, y, z))
    return np.asarray(out, dtype=np.float64) if out else np.empty((0, 3))
```

### 1.2 Python 루프의 비용

`_read_xyz`는 `stride=1`(서브샘플 없음) 기준으로 N개 포인트에 대해 O(N) Python 루프를 수행한다. 루프 바디에서 반복되는 연산은 다음과 같다.

- `struct.unpack_from("<f", data, base + ox)[0]` 3회 — C 확장이지만 호출마다 Python 객체 경계를 넘는 비용이 발생한다.
- `math.isfinite(x) and math.isfinite(y) and math.isfinite(z)` — Python 스칼라 3회 검사.
- `out.append((x, y, z))` — Python 리스트에 튜플 추가; 리스트가 동적 재할당될 때 추가 비용이 발생한다.

CPython의 인터프리터 오버헤드(바이트코드 디스패치, 레퍼런스 카운팅 등)로 인해, 루프 1회 반복당 발생하는 실제 비용은 순수 C 루프 대비 수십 배 높다. D435 카메라의 VGA(640×480) 깊이 이미지를 organized PointCloud2로 출력하면 N ≈ 307,200이다. 박스 상면 inlier만 추출된 `/box_plane/cloud`는 이보다 훨씬 작지만, 구체적인 인라이어 수는 박스 크기와 거리에 따라 달라지므로 정확한 범위는 운영 환경에 의존한다(추정).

루프 종료 후 `np.asarray(out, dtype=np.float64)` 호출 시 Python 리스트 → numpy 배열 변환이 한 번 더 일어난다. 이 변환 자체는 C 수준에서 수행되지만, 리스트의 크기만큼 데이터를 복사하는 비용이 있다.

### 1.3 `cloud_stride=4` 서브샘플링의 완화 효과

`openarmx_pick/grasp_pose_node.py:111`:

```python
self.declare_parameter("cloud_stride", 4)
```

`launch/openarmx_pick.launch.py:54`:

```python
"cloud_stride": 4,
```

`stride=4`는 `range(0, n, 4)` 루프를 의미한다. 즉 루프 반복 횟수가 N에서 N/4로 줄어든다. Python 루프가 병목인 경우 실행 시간은 stride에 선형 반비례한다 — stride=4이면 이론적으로 stride=1 대비 약 4배 빠르다.

`cloud_stride` 파라미터는 `_on_cloud` 콜백의 첫 번째 단계에서 `_read_xyz`에 전달된다 (`openarmx_pick/grasp_pose_node.py:165`):

```python
pts_cam = _read_xyz(cloud, self.stride)
```

서브샘플링의 trade-off는 다음과 같다.

| 항목 | stride=1 | stride=4 |
|------|----------|----------|
| 루프 반복 횟수 | N | N/4 |
| PCA 입력 포인트 수 | N | N/4 |
| PCA 주축 추정 분산 | 낮음 | 다소 높음 |
| 최소 포인트 검사 통과 기준 | 30개 (원본) | 30개 (stride 후) |

박스 상면 inlier 클라우드는 공간적으로 밀집·평탄하므로, stride=4의 균등 서브샘플링 후에도 PCA 주축 추정 품질이 크게 저하되지 않는다(추정). 단, 매우 작은 박스이거나 카메라와의 거리가 멀어 inlier 수가 적은 경우에는 stride를 낮추는 것이 안전하다.

---

## 2. PCA 공분산 고유값 분해 비용

### 2.1 구현

`openarmx_pick/grasp_pose_node.py:180-187`:

```python
centroid = pts.mean(axis=0)
xy = pts[:, :2] - centroid[:2]
cov = xy.T @ xy / max(len(xy) - 1, 1)
evals, evecs = np.linalg.eigh(cov)
long_axis = evecs[:, int(np.argmax(evals))]
```

### 2.2 연산 비용 분석

PCA 파이프라인은 크게 두 단계로 나뉜다.

**공분산 행렬 계산 (`xy.T @ xy`):**

`xy`는 `(M, 2)` 행렬(M = stride 적용 후 유효 포인트 수)이다. `xy.T @ xy`는 `(2, M) @ (M, 2)` 행렬 곱으로 결과는 항상 `(2, 2)` 행렬이다. 연산량은 O(M)이며 numpy의 BLAS(Basic Linear Algebra Subprograms, 기초 선형대수 서브루틴) 백엔드가 C/Fortran 수준으로 실행한다. 이 단계는 M이 수천 이하인 경우 수십 마이크로초(μs) 수준으로 실행된다(추정).

**고유값 분해 (`np.linalg.eigh`):**

`eigh`는 실수 대칭 행렬 전용 LAPACK(Linear Algebra PACKage) 루틴 `dsyevd`를 사용한다. 대상이 항상 `(2, 2)` 행렬이므로 연산량은 입력 포인트 수 M에 완전히 무관하다 — 고정 O(1) 비용이다. `eigh`를 일반 `eig` 대신 사용하는 이유는 두 가지다: (a) 공분산 행렬이 실수 대칭 행렬임이 수학적으로 보장되므로 `eigh`의 가정이 항상 성립하며, (b) 결과 고유벡터가 항상 실수이고 이미 단위벡터로 정규화돼 있다.

전체 PCA 단계의 지배적 비용은 `xy.T @ xy` (O(M)) 이지, `eigh` (O(1)) 가 아니다.

### 2.3 `cloud_stride`와 PCA 비용의 관계

PCA에서 M = (N_original / stride) × (유효 포인트 비율)이다. stride=4를 적용하면 `xy.T @ xy` 의 행렬 곱 비용도 stride에 반비례해 줄어든다. 좌표 변환 단계(`(R @ pts_cam.T).T + t`, `openarmx_pick/grasp_pose_node.py:178`)도 마찬가지로 O(M)이다.

즉 `cloud_stride`는 두 가지 병목을 동시에 완화한다: (1) `_read_xyz` Python 루프, (2) 좌표 변환 + 공분산 행렬 계산.

---

## 3. latched QoS — `TRANSIENT_LOCAL depth=1`의 의미

### 3.1 구현

`openarmx_pick/grasp_pose_node.py:141-145`:

```python
latched = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
self.pose_pub = self.create_publisher(PoseStamped, gp("grasp_pose_topic").value, latched)
self.marker_pub = self.create_publisher(MarkerArray, "/openarmx/grasp_markers", latched)
```

`/openarmx/grasp_pose`와 `/openarmx/grasp_markers` 두 토픽이 이 QoS를 사용한다.

### 3.2 `TRANSIENT_LOCAL`의 의미

ROS2(Robot Operating System 2)의 DDS(Data Distribution Service, 데이터 분산 서비스) 계층에서 `TRANSIENT_LOCAL` durability는 발행자(publisher)가 마지막으로 발행한 메시지를 내부 캐시에 유지하는 것을 의미한다. 새로운 구독자가 이 토픽에 연결될 때 발행자는 캐시된 메시지를 즉시 전송한다. 이는 ROS1의 "latched topic"(`latch=True`)과 동일한 동작이다. `depth=1`은 캐시 크기가 1임을 의미하므로 항상 가장 최근 메시지 하나만 유지한다.

### 3.3 성능 및 운영 관점의 영향

**장점 — RViz 지연 연결 내성:** 운영자가 RViz(ROS Visualization 도구)를 grasp_pose_node 기동 이후에 열더라도, 마지막 grasp 포즈와 마커를 즉시 수신한다. 연결 시점에 새 카메라 프레임이 도착하기를 기다릴 필요가 없다.

**장점 — 시스템 재연결 안정성:** 소비자 노드(예: 상위 FSM(Finite State Machine, 유한 상태 기계))가 일시적으로 재시작되어 구독을 재등록한 경우에도 마지막 grasp 포즈를 즉시 받는다.

**비용 — QoS 불일치 주의:** `TRANSIENT_LOCAL` 발행자에 `VOLATILE` (기본 durability) 구독자가 연결하면 DDS가 QoS 불일치를 감지하고 연결을 거부한다. `/openarmx/grasp_pose`를 구독하는 모든 노드는 `TRANSIENT_LOCAL` durability로 QoS를 명시해야 한다. 일치하지 않으면 토픽이 발행되더라도 구독자가 수신하지 못하는 침묵 장애(silent failure)가 발생한다.

**비용 — 메모리 유지:** 발행자가 메시지 하나를 DDS 레이어에 유지한다. `PoseStamped`와 `MarkerArray(1개 마커)`는 수백 바이트 수준이므로 메모리 비용은 무시할 수 있다.

**`/openarmx/left/movel` QoS와의 대조:**

`openarmx_pick/grasp_pose_node.py:148`:

```python
self.movel_pub = self.create_publisher(MoveL, gp("movel_topic").value, 10)
```

MoveL 토픽은 depth=10의 기본 QoS(VOLATILE durability)를 사용한다. solver가 latched 방식으로 과거 MoveL을 재수신할 필요가 없기 때문이다 — solver는 명령이 들어오는 순간에만 trajectory를 갱신한다.

---

## 4. MoveL debounce — solver trajectory 재시작 방지

### 4.1 문제: 매 프레임 MoveL 발행 시 발생하는 현상

카메라가 10 Hz로 동작한다고 가정하면 `_on_cloud` 콜백이 초당 10회 호출된다. debounce 없이 매 콜백마다 MoveL을 발행하면:

1. solver(`omx_movel_controller_node`)가 새 목표 포즈를 수신할 때마다 현재 trajectory를 초기화하고 새 trajectory 계획을 시작한다.
2. solver의 제어 루프 주파수는 `control_frequency=100 Hz` (`launch/openarmx_movel.launch.py:76`)로 설정돼 있다.
3. 100 Hz 제어 루프 100번 중 10번(10Hz 카메라 주기마다) trajectory가 재시작된다.
4. 매 trajectory 재시작 시 현재 관절 속도에서 zero까지 감속 후 새 trajectory로 가속한다 — 팔이 목표를 향해 전진하는 대신 정지·재시작을 반복하며 실질적으로 거의 움직이지 않는(crawl) 현상이 발생한다.

코드 주석이 이 문제를 명확히 기술하고 있다 (`openarmx_pick/grasp_pose_node.py:114-119`):

```python
# Debounce so we do NOT re-send a MoveL every camera frame (which would
# restart the solver's trajectory each cycle -> the arm only crawls).
# Re-send only when the target shifts > send_min_delta OR after a
# send_min_interval cooldown (>= move_time lets a motion finish first).
self.declare_parameter("send_min_interval", 5.0)  # s cooldown between sends
self.declare_parameter("send_min_delta", 0.02)    # m target shift to re-send
```

### 4.2 `_should_send` 구현

`openarmx_pick/grasp_pose_node.py:226-238`:

```python
def _should_send(self, pre_xyz) -> bool:
    now = self.get_clock().now().nanoseconds * 1e-9
    if self._last_sent_xyz is None:
        self._last_sent_xyz, self._last_sent_t = np.asarray(pre_xyz), now
        return True
    moved = float(np.linalg.norm(np.asarray(pre_xyz) - self._last_sent_xyz))
    elapsed = now - self._last_sent_t
    if moved > self.send_min_delta or elapsed > self.send_min_interval:
        self._last_sent_xyz, self._last_sent_t = np.asarray(pre_xyz), now
        return True
    return False
```

debounce 상태는 `_last_sent_xyz`(마지막 발행 위치)와 `_last_sent_t`(마지막 발행 시각) 두 멤버 변수로 관리된다 (`openarmx_pick/grasp_pose_node.py:132-133`).

### 4.3 두 조건의 역할 분리

**조건 1 — `moved > send_min_delta` (기본값: 0.02 m = 2 cm):**

박스 또는 카메라 위치의 실질적 변화를 감지한다. 예를 들어 컨베이어 벨트로 박스가 이동하거나 카메라 TF 추정이 갱신돼 grasp 포즈가 2 cm 이상 이동한 경우 즉시 새 MoveL을 발행한다. `send_min_interval` 쿨다운 여부와 무관하게 반응하므로 동적 환경에서 팔이 갱신된 목표를 추종할 수 있다.

**조건 2 — `elapsed > send_min_interval` (기본값: 5.0 s):**

박스 위치가 거의 변하지 않는(정적) 상황에서도 주기적으로 MoveL을 재발행해 solver가 목표를 "잊지 않도록" 유지한다. 또한 최초 발행 후 solver가 목표에 도달하지 못한 경우(예: 외란으로 trajectory가 중단됨) 자동 복구 수단이 된다.

### 4.4 `send_min_interval >= move_time` 설계 의도

기본값 비교:

| 파라미터 | 기본값 |
|----------|--------|
| `move_time` | 4.0 s |
| `send_min_interval` | 5.0 s |

`send_min_interval`(5.0 s)이 `move_time`(4.0 s)보다 크게 설정된 것은 의도적이다. `move_time`은 solver에게 전달되는 `time_from_start` 필드 (`openarmx_pick/grasp_pose_node.py:247-248`) 값으로, solver가 이 시간 안에 목표 포즈에 도달하도록 trajectory를 계획하는 기준이다:

```python
m.time_from_start.sec = int(self.move_time)
m.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
```

`send_min_interval >= move_time`을 보장하면:

- 첫 MoveL 발행(t=0) → 팔이 4.0 s 동안 trajectory 실행 → 도달
- 쿨다운 만료(t=5.0 s) → 박스가 거의 그대로이면 재발행 생략 또는 재확인
- 박스가 이동하면(moved > 2 cm) → 즉시 새 MoveL 발행

이 설계에서 trajectory가 실행 중인 4.0 s 동안 solver는 새로운 MoveL을 받지 않으므로 재시작 없이 연속적으로 목표를 향해 이동한다.

### 4.5 `_should_send`의 연산 비용

`_should_send` 자체의 비용은 무시할 수준이다: `np.linalg.norm` 한 번 (3차원 벡터 차), Python 시간 조회 한 번, 비교 연산 두 번. 전체 `_on_cloud` 콜백 대비 기여 비율은 1% 미만으로 추정된다.

---

## 5. 콜백 주기 대비 연산량 요약

### 5.1 `_on_cloud` 콜백 단계별 비용 분류

카메라 주기 ≈ 10 Hz, 즉 콜백 예산은 약 100 ms이다.

| 단계 | 구현 위치 | 복잡도 | 실행 레이어 | 비용 수준 |
|------|-----------|--------|------------|----------|
| `_read_xyz` (stride=4) | `grasp_pose_node.py:46-62` | O(N/4) Python 루프 | CPython | 중간 — 지배적 병목 |
| TF2 lookup | `grasp_pose_node.py:170-176` | O(1) | C++ (tf2 내부) | 낮음 |
| `_tf_to_Rt` (quaternion → R) | `grasp_pose_node.py:205-215` | O(1) | numpy | 무시 |
| 좌표 변환 `(R @ pts_cam.T).T + t` | `grasp_pose_node.py:178` | O(M) BLAS | numpy/BLAS | 낮음 |
| centroid 계산 | `grasp_pose_node.py:180` | O(M) | numpy | 낮음 |
| XY 뺄셈 + 공분산 `xy.T @ xy` | `grasp_pose_node.py:181-182` | O(M) BLAS | numpy/BLAS | 낮음 |
| `np.linalg.eigh` (2×2) | `grasp_pose_node.py:183` | O(1) LAPACK | numpy/LAPACK | 무시 |
| `_grasp_rotation` | `grasp_pose_node.py:83-97` | O(1) | numpy | 무시 |
| `_quat_from_matrix` | `grasp_pose_node.py:65-80` | O(1) | math/numpy | 무시 |
| `_publish_pose`, `_publish_marker` | `grasp_pose_node.py:217-262` | O(1) | Python/ROS2 | 낮음 |
| `_should_send` + (선택) `_send_movel` | `grasp_pose_node.py:226-249` | O(1) | numpy/Python | 무시 |

### 5.2 지배적 병목

`_read_xyz`의 Python 루프가 단일 최대 병목이다. 다른 모든 단계는 numpy/BLAS/LAPACK이 C 레이어에서 실행하거나 O(1) 연산이다.

D435의 `/box_plane/cloud`가 최대 수천 포인트 규모라고 가정하면(추정), stride=4 적용 후 루프 반복 횟수는 수백에서 수천 회 수준이다. CPython의 인터프리터 오버헤드를 고려하면 이 단계가 전체 콜백 시간의 절반 이상을 차지할 가능성이 높다(추정).

---

## 6. 잠재적 병목과 개선 방향

이 섹션의 내용은 분석에서 도출한 **개선 제안**이다. 현재 코드를 수정하지 않는다.

### 6.1 `_read_xyz` 벡터화 — `numpy.frombuffer` + structured dtype

가장 직접적인 개선 방향은 Python 루프를 numpy 벡터 연산으로 교체하는 것이다. `PointCloud2.data`는 bytes 또는 array.array 객체이므로 `numpy.frombuffer`로 dtype을 지정해 직접 배열로 읽을 수 있다.

개념적 구현:

```python
import numpy as np

def _read_xyz_fast(cloud, stride=1):
    off = {f.name: f.offset for f in cloud.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return np.empty((0, 3))
    step = cloud.point_step
    n = cloud.width * cloud.height
    # structured dtype: 포인트 하나를 step 바이트 raw 배열로 읽음
    dt = np.dtype([("raw", np.uint8, step)])
    buf = np.frombuffer(bytes(cloud.data), dtype=dt)
    buf = buf[::stride]  # stride 서브샘플
    # x, y, z 오프셋에서 float32 직접 추출
    ox, oy, oz = off["x"], off["y"], off["z"]
    x = buf["raw"][:, ox:ox+4].view(np.float32).reshape(-1)
    y = buf["raw"][:, oy:oy+4].view(np.float32).reshape(-1)
    z = buf["raw"][:, oz:oz+4].view(np.float32).reshape(-1)
    mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return np.column_stack([x[mask], y[mask], z[mask]]).astype(np.float64)
```

대안으로, x/y/z가 연속하여 저장되고 step이 12바이트(float32×3)인 경우 단순히:

```python
arr = np.frombuffer(bytes(cloud.data), dtype=np.float32).reshape(n, step // 4)
xyz = arr[::stride, [0, 1, 2]]  # ox/oy/oz가 0,4,8이면 열 인덱스는 0,1,2
mask = np.isfinite(xyz).all(axis=1)
return xyz[mask].astype(np.float64)
```

이 방식은 Python 루프를 완전히 제거하고 전체 버퍼를 한 번의 C 수준 복사로 numpy 배열로 변환한다. 예상 속도 향상은 포인트 수에 따라 10배 ~ 100배 수준으로 추정된다.

**주의**: `bytes(cloud.data)` 변환이 추가 복사를 유발할 수 있다. `cloud.data`가 `bytearray`이면 `memoryview(cloud.data)` 또는 `np.frombuffer(cloud.data, ...)` 를 직접 사용하는 것이 더 효율적이다.

### 6.2 stride 파라미터 자적응(adaptive stride)

현재 `cloud_stride=4`는 고정 파라미터다. 박스 크기나 카메라 거리에 따라 inlier 포인트 수가 크게 달라질 수 있다. 포인트 수가 많을 때는 stride를 더 크게, 적을 때는 stride=1로 자동 조정하는 적응형 서브샘플링을 적용하면 PCA 품질을 유지하면서 최악 케이스 비용을 줄일 수 있다(추정). 이는 코드 수정 없이 런타임에 ROS2 파라미터 서버로 `cloud_stride`를 동적 변경하는 방식으로도 부분적으로 구현 가능하다.

### 6.3 `np.linalg.eigh` vs. 2×2 해석해

`np.linalg.eigh`는 2×2 대칭 행렬에 대해 LAPACK 루틴을 호출한다. 2×2 행렬의 고유값/고유벡터는 해석적으로 닫힌 형식(closed-form)으로 계산할 수 있다:

```
공분산 행렬 [[a, b], [b, c]] 의 고유값:
  λ = ((a+c) ± sqrt((a-c)^2 + 4b^2)) / 2
```

이를 직접 구현하면 LAPACK 호출 오버헤드(함수 디스패치, 조건 검사 등)를 제거할 수 있다. 다만 2×2 `eigh`의 절대적 실행 시간이 이미 수 마이크로초(μs) 수준으로 매우 낮으므로, 전체 파이프라인에서 이 최적화의 영향은 미미하다(추정).

### 6.4 `_read_xyz` 이후 `np.asarray` 변환 비용

현재 구현에서 `out`은 Python 튜플의 리스트이고, 루프 종료 후 `np.asarray(out, dtype=np.float64)`로 변환한다. 이 변환은 O(M) 시간과 새 배열 메모리 할당을 요구한다. `numpy.frombuffer` 기반 벡터화 구현으로 교체하면 이 단계가 불필요해진다.

### 6.5 단일 스레드 executor와 콜백 지연

`rclpy.spin(node)` (`openarmx_pick/grasp_pose_node.py:269`)는 기본 `SingleThreadedExecutor`를 사용한다. `_on_cloud` 콜백이 실행되는 동안 다른 콜백(`_on_info`, TF2 리스너 내부 콜백)은 블록된다. `_read_xyz` Python 루프가 길어지면 TF2 타임스탬프 갱신이 지연돼 다음 `_on_cloud` 호출에서 오래된 TF를 사용하거나 TF lookup 실패 가능성이 높아진다(추정). 벡터화로 `_read_xyz` 비용을 줄이면 이 문제도 간접적으로 완화된다.

---

## 7. 종합

`openarmx_pick/grasp_pose_node.py`의 성능 특성을 요약하면 다음과 같다.

`_read_xyz`의 Python `struct.unpack_from` 루프가 단일 최대 비용 요소이며, `cloud_stride=4` 파라미터가 반복 횟수를 1/4로 줄여 이를 가장 직접적으로 완화한다. PCA 단계는 `xy.T @ xy` (O(M) BLAS)와 `np.linalg.eigh` (O(1) LAPACK, 2×2 고정)로 구성되며 전체 비용에서 차지하는 비율이 낮다.

latched QoS(`TRANSIENT_LOCAL depth=1`)는 grasp 포즈와 RViz 마커에 적용돼 늦게 연결되는 구독자에게 마지막 상태를 즉시 전달하며, MoveL 토픽은 latching이 없는 표준 QoS를 사용해 solver가 과거 명령을 재수신하지 않도록 한다.

MoveL debounce(`send_min_delta=0.02 m` / `send_min_interval=5.0 s`)는 카메라 주기마다 발행되는 MoveL이 solver trajectory를 매 프레임 재시작시켜 팔이 crawl하는 현상을 방지하는 핵심 메커니즘이다. `send_min_interval >= move_time` 설계로 한 trajectory 실행이 완료된 후에야 다음 명령이 발행된다.

개선 여력은 `_read_xyz`의 `numpy.frombuffer` + structured dtype 벡터화에 집중돼 있으며, 이 변환은 Python 루프를 완전히 제거해 10배 이상의 속도 향상을 기대할 수 있다.
