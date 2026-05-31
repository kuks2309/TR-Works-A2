# 평면 검출 알고리즘 — 상세

`box_detection.analyze_planes.analyze_cloud()` 의 처리 흐름. 입력은 카메라 한 대의 누적 PointCloud (N × 3, optical frame), 출력은 ground / box-top 평면 방정식.

---

## 1. 파이프라인 개요

```
PointCloud (optical frame)
        │
        ▼
[1] 작업영역 crop  ───────┐
        │                 │
        ▼                 │  점이 너무 적으면 None 반환 (min 1000)
[2] 4-회 RANSAC ──────────┘
        │
        ▼
[3] 모든 plane normal을 +y(=down) 으로 통일
        │
        ▼
[4] 가장 큰 plane을 reference로,
    angle < 15° 인 plane만 parallel 후보로 채택
        │
        ▼
[5] parallel 후보를 d 오름차순 정렬
    → ground = d 가장 작은 (가장 음수, 카메라에서 가장 멀리)
        │
        ▼
[6] ground 보다 d 가 +3 cm 이상 큰 후보 중
    inlier 가장 많은 것 = box_top
        │
        ▼
ground / box_top  +  inlier 점들
```

---

## 2. 단계별 상세

### [1] crop

RealSense optical frame 가정 (`+x = right, +y = down, +z = forward`). 작업대 영역만 남기고 나머지(벽, 천장, 카메라 뒤) 제거:

| 축 | 범위 | 의미 |
|---|---|---|
| z | 0.2 ~ 2.5 m | 카메라 앞 0.2~2.5 m 구간 (책상은 ~0.8 m, 벽은 제외) |
| x | abs < 1.5 m | 좌우 ±1.5 m |
| y | abs < 1.5 m | 위아래 ±1.5 m |

또한 NaN 점 자동 제거(`pc2.read_points(skip_nans=True)`).

### [2] 반복 RANSAC

`open3d.geometry.PointCloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=2000)` 을 최대 4회 반복:

- 매 회 inlier 제거 후 남은 점에서 다시 plane 추출
- 추출된 plane의 inlier 수가 500 미만이면 중단
- 결과는 `[(coef, n_inliers, inlier_pts, centroid), ...]` 리스트

> 4회로 제한한 이유: 책상 + 박스 윗면 + (가끔) 벽/박스 옆면이면 4개로 충분. 더 늘리면 노이즈 plane만 늘어남.

### [3] normal canonicalization

각 plane `[a, b, c, d]` 에 대해 `b < 0` 이면 모든 부호 반전 → `b ≥ 0`. RealSense optical frame에서 `+y = down` 이라 모든 평면 normal이 "아래쪽" 방향으로 통일됨.

이렇게 하면 같은 평면이 RANSAC seed에 따라 부호 다르게 나오는 비결정성을 제거. 이후 `d` 값으로 평면들을 직접 비교 가능.

### [4] horizontal filter

가장 큰 plane(inlier 최다)을 "수평 reference" 로 잡고, 다른 plane들과의 각도가 15° 이내면 후보로 인정.

> 책상과 박스 윗면은 거의 평행이라는 사전 지식 사용. 책상이 카메라에 대해 살짝 기울어 있어도(보통 ±5°) 충분.

추가로 `n_inliers >= 1500` 인 후보만 유효로 간주(노이즈 평면 제거).

### [5] ground 결정

`+y` 방향으로 통일된 normal이 주어졌을 때, 카메라 원점 `(0,0,0)` 을 평면 식에 대입한 값이 정확히 `d`. 즉:

- `d > 0` → 카메라 원점이 평면의 `+normal` 쪽 (=평면이 카메라 위쪽에 있음)
- `d < 0` → 카메라 원점이 `-normal` 쪽 (=평면이 카메라 아래쪽에 있음)

normal을 `+y(=down)` 으로 고정했으므로 `d 가 작을수록(=음수 절댓값 크다)` 평면이 카메라보다 더 아래에 위치 → **가장 작은 d가 책상**.

```python
parallel.sort(key=lambda c: c["coef"][3])
ground = parallel[0]
```

### [6] box-top 결정

ground 보다 `d`가 +3 cm 이상 큰 후보(=ground 보다 위에 있는 평면) 중 inlier 가장 많은 것을 box-top으로.

```python
above = [c for c in parallel[1:] if c["coef"][3] > ground["coef"][3] + 0.03]
box_top = max(above, key=lambda c: c["n_inliers"]) if above else None
```

`+0.03 m` 임계는 RealSense depth noise(약 ±5 mm @ 1 m)를 안전하게 넘기 위한 값.

---

## 3. 5-frame 평균 (live & bag 공통)

단일 PointCloud의 inlier 분포가 박스/그림자에 따라 흔들려 ground/box-top 라벨이 뒤바뀌는 케이스가 있어, **시간상 등간격 5 프레임을 vstack하여 분석**.

bag 모드 (`extract_sampled_cloud`):
- bag의 PointCloud2 메시지 N개 중 0%, 25%, 50%, 75%, 100% 시점 5개 선택
- N ≤ 5 면 전체 사용

live 모드 (`live_plane_detector._on_cloud`):
- `collections.deque(maxlen=5)` 로 최근 5 프레임 유지
- timer (1 Hz 기본)에서 vstack 후 분석

5 프레임 합치면 점 수 ≈ 5 × 30k = 150k → RANSAC 수행시간 ≈ 100 ms (Open3D 0.19, x86_64 single-thread).

---

## 4. 시각화 출력

`visualize_planes.PlanePublisher` / `live_plane_detector.LivePlaneDetector` 가 다음을 발행:

### 4.1 `/plane_markers` (`MarkerArray`)

각 카메라 × {ground, box-top} × {plane CUBE, normal arrow} = 최대 8 marker.

- **plane CUBE**:
  - center = `−d * n` (평면 위 원점에 가장 가까운 점)
  - orientation = `quat([0,0,1] → n)` (CUBE z축이 normal과 정렬)
  - size = 0.8 × 0.8 × 0.003 m (ground), 0.5 × 0.5 × 0.003 m (box-top)
  - color = (0.2, 0.9, 0.2) ground / (0.95, 0.2, 0.2) box-top, alpha 0.45
- **normal arrow**:
  - base = `−d * n`, tip = `−d * n + 0.25 * n`

### 4.2 `/plane_inliers/{ground,box_top}` (`PointCloud2`)

inlier 점들을 `xyzrgb` 포맷으로 패킹(`make_colored_pointcloud2`):
- ground inlier → 녹색 `(60, 230, 60)`
- box-top inlier → 빨강 `(240, 60, 60)`
- 카메라 0번(center)의 inlier만 단일 cloud로 발행 (양 카메라를 합치려면 캘리브레이션 필요)

QoS 모두 `RELIABLE` + `TRANSIENT_LOCAL` (latched) — RViz 늦게 켜도 즉시 받음.

---

## 5. 파라미터 요약 (튜닝 가능)

| 파라미터 | 위치 | 기본값 | 영향 |
|---|---|---|---|
| `distance_threshold` | `analyze_planes.fit_plane` | 0.01 m | RANSAC plane 두께 허용오차 |
| `ransac_n` | 동상 | 3 | plane fit에 쓸 sample 수 |
| `num_iter` | 동상 | 2000 | RANSAC 반복 |
| 4-iter loop | `analyze_cloud` | 4 | 추출할 candidate plane 수 |
| angle < 15° | 동상 | 15 deg | parallel 판정 |
| 3 cm 임계 | 동상 | 0.03 m | box-top 최소 높이 |
| crop 범위 | 동상 | x±1.5, y±1.5, z 0.2~2.5 | 작업영역 |
| `frame_buffer` | `live_plane_detector` (param) | 5 | 누적 frame 수 |
| `refresh_rate` | 동상 | 1.0 Hz | 분석 주기 |
| `min_points` | 동상 | 20000 | 분석 최소 누적 점 수 |
