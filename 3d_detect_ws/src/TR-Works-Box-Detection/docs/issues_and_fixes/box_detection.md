# box_detection — Issues & Fixes

평면 검출 패키지 개발/운용 중 만난 문제와 해결 기록.

---

## 1. RANSAC이 책상과 박스 윗면을 뒤집어 라벨링

### 증상
초기 버전에서 `inlier 수가 가장 많은 plane = ground` 로 가정. `big_box_a` / `medium_box_*` 처럼 박스가 시야 큰 비중을 차지하는 경우 박스 윗면 inlier 수 > 책상 inlier 수가 되어 ground와 box-top이 뒤집힘.

### 원인
"수평 plane 중 가장 큰 것" 휴리스틱이 박스가 클 때 무너짐. inlier 수는 박스 크기와 시야 각도에 따라 변동이 커서 ground 식별에 부적합.

### 해결
inlier 수 대신 **카메라 원점에서의 거리**로 ground 결정.

```python
# normal을 +y(=down) 방향으로 통일하면, 평면 식 a x + b y + c z + d = 0 에
# 카메라 원점 대입 결과가 d. d가 가장 작은 것이 카메라 아래쪽으로 가장 멀리 있는 plane.
parallel.sort(key=lambda c: c["coef"][3])
ground = parallel[0]
```

5개 bag 전체에서 라벨이 정상 정렬됨 ([results.md](../box_detection/results.md) §3).

---

## 2. 단일 프레임 분석에서 RANSAC이 박스 윗면 두 번 잡음

### 증상
`short_box_narrow_a/upper` 에서 ground와 box-top의 `d` 차이가 3 mm (`gap=0.0033 m`). 서로 다른 plane이 아니라 같은 박스 윗면을 RANSAC이 두 번 fit한 것.

### 원인
박스가 시야 대부분을 차지해서 책상이 가장자리만 일부 잡힘. RANSAC이 inlier 많은 박스 윗면을 둘로 분할.

### 해결
**bag duration 등간격 5 프레임을 합쳐 분석**(`extract_sampled_cloud(n_frames=5)`).

박스가 시야를 가린 시점뿐 아니라 그렇지 않은 시점도 포함되어 책상 점이 충분히 누적됨. 1 frame 당 ~30k 점 → 5 frame ~150k 점 → 책상이 항상 충분한 inlier 확보.

---

## 3. RViz Fixed Frame을 optical frame에 두니 박스가 책상 아래에 표시됨

### 증상
`d435_center_depth_optical_frame` 을 Fixed Frame으로 두니 RViz 화면에서 책상(녹) 평면이 박스(빨) 평면보다 위에 표시되어 직관과 반대.

### 원인
RealSense optical frame은 `+y = down` 규약. RViz Orbit camera가 +z를 위로 두려 하지만 optical frame에서 +z는 forward라 화면 회전 결과 +y가 위로 가게 되어 거꾸로 보임.

### 해결
Fixed Frame을 ROS 표준 frame인 **`d435_center_depth_frame`** (`+x = forward, +y = left, +z = up`) 으로 변경. `/tf_static` 에 `depth_frame → depth_optical_frame` 변환이 있어 RViz가 자동 변환.

`config/box_analysis.rviz`:
```yaml
Global Options:
  Fixed Frame: d435_center_depth_frame
```

---

## 4. base_link로 변환할 수 없음 — bag에 카메라↔로봇 TF 없음

### 증상
초기 분석 시 `chain_transform(... 'base_link')` 가 항상 `None` → 평면 분석 실패.

### 원인
bag의 `/tf_static` 에는 RealSense 내부 frame (`d435_center_link → ..._color/depth_frame → ..._optical_frame`) 만 존재. `base_link → d435_center_link` 변환은 외부 워크스페이스의 robot URDF가 발행하는데 그 TF가 bag에 기록되지 않음.

```
/tf_static unique edges:
  d435_center_link → d435_center_color_frame
  d435_center_depth_frame → d435_center_depth_optical_frame
  ...
  (base_link 없음)
```

### 해결 (현재 패키지)
- 분석을 **각 카메라 optical frame에서 독립 수행**. 결과 평면 방정식도 카메라 frame 기준.
- 양 카메라를 한 좌표계에서 합치려면 외부 calibration TF 필요 — 패키지 책임 밖. 외부 robot URDF (`d435_center_link / d435_center_upper_link` joint) 가 발행해야 함.

### 향후
calibration 데이터가 들어오면:
- bag 분석 시 `base_link → cam_*` TF를 외부에서 받거나 yaml로 주입
- 평면을 base_link 좌표계로 통일해 발행

---

## 5. `big_box_a/upper` outlier — RANSAC이 책상 대신 수직 평면을 잡음

### 증상
`big_box_a/upper` 만 ground 평면 normal이 다른 bag과 정반대 (`b≈+0.49`, `c≈-0.87`). `d=+0.42`. 박스 윗면도 비슷한 normal에 `d=+1.75`. 즉 두 plane이 모두 "수직"이고 두 vert plane 사이 거리가 1.3 m.

### 원인
big_box가 upper 카메라 시야의 거의 전부를 차지. 박스 옆면 + 박스 너머 벽이 RANSAC의 큰 후보가 되어 책상이 안 잡힘.

### 임시 회피
- center 카메라 결과 사용 (정상).
- upper 카메라 결과는 detection 실패 케이스로 분류, [results.md](../box_detection/results.md) §1.2에 명시.

### 향후 개선 후보
- crop 범위를 더 좁게 (z 0.2 → 0.5, |y| 1.5 → 0.8)
- "수평 reference" 를 단일 카메라 내부에서 결정하지 않고 **타 카메라(center)의 ground normal을 reference로 강제** (양 카메라 calibration 필요)
- 수직 plane(angle to z-axis < 30°) 후보 자동 제외

---

## 6. PointCloud2 inlier 발행 시 Python `struct.pack_into` 루프가 너무 느림

### 증상
초기 구현에서 27만 점을 1 Hz로 패킹하려니 Python loop가 0.5 초 이상 걸려 노드 응답 지연. RViz publishing이 2초 간격으로 늦어짐.

### 원인
점 하나마다 `struct.pack_into("ffff", ...)` 호출 → CPython 함수 호출 오버헤드 누적.

### 해결
numpy `tobytes()` 일괄 변환:
```python
arr = np.zeros((n, 4), dtype=np.float32)
arr[:, 0:3] = points_xyz.astype(np.float32)
arr[:, 3] = rgb_float
msg.data = arr.tobytes()
```
27만 점 인코딩 ~30 ms → 1 Hz 정시 발행.

---

## 7. cycle_bags.sh 의 `set -u` 가 ROS setup.bash 와 충돌

### 증상
```
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```
스크립트가 즉시 종료.

### 원인
`set -u` (=nounset) 켜진 상태에서 ROS의 `ament_*.sh` 가 정의되지 않은 환경변수를 참조.

### 해결
`set -u` 제거. ROS setup이 보장하는 변수에 의존.

```diff
-set -u
 cd "$(dirname "$0")"
 source /opt/ros/humble/setup.bash
```

---

## 8. ROS2 launch 에서 작업 디렉토리가 ws 루트로 리셋됨

### 증상
`python3 visualize_planes.py --bag short_box_wide_a` 를 background로 띄우니
```
python3: can't open file '/home/amap/TR-Works/TR-Works_ros2_ws/visualize_planes.py'
```

### 원인
배경 실행 시 cwd가 ws 루트(`/home/amap/TR-Works/TR-Works_ros2_ws`)로 리셋되는데, 스크립트가 `rosbag/` 안에 있음. 또한 launch 파일 작성 시 절대 경로를 명시하지 않으면 같은 문제 발생.

### 해결
- 쉘 스크립트: `cd "$(dirname "$0")"` 로 자기 디렉토리로 이동
- ROS2 launch: bag 경로를 절대 경로로 (`bag_root` 인자 default = `/home/amap/.../rosbag/bags/20260506`)
- 패키지화 후에는 `ros2 run / ros2 launch` 가 cwd 무관하게 동작

---

## 9. open3d `read_points` 결과를 그대로 float64 캐스팅 시 TypeError

### 증상
```python
points = np.array(list(pc2.read_points(...)), dtype=np.float64)
# TypeError: Cannot cast array data from dtype([('x','<f4'),...]) to dtype('float64')
```

### 원인
`sensor_msgs_py.point_cloud2.read_points` 는 structured numpy array (named fields) 반환. 일반 ndarray로 직접 캐스팅 불가.

### 해결
필드별로 추출 후 `np.stack`:
```python
s = pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
points = np.stack([np.asarray(s["x"], dtype=np.float64),
                   np.asarray(s["y"], dtype=np.float64),
                   np.asarray(s["z"], dtype=np.float64)], axis=1)
```

---

## 10. RViz `--loop` 재생 시 "Detected jump back in time" 경고 폭주

### 증상
bag을 `--loop` 재생할 때마다 RViz 콘솔에 `Detected jump back in time. Resetting RViz.` 가 6초마다 출력.

### 원인
정상 — `--loop` 가 반복 시작 시 `/clock` 을 처음부터 다시 발행 → RViz가 sim_time이 과거로 갔다고 판단해 TF buffer 리셋.

### 대응
무시 가능 (실제 표시는 정상). 경고만 신경 쓰일 뿐.

장시간 표시가 필요하면 `--loop` 대신 외부 wrapper에서 끝나면 다시 실행하는 식으로 회피 가능.
