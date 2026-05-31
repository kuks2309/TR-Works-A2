# box_detection — 박스 / 책상 평면 검출 패키지

TR-Works 시스템의 RealSense D435 스트림(또는 rosbag)에서 **책상(ground) 평면**과 **박스 윗면(box-top) 평면**의 평면 방정식 `ax+by+cz+d=0` 을 RANSAC으로 도출하고, RViz에 시각화하는 ROS2 Humble 패키지.

> 자매 문서: [results.md](results.md) (실측 결과 표) · [algorithm.md](algorithm.md) (알고리즘 상세) · [../issues_and_fixes/box_detection.md](../issues_and_fixes/box_detection.md) (트러블슈팅)

---

## 1. 구성

| 노드 / CLI | 역할 | 입력 | 출력 |
|---|---|---|---|
| `analyze_planes` | bag 일괄 분석 (CLI) | rosbag2 디렉토리 | 콘솔 평면 방정식 + 박스 높이 |
| `visualize_planes` | bag 분석 결과 RViz 시각화 노드 | rosbag2 디렉토리 | `/plane_markers`, `/plane_inliers/{ground,box_top}` |
| `live_plane_detector` | 실시간 카메라 스트림 검출 노드 | `/d435_center/...`, `/d435_center_upper/...` PointCloud2 | 동상 |

| Launch | 용도 |
|---|---|
| `visualize.launch.py` | bag을 재생하면서 평면 RViz 오버레이 |
| `live.launch.py` | D435 카메라 2대 launch + 실시간 검출 + RViz 통합 |

---

## 2. 설치 / 빌드

```bash
cd <ros2_ws>
git clone https://github.com/kuks2309/TR-Works-Box-Detection.git src/box_detection
# 또는 워크스페이스 자체를 clone하면 src/box_detection 포함됨

# 의존: ros-humble-realsense2-camera (apt) — live.launch.py 용
sudo apt install -y ros-humble-realsense2-camera

colcon build --packages-select box_detection --symlink-install
source install/setup.bash
```

런타임 Python 의존성: `numpy`, `open3d` (RANSAC).

---

## 3. 사용법

### 3.1 rosbag 분석 (오프라인)

```bash
# 모든 bag 분석 (한 줄씩 결과 출력)
ros2 run box_detection analyze_planes --root rosbag/bags/20260506

# 한 bag만
ros2 run box_detection analyze_planes \
    --root rosbag/bags/20260506 --bag short_box_wide_a
```

분석 결과 표는 [results.md](results.md) 참조.

### 3.2 bag을 RViz에 시각화

```bash
ros2 launch box_detection visualize.launch.py bag:=short_box_wide_a
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `bag` | `short_box_wide_a` | 재생할 bag 폴더명 |
| `bag_root` | `…/rosbag/bags/20260506` | bag 폴더의 부모 |
| `play` | `true` | bag을 자동 재생 (loop+clock) |
| `rviz` | `true` | RViz 자동 기동 |

### 3.3 실시간 카메라 검출

```bash
# (선택) robot_state_publisher가 d435_center_link / d435_center_upper_link TF를
# 발행하는 외부 ws launch를 먼저 띄움 — HOWTO.md §1.2 참고

ros2 launch box_detection live.launch.py
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `center_serial` | `_818312070932` | d435_center 시리얼 |
| `upper_serial` | `_819612070814` | d435_center_upper 시리얼 |
| `refresh_rate` | `1.0` Hz | 평면 갱신 주기 |
| `rviz` | `true` | RViz 자동 기동 |

---

## 4. 토픽

발행 (모두 latched / `TRANSIENT_LOCAL` + `RELIABLE`):

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/plane_markers` | `visualization_msgs/MarkerArray` | ground/box-top 평면 CUBE + normal arrow (양 카메라) |
| `/plane_inliers/ground` | `sensor_msgs/PointCloud2` | RGB8 색칠된 ground inlier 점들 (녹색) |
| `/plane_inliers/box_top` | `sensor_msgs/PointCloud2` | 동상 (빨간색) |

구독 (실시간 모드만):

| 토픽 | QoS |
|---|---|
| `/d435_center/d435_center/depth/color/points` | BEST_EFFORT, VOLATILE |
| `/d435_center_upper/d435_center_upper/depth/color/points` | 동상 |

---

## 5. 좌표계 / Frame 가정

- 모든 평면 방정식은 **각 카메라의 RealSense optical frame** 기준 (`d435_center_depth_optical_frame`, `d435_center_upper_depth_optical_frame`).
  - `+x = right`, `+y = down`, `+z = forward`.
- `live_plane_detector`는 두 카메라가 **별개 TF tree**여도 동작하지만, 양 카메라를 한 RViz 화면에서 함께 보려면 외부 robot_state_publisher가 `base_link → d435_center_link / d435_center_upper_link` 정적 TF를 발행해야 합니다.
- 패키지 자체에는 카메라 외부 캘리브레이션이 없습니다. 팀 정책에 따라 robot URDF(외부 ws)에서 관리.

---

## 6. RViz 화면 구성 (`config/box_analysis.rviz`)

- Fixed Frame: `d435_center_depth_frame` (z=up이라 책상 위에 박스가 자연스럽게 표시)
- Display
  - `PointCloud2 - center` / `PointCloud2 - upper` — 카메라 원본
  - `Ground inliers` (녹) / `Box-top inliers` (빨) — RANSAC 결과 점들
  - `Plane markers` — 평면 CUBE + 법선 화살표
  - TF axes

---

## 7. 한계 · 미해결

- 양 카메라를 같은 frame에서 합쳐 분석하지 않음 (외부 캘리브레이션 필요). 두 카메라 결과는 독립.
- 박스 윗면이 시야의 대부분을 차지하면 RANSAC이 ground를 못 찾음 (예: `big_box_a/upper` 케이스). [issues_and_fixes/box_detection.md](../issues_and_fixes/box_detection.md) 참조.
- 검출 주기 1 Hz 기본 — 더 빠르게 하려면 `refresh_rate` 인자 늘리거나 RANSAC 반복 수 줄이기.
