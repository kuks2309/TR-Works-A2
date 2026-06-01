# yolov8_detection

D435 컬러/깊이 → YOLOv8 (Ultralytics) 검출 → **박스 윗면 + 바닥면** RANSAC 평면 핏 → RViz2에서 openarmx URDF 위에 겹쳐 표시.

## 1. 가상환경 / numpy 버전

`ros-humble-cv-bridge`는 `numpy<2`로 컴파일돼 있고 시스템 파이썬에는 `numpy>=2`가 있어서 충돌합니다. 그래서 다음 venv를 쓰고, 거기서 numpy를 1.x로 고정해 둡니다.

```
/home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/yolov8_env/
  numpy==1.26.4, ultralytics==8.4.55, torch, opencv-python, pillow
```

colcon이 빌드할 때 entry-point 셔뱅을 시스템 python으로 생성하므로 [scripts/run_yolov8_ros.sh](../../../Yolo/Yolov8/scripts/run_yolov8_ros.sh)에서 매 실행마다 venv 파이썬으로 다시 박아 줍니다.

## 2. 빌드

```bash
cd /home/openarmx/TR-Works/kkw/China/3d_detect_ws
source /opt/ros/humble/setup.bash
source /home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/yolov8_env/bin/activate
colcon build --packages-select yolov8_detection --symlink-install
```

## 3. 실행 (한 줄)

```bash
/home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/scripts/run_yolov8_ros.sh
```

이 스크립트는 인자 없이 호출하면 검증된 풀 파이프라인을 기동합니다:

```
ros2 launch yolov8_detection yolov8_d435.launch.py \
    rviz:=true \
    show_robot:=true \
    fit_box_plane:=true \
    model:=yolov8l-worldv2.pt \
    prompts:="cardboard box,box,carton,package" \
    confidence:=0.10
```

내부적으로 띄우는 노드:
- `/camera/camera` — realsense2_camera D435 (컬러 + aligned depth + RGBD 포인트클라우드)
- `/yolov8_node` — YOLOv8-World 오픈-보캐뷸러리 검출. **on-demand action server** (`~/detect`). 런치 직후에는 idle (모델만 메모리 상주, 추론 0). venv Python
- `/box_plane_node` — body-프레임 RANSAC 박스 윗면 + 바닥 평면. `/yolov8_node/detections`가 올 때만 동작
- `/robot_state_publisher` + `/joint_state_publisher` — openarmx URDF
- `/openarmx_body_link0_to_camera_link_tf` — 캘리브된 정적 TF
- `/rviz2` — Fixed Frame `openarmx_body_link0`, RobotModel + RGBD 클라우드 + 두 평면 마커

`show_robot:=false`/`fit_box_plane:=false`로 부분 기능만 켤 수도 있습니다.

### 3.1. 검출 요청 (on-demand)

`yolov8_node`는 더 이상 매 프레임 추론하지 않는다(연속 자원 낭비 제거). 검출은 `DetectBox` 액션 goal이 올 때만 **1회** 수행되고, 기존 `~/detections`·`~/image_annotated` 토픽으로 1회 발행된다(→ `box_plane_node`/`grasp_pose_node`/RViz가 goal당 1회 갱신).

```bash
# 최신 프레임 1장으로 검출 1회 + 결과 받기
ros2 action send_goal /yolov8_node/detect yolov8_detection_msgs/action/DetectBox \
    "{publish_annotated: true}" --feedback

# 프롬프트/신뢰도 일시 오버라이드 (빈 값/<=0 이면 노드 기본값)
ros2 action send_goal /yolov8_node/detect yolov8_detection_msgs/action/DetectBox \
    "{prompts: 'cardboard box,carton', confidence: 0.15, publish_annotated: true}"
```

Result 필드: `success`, `message`, `num_detections`, `detections_json`(`~/detections`와 동일 payload).

## 4. 자주 쓰는 launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `rviz` | `false` | RViz2를 같이 띄움 |
| `show_robot` | `false` | URDF + body→camera 정적 TF |
| `fit_box_plane` | `false` | box_plane_node 실행 (aligned depth 자동 켜짐) |
| `model` | `yolov8n.pt` | Ultralytics 가중치 — World 모델은 `yolov8s-worldv2.pt`, `yolov8l-worldv2.pt`, `yolov8x-worldv2.pt` |
| `prompts` | `""` | World 모델용 콤마-구분 텍스트 (예: `"box,cardboard box,carton"`) |
| `confidence` | `0.35` | YOLO 신뢰도 임계값 (World 모델은 0.10 권장) |
| `device` | `cpu` | `cpu` 또는 `cuda:0` |
| `image_size` | `640` | 추론 해상도 |
| `box_keywords` | `box,cardboard box,carton,package` | box_plane_node가 박스로 취급할 클래스명 |
| `camera_link_frame` | `camera_link` | TF child 프레임 (realsense2_camera가 발행하는 카메라 본체 프레임) |
| `urdf_path` | `…/openarmx_robot.urdf` | robot_state_publisher가 읽을 URDF |

## 5. 토픽

| 방향 | 토픽 | 타입 | 비고 |
| --- | --- | --- | --- |
| sub | `/camera/camera/color/image_raw` | `sensor_msgs/Image` | YOLO 입력 |
| sub | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | uint16 mm, `use_depth:=true` 또는 `fit_box_plane:=true` 시 |
| sub | `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 인트린식 |
| sub | `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | RGBD 클라우드 (fit_box_plane 시) |
| pub | `/yolov8_node/image_annotated` | `sensor_msgs/Image` | bbox 그린 BGR |
| pub | `/yolov8_node/detections` | `std_msgs/String` | JSON; class/conf/xyxy/center_px, depth 모드면 `point_camera` |
| pub | `/box_plane/cloud` | `sensor_msgs/PointCloud2` | 박스 윗면 인라이어 (빨강) |
| pub | `/box_plane/markers` | `visualization_msgs/MarkerArray` | 박스 윗면 disk + 법선 화살표, 바닥 disk + 화살표 |
| pub | `/box_plane/info` | `std_msgs/String` | JSON: planes[]/ground/box_height_m |
| pub | `/ground_plane/cloud` | `sensor_msgs/PointCloud2` | 바닥 인라이어 (초록) |

## 6. 박스 윗면 + 바닥면 평면 핏 알고리즘

핵심 아이디어 — **두 평면 모두 `openarmx_body_link0` 프레임의 +z 방향**으로 같은 법선을 가집니다 (박스가 바닥에 놓여 있고 윗면이 수평).

1. `tf2` lookup으로 정적 TF `openarmx_body_link0 → camera_color_optical_frame`을 한 번 가져와서 캐시
   - `body_up_in_cam`: body +z를 카메라 광학 프레임에 표현한 단위 벡터
   - `R_body_cam`, `t_body_cam`: 카메라 점 → body 점 변환
2. **박스 윗면**: YOLO bbox로 잘라낸 depth ROI 픽셀을 카메라 프레임으로 백프로젝션 → body 프레임으로 변환 → **body z 값 상위 35%**(상층 슬랩)만 남김 → RANSAC + SVD 평면. 법선 부호는 `body_up_in_cam`과 같은 반구로 통일.
3. **바닥**: bbox 바깥 픽셀들을 6 픽셀 간격으로 서브샘플 → 카메라 프레임 백프로젝션 → 박스 평면과 평행(법선 일치, 20° 이내)하면서 body z가 박스 윗면보다 2cm 이상 낮은 후보를 peel-off RANSAC으로 최대 6회 시도 → 인라이어가 많은 후보 선택.
4. 박스 높이 = `box_top.centroid_body.z - ground.centroid_body.z`.

이 두 단계를 `tf2_ros`/numpy만으로 처리하므로 Open3D 의존성이 없습니다.

## 7. 검증된 결과 (2026-05-27 캘리브 박스 장면)

```
BOX TOP (body frame):
  centroid = (+0.625, -0.001, +0.198) m
  normal   = (-0.02, -0.00, +1.00)   ← pure +z
  inliers  = 3989
GROUND  (body frame):
  centroid = (+0.830, +0.132, -0.002) m  ← 바닥 (z ≈ 0)
  normal   = (-0.00, -0.01, +1.00)   ← pure +z
  inliers  = 1146
BOX HEIGHT: 20.1 cm
```

## 8. 빠른 시각화

```bash
# rqt로 컬러+검출 이미지만
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /yolov8_node/image_annotated

# 한 프레임을 2D 이미지 + 3D 평면 패치로 합성
/home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/yolov8_env/bin/python \
    /tmp/yolo_plane_visualize.py /tmp/out.png
```

## 9. 종료

```bash
ps -ef | awk '/yolov8_detection|realsense2_camera_node|rviz2|robot_state_publisher|joint_state_publisher|static_transform_publisher|ros2 launch yolov8|ros2 run yolov8|box_plane_node|yolov8_node/ && !/awk/ {print $2}' \
  | xargs -r kill -TERM
sleep 2
ros2 daemon stop   # 토픽 캐시까지 비울 때
```
