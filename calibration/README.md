# D435 카메라 캘리브레이션 & 시각화

OpenArmX bimanual 로봇의 중앙 RealSense **D435** 카메라 외부 파라미터(extrinsic)
캘리브레이션과 RViz 시각화 절차를 정리한다.

## 1. 캘리브레이션 결과 (2026-05-27)

ChArUco 보드 기반 PnP(Perspective-n-Point, 원근 n점 자세추정) 평균으로 산출한
정적 변환:

```
openarmx_body_link0 → d435_center_link
  translation (m)  : x=0.034018, y=0.036608, z=0.644715
  rotation rpy(deg): roll=-1.4041, pitch=31.0059, yaw=-2.1785
```

- PnP 30프레임 평균, 위치 표준편차 < 0.3 mm.
- `openarmx_body_link0 == base_link` (둘 다 world 원점). 이 프레임은 **bimanual urdf
  에만 존재**한다 (단일팔 v10 은 `openarmx_link0` 루트).

## 2. 보드 셋업

| 항목 | 값 |
|---|---|
| 보드 | ChArUco 5×7, square 40 mm, marker 30 mm, `DICT_5X5_50` |
| 보드 원점 (base_link 기준) | (x, y, z) = (+0.59, 0.00, 0.00) m |
| 보드 자세 | 수평, printed side 아래 → 캘리브 입력 `--roll 180` |

보드 PDF/PNG: [`boards/charuco_5x7_40mm_30mm.pdf`](boards/charuco_5x7_40mm_30mm.pdf)
(생성 스크립트 [`scripts/generate_charuco.py`](scripts/generate_charuco.py)).

## 3. 캘리브레이션 절차

```bash
cd ~/TR-Works/kkw/China/openarmx_ws && source install/setup.bash

# (1) 카메라 + 로봇 + ChArUco 라이브 검출 bringup (보드가 화면에 잘 잡히는지 확인)
ros2 launch <repo>/calibration/launch/calibration_bringup.launch.py bimanual:=true

# (2) extrinsic 산출 — 보드를 (+0.59, 0, 0)m 에 수평 배치한 상태에서
python3 <repo>/calibration/scripts/solve_extrinsic.py \
    --bx 0.59 --by 0.0 --bz 0.0 --roll 180 --yaw 0 \
    --samples 30 --parent base_link --child d435_center_link
```

- `solve_extrinsic.py` 출력은 **광학 프레임**(`d435_center_color_optical_frame`) 기준이다.
  URDF 카메라 링크(`d435_center_link`) 기준으로 바꾸려면
  `tf2_echo d435_center_link d435_center_color_optical_frame` 의 정적 오프셋
  (≈ translation (0, 0.015, 0), rpy (-90°, 0, -90°)) 으로 합성한다.
- 위 §1 값은 그 합성까지 끝난 link 기준 최종값이다.

## 4. 카메라를 로봇 모델에 포함 (시각화)

카메라 본체(`d435.dae` 메쉬)는 **로봇 description 에 직접 베이크**되어 있다 —
별도 `static_transform_publisher` 나 마커 노드가 필요 없다.

- 위치: [`openarmx_ws/src/openarmx_description/urdf/robot/v10.urdf.xacro`](../openarmx_ws/src/openarmx_description/urdf/robot/v10.urdf.xacro)
  의 끝, `<xacro:if value="$(arg bimanual)">` 가드 안.
  - fixed joint `openarmx_body_link0 → d435_center_link` (origin = §1 extrinsic)
  - `d435_center_link` 에 `package://realsense2_description/meshes/d435.dae`
    (**visual only, collision 없음** → 모션 플래닝/제어 영향 없음)
  - 메쉬 visual origin 은 realsense `_d435.urdf.xacro` 에서 그대로 복사.
- 효과: `bimanual:=true` 로 로봇을 로드하는 **모든** 런치(scenario_player, bringup,
  moveit demo, VR)에서 카메라가 자동 표시되고 `robot_state_publisher` 가 TF 도 발행.
- **bimanual 가드 필수**: 단일팔(`bimanual:=false`)은 `openarmx_body_link0` 이 없어
  카메라 블록을 스킵한다(에러 없음).

### 전용 시각화 런치

```bash
# 로봇 + 카메라 본체 + TF 좌표축 (하드웨어 불필요)
ros2 launch <repo>/calibration/launch/calibration_bringup_with_camera_tf.launch.py \
    enable_camera:=false enable_charuco:=false
```

- RViz 설정: [`rviz/calibration_camera.rviz`](rviz/calibration_camera.rviz)
  (RobotModel + TF, fixed frame `openarmx_body_link0`).
- 카메라 데이터(pointcloud/이미지)까지 보려면 `enable_camera:=true` (D435 연결 필요).

## 5. 빌드 노트

- `openarmx_description` 수정 후 반영: `colcon build --packages-select openarmx_description --symlink-install`
  (symlink-install 이므로 이후 xacro 편집은 리빌드 없이 반영).
- ⚠️ `build/` 가 다른 경로(`/home/openarmx/openarmx_ws`)에서 flatten 되어 와
  CMakeCache 가 stale 인 경우 `"source does not match"` 에러 발생 →
  `rm -rf build/<pkg>` 후 재빌드.

## 디렉터리

```
calibration/
├── boards/      ChArUco 보드 PDF/PNG
├── launch/      bringup, bringup_with_camera_tf
├── rviz/        calibration_camera.rviz
└── scripts/     generate_charuco, live_detect, solve_extrinsic[_square], test_detect
```
