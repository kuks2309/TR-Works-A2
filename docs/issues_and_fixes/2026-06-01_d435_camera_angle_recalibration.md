# D435 카메라 마운트 각도 변경 → 재캘리브레이션 (30° → 45° → 60°)

## 2026-06-01 22:48 (KST) — 카메라 틸트 변경 시 extrinsic 재캘리브 워크플로

### 배경 / 문제

D435 중앙 카메라의 물리 **마운트 틸트 각도를 30° → 45° → 60°** 로 변경.
"각도만 바뀌었으니 URDF 의 pitch 숫자만 31 → 45 → 60 으로 패치하면 되지 않나"
라는 접근은 **틀렸다.**

- 베이크된 카메라 pose 는 캘리브레이션으로 측정한 **6-DOF extrinsic**
  (`openarmx_body_link0 → d435_center_link`, [v10.urdf.xacro](../../openarmx_ws/src/openarmx_description/urdf/robot/v10.urdf.xacro) 끝, bimanual 가드 안).
- 마운트가 피벗을 중심으로 회전하면 **pitch 뿐 아니라 광학중심 위치(translation)도
  수 cm 이동**한다. pitch 만 고치면 translation 이 옛 각도 값으로 남아 TF 가
  물리 현실과 불일치 → 박스 검출 좌표·grasp pose 가 틀어진다.
- → 각도 변경 시 **반드시 ChArUco extrinsic 재캘리브**.

### 절차 (각 각도마다 반복)

1. 전체 kill (`openarmx_ws/scripts/kill_all_ros2.sh`) — 충돌 카메라 노드 종료, USB 해제.
2. `calibration/launch/calibration_bringup_with_camera_tf.launch.py enable_camera:=true enable_charuco:=true`
   (realsense `d435_center` + 로봇 RSP + ChArUco 라이브 검출 + RViz).
3. 보드를 카메라 FOV 에 두고 **검출 코너가 24/24 로 안정**될 때까지 위치 조정
   (틸트가 클수록 보드를 카메라 정면으로 옮겨야 함).
4. `calibration/scripts/solve_extrinsic.py --bx 0.35 --by 0.0 --bz 0.12 --roll 180 --yaw 0 --samples 30`
   → optical-frame TF 산출 (base_link → `d435_center_color_optical_frame`).
5. optical→link 합성: `d435_center_link ← optical` 정적 오프셋
   (≈ t(0,0.015,0), q≈(-0.5,0.5,-0.5,0.5)) 와 합성 → 최종 `body_link0 → d435_center_link`.
6. 6개 SSOT 파일에 반영 (아래 "수정").

### 결과 (보드 자세 0.35 / 0 / 0.12 m, roll 180 고정)

| 마운트 | translation (m) | rpy pitch (deg) | 위치 std |
|---|---|---|---|
| 30° (05-27) | (0.034018, 0.036608, 0.644715) | 31.0059 | <0.3 mm |
| 45° (06-01) | (0.072753, 0.011982, 0.639814) | 44.5846 | <0.1 mm |
| 60° (06-01) | (0.065430, 0.000987, 0.641921) | 59.7350 | <0.1 mm |

- **검증 지표 1**: link-frame pitch 가 마운트 각도로 수렴 (31 / 44.6 / 59.7 ≈ 30 / 45 / 60).
- **검증 지표 2**: z(카메라 높이) ≈ 0.64 m 유지, roll/yaw ≈ 0.
- **검증 지표 3 (라이브)**: `charuco_board` TF(부모 optical → 캘리브 체인 → body_link0)
  를 base_link 기준으로 lookup → 보드 실측 pose (0.35, 0, 0.12, roll180, pitch0, yaw0)
  재현. 60° 최종 확인: 오차 **|Δ|=2.1 mm, yaw -0.76°, pitch -0.06°(수평 평행 확인)**.
  (보드가 일시적으로 yaw 6.7° 틀어졌을 땐 오차 20.7mm → 정렬 복원 후 2.1mm.
   캘리브 자체엔 yaw 바이어스 없음을 입증.)

### 수정 (60° 기준, extrinsic 값 보유 6개 파일)

| 파일 | 변경 |
|---|---|
| `openarmx_description/urdf/robot/v10.urdf.xacro` | `d435_center_calib_joint` origin xyz/rpy + 주석 |
| `calibration/launch/calibration_bringup_with_camera_tf.launch.py` | docstring 주석 |
| `calibration/README.md` | §1 결과 + 각도 이력(30→45→60) |
| `calibration/rviz/calibration_camera.rviz` | PointCloud2 디스플레이 추가 (포인트클라우드 정합 확인용) |
| `3d_detect_ws/src/yolov8_detection/launch/yolov8_d435.launch.py` | `_CAM_TF` (박스검출 standalone TF) |
| 메모리 2개 (repo 외) | `d435-camera-extrinsic`, `charuco-board-setup` |

- `openarmx_description` 은 `--symlink-install` → xacro 편집 후 리빌드 불필요.

### 재발 방지

- 카메라 마운트 각도/자세가 바뀌면 **숫자 패치 금지 → ChArUco 재캘리브 필수**
  (틸트는 pitch + translation 동시 변경).
- extrinsic 값은 **6곳에 중복** 보유 → 갱신 시 전부 동기화
  (`grep -rn "<옛 pitch 값>"` 으로 잔존 확인).
- 캘리브 품질: solve_extrinsic 전 **검출 코너 24/24 안정** 확인. 보드는 **수평·base 정렬**
  (pitch 0, yaw 0 가정) — 기울이거나 돌리면 그 각도를 `--pitch/--roll/--yaw` 로 입력.
- 검증: link-frame pitch ≈ 마운트 각도 + 라이브 `charuco_board` TF 가 보드 실측 pose
  재현(수 mm) 으로 확인.

### 관련 파일

- 캘리브 SSOT: [calibration/README.md](../../calibration/README.md)
- solve 스크립트: `calibration/scripts/solve_extrinsic.py`
- 이전 사건: [2026-06-01_yolov8_on_demand_action_server.md](2026-06-01_yolov8_on_demand_action_server.md)

---
