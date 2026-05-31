# openarmx_motion

OpenArmX 양팔 모션 인프라 패키지 — `cyclo_control` 의 2차 계획법 + 제어 장벽 함수 (MoveL / VR) 솔버를 OpenArmX 환경에 적응시키는 어댑터 / launch / URDF 만 담는다. 응용 (잡기, 텔레오퍼레이션 등) 은 별도 패키지에서 이 패키지를 의존한다 — 반대로는 안 함.

## 위치

```
src/openarmx_motion/
├── urdf/                            # 솔버용 reduced URDF
│   └── openarmx_bimanual_solver.urdf  # 양팔 14자유도 가동 (예정)
├── scripts/                         # URDF 생성기·단위 검증
│   └── gen_bimanual_urdf.py         # 예정
├── launch/
│   ├── openarmx_vr_bimanual.launch.py     # 양팔 통합 QP (cyclo vr_controller_node) — 예정
│   └── openarmx_drag_follow.launch.py     # 마커 × 2 + vr_controller + 어댑터 — 예정
├── openarmx_motion/                 # Python 모듈
│   └── marker_to_posestamped.py     # cyclo MoveL → PoseStamped 어댑터 — 예정
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 의존성

- `cyclo_motion_controller_ros` — cyclo 솔버 실행 노드 (`vr_controller_node`, `omx_movel_controller_node` 등)
- `robotis_interfaces` — `MoveL.msg`
- 표준 ROS 2: `rclpy`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`, `tf2_ros`, `visualization_msgs`

`cyclo_motion_controller_ros` 와 `robotis_interfaces` 는 `cyclo_ws` 워크스페이스에서 빌드되어 있어야 한다 (`MAKEFLAGS=-j1` 필수 — Pinocchio 컴파일 메모리 과다 회피).

## 빌드

```bash
cd ~/TR-Works/kkw/China/openarmx_ws
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/cyclo_ws/install/setup.bash
colcon build --packages-select openarmx_motion
```

## 설계 원칙

1. **cyclo 본체는 수정하지 않음** — 어댑터·launch·URDF 만 추가
2. **응용 무관**: 잡기, 텔레오퍼레이션, 키네틱 교시 등 어느 응용이 와도 같은 인프라 위에서 동작
3. **양팔 통합 2차 계획법 우선**: 단순 두 독립 솔버가 아니라 양팔 14자유도 단일 QP — null-space 협조와 자기충돌 회피 활용 가능
4. **상태 분리**: 인프라 변경 시 응용 패키지 빌드 영향 최소화
