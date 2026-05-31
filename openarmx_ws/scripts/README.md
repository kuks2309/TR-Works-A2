# kill_all_ros2.sh — ROS2 노드/프로세스 일괄 종료 (canonical)

실행 중인 모든 ROS2 노드와 관련 프로세스를 안전하게 종료하는 통합 스크립트.
`kuks2309` 계정의 여러 ROS2 워크스페이스에 흩어져 있던 변형들의 장점을 합친 **정본(canonical)** 버전이다.

## 사용법

```bash
# 직접 실행
./kill_all_ros2.sh

# bash alias (개발 PC)
kill_all_node          # → ~/.local/bin/kill_ros2_nodes.sh (동일 내용)

# scenario_ui "STOP ALL" 버튼
#   main_window.py 의 KILL_ALL_SCRIPT = ~/kill_all_ros2.sh 를 호출
#   설치: cp openarmx_ws/scripts/kill_all_ros2.sh ~/kill_all_ros2.sh && chmod +x ~/kill_all_ros2.sh
```

## 동작 방식

| 단계 | 내용 |
|------|------|
| **Phase A** | `ros2 node list` → 각 노드를 `__node:=<name>\b` 패턴으로 PID 역매핑 (namespace `/rtabmap/rtabmap` 처리, 단어경계 앵커로 `foo`/`foo_adapter` 오매칭 방지) → **SIGINT(graceful) 후 SIGKILL** |
| **Phase B** | 광역 정리 — `rviz2`/`rqt`/`rosbag2`, `/opt/ros/` 바이너리, Gazebo/Ignition(`gzserver`/`gzclient`/`ign gazebo`/`gz sim`) |
| **Phase C** | **FastDDS 공유메모리 `/dev/shm/fastrtps_*` 정리** + `ros2 daemon stop` |
| **검증/retry** | `ros2 node list` 재확인 → 남아있으면 1회 재시도 |

### 핵심 설계 포인트

- **self + 조상 PID 제외**: `$$`와 부모 프로세스 체인을 `SAFE_PIDS`로 보호한다.
  과거 버그 — 파일명에 `ros2`가 들어가 `pkill -f "ros2"`가 **스크립트 자기 자신을 SIGKILL**하던 문제를 근본 해결.
- **node-list 기반**: `/opt/ros/` 패턴만으로는 colcon 워크스페이스(`~/.../install/...`)에서 띄운 노드를 못 잡는다.
  `ros2 node list` + `__node:=` 매핑으로 설치 경로와 무관하게 모두 잡는다.
- **FastDDS ghost 노드 해결**: SIGKILL된 노드가 `/dev/shm/fastrtps_*` 세그먼트를 남기면
  죽은 노드가 `ros2 node list`에 계속 살아있는 것처럼 보인다 → 자기 세그먼트를 unlink 해 제거.
- **graceful 우선**: SIGINT를 먼저 보내 rclcpp/하드웨어 드라이버가 안전하게 종료하도록 한 뒤 SIGKILL fallback.
- **내부 노드**: `transform_listener_impl_*` 등은 독립 프로세스가 아니라 호스트 프로세스 내부 노드라
  별도 PID가 없고, 호스트를 종료하면 함께 사라진다(스킵 처리).

## 정본 배포 현황 (2026-05-31)

동일 내용이 다음 5개 repo + 로컬에 배포됨:

| repo | 경로 |
|------|------|
| `TR-Works-A2` | `openarmx_ws/scripts/kill_all_ros2.sh` |
| `T-AMR_ros2_ws` | `kill_all_ros2.sh` |
| `T-Robot_nav_ros2_ws` | `kill_all_ros2.sh` |
| `TM_Robot_ros2_ws` | `kill_all_ros2.sh` |
| `ros2_3dslam_ws` | `scripts/kill_all_ros2.sh` |
| 로컬 | `~/.local/bin/kill_ros2_nodes.sh`, `~/.local/bin/kill_all_ros2.sh` |

> 스크립트를 수정할 때는 위 사본들도 함께 동기화할 것.
