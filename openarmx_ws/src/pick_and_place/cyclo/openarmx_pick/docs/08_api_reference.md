# openarmx_pick — API 레퍼런스 (토픽 / 프레임 / 파라미터)

분석일: 2026-06-03  
패키지: openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지가 외부와 주고받는 모든 ROS2 토픽, 사용하는 TF (Transform Framework) 프레임, 그리고 `grasp_pose_node`가 선언하는 전체 파라미터의 완전한 레퍼런스를 제공한다. 소스 코드 실제 기본값과 README 표를 교차 검증하여 불일치 사항을 명시한다. 아키텍처 설계·그래스프 합성 이론·솔버 URDF·launch 구성·검증 절차 등은 각각의 전담 문서(01, 03, 04, 05, 06)를 참조하라.

---

## 1. 노드 목록

| 노드 이름 | 실행 파일 | 패키지 | 소스 |
|---|---|---|---|
| `grasp_pose_node` | `grasp_pose_node` | `openarmx_pick` | `openarmx_pick/grasp_pose_node.py` |
| `openarmx_left_movel_controller` | `omx_movel_controller_node` | `cyclo_motion_controller_ros` | 외부 패키지 (cyclo_ws) |
| `openarmx_right_movel_controller` | `omx_movel_controller_node` | `cyclo_motion_controller_ros` | 외부 패키지 (cyclo_ws) |

본 문서의 파라미터 레퍼런스는 `grasp_pose_node`에만 집중한다. `omx_movel_controller_node`의 파라미터(QP (Quadratic Programming, 이차계획법) 가중치, CBF (Control Barrier Function, 제어 장벽 함수) 계수 등)는 `launch/openarmx_movel.launch.py` 및 `launch/openarmx_movel_bimanual.launch.py`에 하드코딩된 값으로 확인할 수 있으며, 해당 값은 5절에 별도 정리한다.

---

## 2. 토픽 레퍼런스

### 2.1 `grasp_pose_node` 입출력 토픽

아래 표는 코드 실제 값(`openarmx_pick/grasp_pose_node.py`)과 README 표를 교차 검증한 결과다.

| 토픽 | 방향 | 메시지 타입 | QoS | 프레임 | 조건 | 소스 근거 |
|---|---|---|---|---|---|---|
| `/box_plane/cloud` (파라미터 `cloud_topic`으로 변경 가능) | 구독 (in) | `sensor_msgs/PointCloud2` | depth=5, RELIABLE, VOLATILE | `camera_color_optical_frame` (업스트림 박스 평면 노드 발행) | 항상 활성 | `grasp_pose_node.py:153` |
| `/box_plane/info` (파라미터 `info_topic`으로 변경 가능) | 구독 (in) | `std_msgs/String` | depth=10, RELIABLE, VOLATILE | — (JSON 페이로드, 프레임 없음) | 항상 활성 | `grasp_pose_node.py:152` |
| `/openarmx/grasp_pose` (파라미터 `grasp_pose_topic`으로 변경 가능) | 발행 (out) | `geometry_msgs/PoseStamped` | depth=1, RELIABLE, TRANSIENT_LOCAL (latch) | `openarmx_body_link0` (파라미터 `base_frame`) | 항상 발행 (포인트 수 ≥ 30) | `grasp_pose_node.py:144` |
| `/openarmx/grasp_markers` | 발행 (out) | `visualization_msgs/MarkerArray` | depth=1, RELIABLE, TRANSIENT_LOCAL (latch) | `openarmx_body_link0` | 항상 발행 (포인트 수 ≥ 30) | `grasp_pose_node.py:145` |
| `/openarmx/left/movel` (파라미터 `movel_topic`으로 변경 가능) | 발행 (out) | `openarmx_scenario_player_msgs/MoveL` | depth=10, RELIABLE, VOLATILE | `openarmx_body_link0` | `auto_send:=true` **이고** `openarmx_scenario_player_msgs` 설치된 경우에만 | `grasp_pose_node.py:147-150` |

#### 토픽 QoS 상세

- **입력 토픽** (`/box_plane/cloud`, `/box_plane/info`): `create_subscription` 호출 시 정수 depth만 지정하므로 ROS2 기본 QoS (RELIABLE + VOLATILE) 가 적용된다. `grasp_pose_node.py:152-153` 참조.
- **출력 토픽** (`/openarmx/grasp_pose`, `/openarmx/grasp_markers`): `TRANSIENT_LOCAL` (latch) 설정이 명시적으로 적용된다. 구독자가 늦게 연결되어도 최근 값을 수신한다. `grasp_pose_node.py:141-144` 참조.
- **MoveL 출력**: 일반 depth=10 RELIABLE QoS. `grasp_pose_node.py:148` 참조.

### 2.2 `/box_plane/info` 페이로드 구조

`_on_info` 콜백은 JSON을 파싱하여 `box_height_m` 키만 추출한다. 현재 노드 내부에서 참고 정보(로그 출력)로만 사용하며 그래스프 자세 계산에는 직접 영향을 주지 않는다.

```python
# grasp_pose_node.py:158-162
def _on_info(self, msg: String):
    try:
        self._box_height = json.loads(msg.data).get("box_height_m")
    except Exception:
        pass
```

예시 페이로드: `{"box_height_m": 0.172}`

### 2.3 `/openarmx/grasp_markers` 마커 구조

MarkerArray에는 마커 1개(`ns="grasp"`, `id=0`)가 포함된다. 타입은 `ARROW` (pre-grasp 위치 → grasp 위치를 잇는 화살표). 크기: `scale.x=0.01 m` (샤프트 직경), `scale.y=0.02 m` (헤드 직경). 색상: RGBA `(0.1, 1.0, 0.2, 0.9)` (녹색 계열). `grasp_pose_node.py:251-261` 참조.

### 2.4 `omx_movel_controller_node` 솔버 토픽

`launch/openarmx_movel.launch.py` (단팔) 및 `launch/openarmx_movel_bimanual.launch.py` (양팔)에서 실제 적용되는 토픽이다. 이 노드는 외부 패키지(`cyclo_motion_controller_ros`)이며, 아래는 launch 파일에서 확인된 값이다.

#### 단팔 구성 (`openarmx_movel.launch.py`)

| 토픽 | 방향 | 메시지 타입 | 기본값 | 소스 |
|---|---|---|---|---|
| `<joint_states_topic>` | 구독 (in) | `sensor_msgs/JointState` | `/joint_states` | `openarmx_movel.launch.py:43` |
| `<movel_topic>` | 구독 (in) | `openarmx_scenario_player_msgs/MoveL` | `/openarmx/left/movel` | `openarmx_movel.launch.py:48` |
| `<joint_command_topic>` | 발행 (out) | `trajectory_msgs/JointTrajectory` | `/openarmx/left_arm/joint_trajectory` | `openarmx_movel.launch.py:44-47` |
| `<ee_pose_topic>` | 발행 (out) | `geometry_msgs/PoseStamped` | `/openarmx/left_ee_pose` | `openarmx_movel.launch.py:49` |

#### 양팔 구성 (`openarmx_movel_bimanual.launch.py`)

| 팔 | MoveL 구독 토픽 | 관절 명령 발행 토픽 | EE 자세 발행 토픽 |
|---|---|---|---|
| left | `/openarmx/left/movel` | `/left_joint_trajectory_controller/joint_trajectory` | `/openarmx/left/ee_pose` |
| right | `/openarmx/right/movel` | `/right_joint_trajectory_controller/joint_trajectory` | `/openarmx/right/ee_pose` |

> **주의**: 단팔 launch(`openarmx_movel.launch.py`)의 `joint_command_topic` 기본값은 `/openarmx/left_arm/joint_trajectory`이나, 양팔 launch(`openarmx_movel_bimanual.launch.py`)의 왼팔은 `/left_joint_trajectory_controller/joint_trajectory`로 다르다. 양팔 launch 주석(`openarmx_movel_bimanual.launch.py:42-44`)에 따르면 이전 경로(`/openarmx/left_arm/joint_trajectory`)는 구독자가 없어 trajectory가 드롭되었기 때문에 ros2_control 표준 JTC (Joint Trajectory Controller) 네임스페이스로 수정된 것이다.

---

## 3. TF (Transform Framework) 프레임 레퍼런스

| 프레임 | 역할 | 사용 위치 |
|---|---|---|
| `openarmx_body_link0` | 로봇 기저 프레임 (파라미터 `base_frame`). 솔버 URDF의 루트이자 그래스프 자세 발행 프레임. | `grasp_pose_node.py:106` (파라미터 기본값), `grasp_pose_node.py:171` (TF lookup target), `grasp_pose_node.py:219`, `grasp_pose_node.py:242` |
| `camera_color_optical_frame` | D435 카메라의 컬러 광학 프레임. `/box_plane/cloud`의 `header.frame_id`. 업스트림에서 발행하며 `grasp_pose_node`는 이 값을 cloud 헤더에서 읽어 TF lookup의 source frame으로 사용. | `grasp_pose_node.py:172` (cloud.header.frame_id 동적 참조) |
| `openarmx_left_hand_tcp` | 왼팔 TCP (Tool Center Point, 공구 중심점) 링크. 솔버의 controlled_link이자 그래스프 approach/opening 축의 기준 툴 프레임. | `openarmx_movel.launch.py:42`, `openarmx_movel_bimanual.launch.py:37` |
| `openarmx_right_hand_tcp` | 오른팔 TCP 링크. 양팔 구성 시 오른팔 솔버의 controlled_link. | `openarmx_movel_bimanual.launch.py:37` |

### TF 조회 동작

`grasp_pose_node`는 매 `/box_plane/cloud` 콜백마다 `tf_buffer.lookup_transform(base_frame, cloud.header.frame_id, rclpy.time.Time())`을 호출하여 **최신(latest) TF**를 사용한다. 타임스탬프 기반 동기화가 아니므로 카메라 TF가 정적인 경우(캘리브레이션 결과 적용 후) 안정적으로 동작한다. `grasp_pose_node.py:171-174` 참조.

---

## 4. `grasp_pose_node` 파라미터 레퍼런스

모든 파라미터는 `GraspPoseNode.__init__`에서 `declare_parameter`로 선언된다. `grasp_pose_node.py:104-121` 참조.

### 4.1 토픽 라우팅 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `cloud_topic` | `string` | `"/box_plane/cloud"` | — | 박스 상면 인라이어 포인트클라우드 구독 토픽. |
| `info_topic` | `string` | `"/box_plane/info"` | — | 박스 메타데이터(JSON) 구독 토픽. |
| `grasp_pose_topic` | `string` | `"/openarmx/grasp_pose"` | — | 그래스프 자세(`PoseStamped`) 발행 토픽. |
| `movel_topic` | `string` | `"/openarmx/left/movel"` | — | MoveL 명령 발행 토픽. `auto_send:=true` 시에만 발행자가 생성된다. |

### 4.2 좌표 및 프레임 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `base_frame` | `string` | `"openarmx_body_link0"` | — | TF lookup의 목적 프레임이자 모든 출력 토픽의 `header.frame_id`. |

### 4.3 그래스프 기하 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `pregrasp_height` | `double` | `0.10` | m | 박스 상면 중심에서 **위** 방향(+z) 오프셋. Pre-grasp hover 위치 높이. MoveL 명령 목표점으로 사용됨. |
| `grasp_depth` | `double` | `0.005` | m | 박스 상면 중심에서 **아래** 방향(-z) 오프셋. 실제 그래스프 자세의 z좌표를 소폭 낮춰 접촉을 보장. |

그래스프 자세 위치 계산 (`grasp_pose_node.py:193-194`):

```python
grasp_xyz = centroid.copy(); grasp_xyz[2] -= self.grasp_depth   # 상면에서 5 mm 아래
pre_xyz   = centroid.copy(); pre_xyz[2]  += self.pregrasp_h    # 상면에서 100 mm 위
```

### 4.4 연산 효율 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `cloud_stride` | `int` | `4` | — | PCA (Principal Component Analysis, 주성분 분석) 속도 향상을 위한 포인트클라우드 서브샘플링 간격. 4이면 4포인트마다 1개를 사용. `_read_xyz` 헬퍼에 전달됨. |

### 4.5 MoveL 자동 전송 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `auto_send` | `bool` | `False` | — | `True`이면 Pre-grasp MoveL을 자동 발행. `False`(기본)이면 그래스프 자세만 발행하고 모션 명령은 없음. |
| `move_time` | `double` | `4.0` | s | MoveL 메시지의 `time_from_start` 값. QP 솔버가 목표에 도달할 때까지의 계획 시간. |
| `send_min_interval` | `double` | `5.0` | s | 디바운스(Debounce) 쿨다운: 이전 MoveL 전송으로부터 이 시간이 경과해야 재전송 허용. 카메라 프레임마다 MoveL이 재전송되어 솔버 궤적이 재시작되는 문제(팔이 기어가듯 느려짐)를 방지. `move_time` 이상으로 설정하는 것이 권장됨. |
| `send_min_delta` | `double` | `0.02` | m | 디바운스 위치 임계값: pre-grasp 목표 위치가 이전 전송 위치에서 이 거리 이상 이동했을 때 쿨다운과 무관하게 재전송. |

디바운스 로직 (`grasp_pose_node.py:226-238`):

```python
# 두 조건 중 하나라도 충족하면 재전송
if moved > self.send_min_delta or elapsed > self.send_min_interval:
    return True
return False
```

### 4.6 툴 프레임 축 파라미터

| 파라미터 이름 | 타입 | 기본값 (코드) | 단위 | 의미 |
|---|---|---|---|---|
| `tool_approach_axis` | `double[]` | `[0.0, 0.0, 1.0]` | — | TCP 로컬 프레임에서 그래스프 접근(진입) 방향 벡터. 기본값은 TCP의 로컬 +z축. |
| `tool_opening_axis` | `double[]` | `[1.0, 0.0, 0.0]` | — | TCP 로컬 프레임에서 그리퍼 손가락이 벌어지는 방향 벡터. 기본값은 TCP의 로컬 +x축. |

이 두 파라미터는 `_grasp_rotation` 함수(`grasp_pose_node.py:83-97`)에 전달되어 **기저 프레임 기준 접근/개구 방향**과 **툴 프레임 기준 접근/개구 축**을 매핑하는 회전행렬 R_base_tool을 계산한다. 기저 프레임에서의 접근 방향은 항상 `[0, 0, -1]` (수직 하강), 개구 방향은 박스 장축의 법선 벡터(단축 방향)로 고정된다. `grasp_pose_node.py:189-190`, `grasp_pose_node.py:186` 참조.

---

## 5. README 표와 코드 기본값 교차 검증

README의 `## Topics & frames` 섹션 표와 코드 `declare_parameter` 값을 항목별로 비교한다.

### 5.1 토픽 표 — 일치 여부

| 항목 | README 기재 | 코드 실제 값 | 판정 |
|---|---|---|---|
| `/box_plane/cloud` 타입 | `sensor_msgs/PointCloud2` | `PointCloud2` (`grasp_pose_node.py:32, 153`) | **일치** |
| `/box_plane/cloud` 방향 | in | 구독 | **일치** |
| `/box_plane/info` 타입 | `std_msgs/String` | `String` (`grasp_pose_node.py:33, 152`) | **일치** |
| `/openarmx/grasp_pose` 타입 | `geometry_msgs/PoseStamped` | `PoseStamped` (`grasp_pose_node.py:29, 144`) | **일치** |
| `/openarmx/grasp_markers` 타입 | `visualization_msgs/MarkerArray` | `MarkerArray` (`grasp_pose_node.py:34, 145`) | **일치** |
| `/openarmx/left/movel` 타입 | `openarmx_scenario_player_msgs/MoveL` | `MoveL` (`grasp_pose_node.py:39, 148`) | **일치** |
| `/openarmx/left/movel` 조건 | `only when auto_send:=true` | `auto_send and _HAVE_MOVEL` (`grasp_pose_node.py:147`) | **일치** |
| `/box_plane/cloud` 프레임 | `camera_color_optical_frame` | 코드에서 `cloud.header.frame_id`를 동적으로 읽음 (`grasp_pose_node.py:172`). 고정 문자열이 아니라 업스트림 발행자 값을 그대로 사용. | **실질 일치** (README가 업스트림 관례를 기재한 것이며 코드는 동적 처리) |

### 5.2 파라미터 — README vs. 코드 기본값

README는 파라미터 표 대신 주요 파라미터를 산문으로 열거한다. 기재된 값과 코드 기본값의 비교:

| 파라미터 | README 언급 | 코드 기본값 | 판정 |
|---|---|---|---|
| `base_frame` | `openarmx_body_link0` | `"openarmx_body_link0"` (`grasp_pose_node.py:106`) | **일치** |
| `pregrasp_height` | `0.10 m` | `0.10` (`grasp_pose_node.py:109`) | **일치** |
| `grasp_depth` | `0.005 m` | `0.005` (`grasp_pose_node.py:110`) | **일치** |
| `auto_send` | `false` | `False` (`grasp_pose_node.py:112`) | **일치** |
| `send_min_delta` | 언급됨 (값 미기재) | `0.02` (`grasp_pose_node.py:119`) | README에 기본값 **미기재** (문서 보완 필요) |
| `send_min_interval` | 언급됨 (값 미기재) | `5.0` (`grasp_pose_node.py:118`) | README에 기본값 **미기재** (문서 보완 필요) |
| `tool_approach_axis` | 언급됨 (값 미기재) | `[0.0, 0.0, 1.0]` (`grasp_pose_node.py:120`) | README에 기본값 **미기재** |
| `tool_opening_axis` | 언급됨 (값 미기재) | `[1.0, 0.0, 0.0]` (`grasp_pose_node.py:121`) | README에 기본값 **미기재** |
| `cloud_stride` | 미언급 | `4` (`grasp_pose_node.py:111`) | README에 **미언급** |
| `move_time` | 미언급 | `4.0` (`grasp_pose_node.py:113`) | README에 **미언급** |
| `cloud_topic` | 미언급 | `"/box_plane/cloud"` (`grasp_pose_node.py:104`) | README에 **미언급** (토픽 표에는 있음) |
| `info_topic` | 미언급 | `"/box_plane/info"` (`grasp_pose_node.py:105`) | README에 **미언급** (토픽 표에는 있음) |
| `grasp_pose_topic` | 미언급 | `"/openarmx/grasp_pose"` (`grasp_pose_node.py:108`) | README에 **미언급** |
| `movel_topic` | 미언급 | `"/openarmx/left/movel"` (`grasp_pose_node.py:107`) | README에 **미언급** (토픽 표에는 있음) |

> **결론**: README와 코드 간 **수치 불일치는 없다**. 다만 README는 일부 파라미터의 기본값을 생략하거나 파라미터 자체를 미언급하고 있다. 본 문서(4절)가 완전한 파라미터 레퍼런스를 제공한다.

---

## 6. `omx_movel_controller_node` launch 파라미터 (참조용)

`grasp_pose_node`의 MoveL 출력을 소비하는 솔버 노드에 적용되는 주요 파라미터다. 단팔/양팔 launch 파일에서 확인된 값이며, 솔버 내부 파라미터이므로 이 섹션은 참조용이다.

### 6.1 단팔 구성 (`openarmx_movel.launch.py`)

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `urdf_path` | `urdf/openarmx_left_solver.urdf` | 단팔(좌) 솔버 URDF 경로 |
| `srdf_path` | `""` | Stage-1: SRDF (Semantic Robot Description Format) 비어있음, 충돌 쌍 없음 |
| `base_frame` | `openarmx_body_link0` | URDF 루트 프레임 |
| `controlled_link` | `openarmx_left_hand_tcp` | 제어 대상 EE (End-Effector, 말단 작동기) 링크 |
| `joint_states_topic` | `/joint_states` | 관절 피드백 구독 토픽 |
| `joint_command_topic` | `/openarmx/left_arm/joint_trajectory` | 관절 명령 발행 토픽 |
| `movel_topic` | `/openarmx/left/movel` | MoveL 목표 구독 (remap `~/movel`) |
| `ee_pose_topic` | `/openarmx/left_ee_pose` | 현재 EE 자세 발행 (remap `~/current_pose`) |
| `control_frequency` | `100.0` | Hz, QP 제어 루프 주파수 |
| `time_step` | `0.01` | s, QP 이산화 시간 간격 |
| `trajectory_time` | `0.05` | s |
| `kp_position` | `4.0` | 위치 비례 게인 |
| `kp_orientation` | `2.5` | 자세 비례 게인 |
| `weight_task_position` | `10.0` | QP 위치 태스크 가중치 |
| `weight_task_orientation` | `1.0` | QP 자세 태스크 가중치 |
| `weight_damping` | `0.05` | QP 댐핑 가중치 |
| `slack_penalty` | `1000.0` | CBF 슬랙 변수 페널티 |
| `cbf_alpha` | `5.0` | CBF α 계수 |
| `joint_state_timeout` | `0.5` | s, 관절 상태 수신 타임아웃 |

### 6.2 양팔 구성 (`openarmx_movel_bimanual.launch.py`) 에서의 차이점

| 파라미터 | 단팔 값 | 양팔 값 | 비고 |
|---|---|---|---|
| `joint_command_topic` | `/openarmx/left_arm/joint_trajectory` | `/left_joint_trajectory_controller/joint_trajectory` | 양팔 launch는 ros2_control JTC 표준 경로 사용 (`openarmx_movel_bimanual.launch.py:44`) |
| `trajectory_time` | `0.05` | `0.0` | 양팔 launch는 upstream cyclo 기본값 복원 |
| `kp_position` | `4.0` | `50.0` | 양팔 launch는 upstream cyclo 기본값 복원 |
| `kp_orientation` | `2.5` | `50.0` | 양팔 launch는 upstream cyclo 기본값 복원 |
| `weight_damping` | `0.05` | `0.001` | 양팔 launch는 upstream cyclo 기본값 복원 |
| `collision_buffer` | 미설정 | `0.01` | 양팔 launch에만 추가 |
| `collision_safe_distance` | 미설정 | `0.005` | 양팔 launch에만 추가 |

---

## 7. `openarmx_pick.launch.py` 런치 인수

`openarmx_pick.launch.py`는 `grasp_pose_node` + 솔버를 함께 기동하는 진입점이다. 외부에서 오버라이드 가능한 인수:

| 인수 이름 | 기본값 | 의미 |
|---|---|---|
| `auto_send` | `"false"` | `grasp_pose_node`의 `auto_send` 파라미터로 전달 |
| `pregrasp_height` | `"0.10"` | `grasp_pose_node`의 `pregrasp_height` 파라미터로 전달 |
| `cloud_topic` | `"/box_plane/cloud"` | `grasp_pose_node`의 `cloud_topic` 파라미터로 전달 |
| `start_solver` | `"true"` | `false`이면 `openarmx_movel.launch.py` 인클루드를 건너뜀. 그래스프 합성만 실행할 때 사용. |

`openarmx_pick.launch.py:26-33` 참조. `launch.py`에서 직접 하드코딩된 `grasp_pose_node` 파라미터(`info_topic`, `base_frame`, `movel_topic`, `grasp_depth`, `cloud_stride`)는 런치 인수로 노출되지 않아 오버라이드하려면 `--ros-args -p` 방식으로 직접 전달해야 한다.
