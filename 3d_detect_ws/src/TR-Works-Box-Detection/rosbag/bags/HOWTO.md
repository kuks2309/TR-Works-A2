# ros2 bag 기록·재생 가이드 (TR-Works)

`box_pick_and_place` 시나리오 bag 기록 및 시각화 전체 절차.

---

## 1. 사전 준비

### 1.1 카메라 launch (D435 2대)

**터미널 A — d435_center (시리얼 818312070932)**
```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_818312070932 \
  camera_name:=d435_center camera_namespace:=d435_center \
  pointcloud.enable:=true align_depth.enable:=true \
  enable_color:=true enable_depth:=true
```

**터미널 B — d435_center_upper (시리얼 819612070814)**
```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_819612070814 \
  camera_name:=d435_center_upper camera_namespace:=d435_center_upper \
  pointcloud.enable:=true align_depth.enable:=true \
  enable_color:=true enable_depth:=true
```

### 1.2 로봇/시나리오 시스템 launch
평소 사용하는 launch 절차로 robot system + scenario_player 띄움.
(arm_controller, hand_node, motors_welcon_node, scenario_ui 등)

### 1.3 토픽 확인
```bash
ros2 topic list | grep d435  # 양쪽 카메라 토픽 모두 보여야 함
ros2 node list               # robot 시스템 노드들 확인
```

---

## 2. bag 기록

### 2.1 폴더 구조 + 명명 규칙

bag은 `bags/<날짜폴더>/<scenario>_<a|b>/` 경로로 저장.

```
bags/
├── HOWTO.md, README.md, qos_override.yaml   ← 루트
├── 20260504/   1-camera 시절 데이터 (구, 보존)
└── 20260506/   2-camera 표준 (현)
```

카메라 구성/시스템이 바뀌면 새 날짜 폴더 생성. 자세한 bag 목록은 [README.md](README.md) 참조.

| Suffix | 의미 |
|---|---|
| `_a` | 앉은 자세에서 박스 잡기 (시작부터 앉은 상태) |
| `_b` | 서있는 자세 → 앉기 → 박스 잡기 (자세 변환 포함) |

시나리오: `big_box_{a,b}`, `medium_box_wide_{a,b}`, `medium_box_narrow_{a,b}`, `short_box_wide_{a,b}`, `short_box_narrow_{a,b}`

### 2.2 기록 명령 (표준)

```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
cd /home/tc/Project/TR-Works-Dev/bags/<날짜폴더> && \
ros2 bag record -a \
  --qos-profile-overrides-path ../qos_override.yaml \
  --max-cache-size 1073741824 \
  -o <시나리오>_<a|b>
```

**예: `20260506/short_box_wide_a`**
```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE && cd /home/tc/Project/TR-Works-Dev/bags/20260506 && ros2 bag record -a --qos-profile-overrides-path ../qos_override.yaml --max-cache-size 1073741824 -o short_box_wide_a
```

### 2.3 옵션 의미

| 옵션 | 이유 |
|---|---|
| `source ... setup.bash` | `tr_works_hand_controller`, `tc_msgs` 같은 커스텀 메시지 타입 인식 |
| `unset FASTRTPS_DEFAULT_PROFILES_FILE` | FASTRTPS 프로파일 환경변수가 통신 깨뜨릴 수 있음 (ISSUE-030) |
| `cd .../bags` | 저장 위치 통일 |
| `-a` | 모든 토픽 자동 기록 |
| `--qos-profile-overrides-path qos_override.yaml` | `/joint_states` publisher 3개(joint_state_broadcaster, welcon_joint_state_publisher, hand_node) QoS 충돌 해결 |
| `--max-cache-size 1073741824` | 1GB 캐시 (카메라 2대 데이터 ~280MB/s 처리, 메시지 손실 방지) |
| `-o <name>` | 출력 폴더 이름 |

### 2.4 qos_override.yaml 내용

```yaml
/joint_states:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 100
```
→ `volatile`로 강제해서 모든 publisher의 메시지 받음 (TRANSIENT_LOCAL이면 일부 publisher 누락).

### 2.5 녹화 절차

1. 위 명령으로 `ros2 bag record` 시작
2. 로그 확인:
   - ✅ `[INFO] Subscribed to topic '/...'` 30+ 개
   - ✅ `[WARN] ... unknown type` **없음** (있으면 source 누락)
   - ✅ `[WARN] ... incompatible QoS` **없음** (있으면 QoS override 누락)
   - ✅ `[INFO] Overriding subscription profile for /joint_states` 보임
3. 다른 터미널에서 시나리오 실행
4. **시나리오 완전히 끝난 후** `Ctrl+C`로 bag 종료
5. 기록 종료 시 `Cache buffers lost messages` 워닝 확인 (전체 1% 미만이면 정상)

### 2.6 검증

```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash
ros2 bag info /home/tc/Project/TR-Works-Dev/bags/<날짜폴더>/<name>
```

#### 검증 원칙 (시나리오 무관)

> **시나리오마다 모션이 달라서 duration이 다르고, 그에 따라 메시지 카운트도 자연스럽게 달라진다.**
> 절대 카운트가 아니라 **rate (fps, Hz)** 와 **본질 조건**으로 판단해야 한다.

##### 본질 조건 (반드시 만족)

1. ✅ 시나리오 정상 완주 — `/scenario_player/status` 의 마지막 메시지가 종료 상태
2. ✅ 2-camera 모두 캡처 — `/d435_center/...` 와 `/d435_center_upper/...` 양쪽 토픽 존재
3. ✅ 커스텀 메시지 토픽 누락 없음 — `/hand/state`, `/hand/command`, `/motor_status` 모두 1+ count
4. ✅ `[WARN] unknown type` 또는 `incompatible QoS` 없이 종료
5. ✅ `Cache buffers lost messages` 워닝이 전체 1% 미만

##### rate 기반 정상 범위 (count / duration)

| 토픽 | 기대 rate | 비고 |
|---|---|---|
| `/joint_states` | 80~100 Hz | publisher 3개 통합, override 적용 |
| `/dynamic_joint_states` | 60~80 Hz | |
| `/d435_center/.../color/image_raw` | 20~30 fps | 카메라 1대 (30fps가 이상적) |
| `/d435_center_upper/.../color/image_raw` | 20~30 fps | 카메라 1대 |
| `/hand/state` | 8~12 Hz | |
| `/motor_status` | 8~12 Hz | |
| `/arm_L_controller/state`, `/arm_R_controller/state` | 양쪽 거의 동일 카운트 | 비대칭이면 한쪽 손실 |

##### 시나리오별 duration 참고치 (실측)

| 시나리오 | `_a` duration | `_b` duration |
|---|---|---|
| short_box_wide | ~6초 | ~10초 |
| short_box_narrow | ~9~10초 | ~12~13초 |
| big_box | ~7초 | (미실측) |
| medium_box_narrow | ~7~8초 | (미실측) |
| medium_box_wide | ~6초 | (미실측) |

##### 판단 절차

1. 본질 조건 5개 모두 ✅ 인지 먼저 확인
2. 본질 조건 통과 시, rate가 위 범위면 정상으로 간주
3. rate가 절반 이하로 떨어지거나 한쪽 카메라만 잡혔으면 재기록
4. 카운트 절대값으로 판단하지 말 것 — 시나리오 모션·duration에 따라 자연 변동

---

## 3. bag 재생 + 시각화

### 3.1 RViz로 로봇 + 카메라 보기 (3 터미널)

**터미널 1 — display launch (RViz + robot_state_publisher)**
```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
ros2 launch tr_works_bringup display.launch.py standalone:=true
```

**터미널 2 — bag 재생**
```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
ros2 bag play /home/tc/Project/TR-Works-Dev/bags/20260506/short_box_wide_a --clock -l
```

옵션:
- `--clock`: simulation time 발행 (다른 노드들이 bag 시간 따라가게)
- `-l`: 무한반복
- `-r 0.5` / `-r 2.0`: 속도 조절
- `--start-paused`: 일시정지로 시작 (스페이스로 재생)

**터미널 3 — use_sim_time 활성화 (RViz 뜬 다음 한 번)**
```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
ros2 param set /robot_state_publisher use_sim_time true && \
ros2 param set /rviz use_sim_time true
```

⚠️ `display.launch.py`는 `use_sim_time` 옵션 안 받음. 수동으로 param 설정 필수.
⚠️ param 변경 후 RViz의 TF buffer가 꼬일 수 있음 → RViz 재시작이 가장 깔끔:
```bash
ros2 run rviz2 rviz2 -d /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/tr_works_bringup/share/tr_works_bringup/config/rviz_default.rviz --ros-args -p use_sim_time:=true
```

### 3.2 rqt_bag로 빠르게 보기 (1 터미널)

```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
ros2 run rqt_bag rqt_bag /home/tc/Project/TR-Works-Dev/bags/20260506/short_box_wide_a
```

GUI에서 타임라인 스크러빙, 토픽별 메시지 inspect, 이미지/플롯 뷰 모두 가능. **분석에 가장 편한 도구.**

### 3.3 카메라 영상만 보기

```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
ros2 run rqt_image_view rqt_image_view
```
→ 상단 드롭다운에서 토픽 선택 (`/d435_center/d435_center/color/image_raw` 등).

### 3.4 RViz Fixed Frame
- `world` 또는 `base_link` 권장
- `map`으로 되어 있으면 변경 (TR-Works bag엔 map frame 없음)

---

## 4. 자주 만나는 문제 & 해결

### 4.1 `[WARN] unknown type 'tr_works_hand_controller/msg/...'`
→ **워크스페이스 source 누락.** `/hand/state`, `/hand/command` 등이 bag에서 빠짐.
**해결:** `source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash` 먼저.

### 4.2 `/joint_states` publisher 1개만 잡힘 (메시지 카운트 적음)
→ QoS mismatch. `joint_state_broadcaster`(TRANSIENT_LOCAL) vs 나머지(VOLATILE).
**해결:** `--qos-profile-overrides-path qos_override.yaml` 옵션 사용.

### 4.3 `Cache buffers lost messages`
→ 카메라 데이터 양 vs 디스크 쓰기 속도 불균형.
**해결:** `--max-cache-size 1073741824` (1GB) 옵션 사용.

### 4.4 RViz에서 로봇 안 움직임 (TF가 죽은 것처럼 보임)
→ `use_sim_time` 미설정 또는 변경 직후 TF buffer 꼬임.
**해결:**
```bash
ros2 param set /robot_state_publisher use_sim_time true
ros2 param set /rviz use_sim_time true
```
이래도 안 되면 RViz 재시작.

### 4.5 `bag play` 명령이 죽음
→ 이전에 띄운 bag play 프로세스와 충돌.
**해결:** `pgrep -af "bag play"` 확인 후 기존 프로세스 `kill`.

---

## 5. 카메라 시리얼 ↔ 네임스페이스 매핑

| 네임스페이스 | 시리얼 | 비고 |
|---|---|---|
| `d435_center` | `818312070932` | 기존 카메라 |
| `d435_center_upper` | `819612070814` | 2026-05-04 추가 |

`rs-enumerate-devices` 명령으로 시리얼 확인 가능.

---

## 6. 참고 파일

- `qos_override.yaml` — joint_states QoS override
- `README.md` — 명명 규칙
- `HOWTO.md` — 이 문서

## 7. 관련 launch 파일 (이 워크스페이스 기준)

| 파일 | 용도 |
|---|---|
| `tr_works_bringup/launch/display.launch.py` | RViz + (옵션)robot_state_publisher |
| `tr_works_bringup/launch/verify_stage_a1.launch.py` | RViz + RSP + joint_state_publisher_gui (검증용) |
| `tr_works_motion_viewer/launch/sil_viewer.launch.py` | RViz + RSP + JSON motion 재생 (bag 아님) |
| `tr_works_joint_control_ui/launch/joint_control.launch.py` | 조인트 컨트롤 UI |
| `tr_works_joint_control_ui/launch/hardware_only.launch.py` | RSP 제외, 하드웨어만 |
