# 그리퍼 컨트롤러 설정 가이드

## 질문: 두 개의 컨트롤러를 동시에 사용해 그리퍼를 제어할 수 있습니까?

**답: 불가능합니다.** ROS2 Control에서는 하나의 관절 명령 인터페이스가 동시에 하나의 컨트롤러에만 점유될 수 있습니다.

## 두 가지 그리퍼 제어 방식 비교

### 방식 1: GripperActionController (기본 설정)

**설정 파일**: `openarmx_v10_bimanual_controllers.yaml`

**특징**:
- ✅ 힘 피드백 지원
- ✅ `max_effort` 제한 지원
- ✅ 스톨(stall) 감지 지원 (`allow_stalling`)
- ✅ 더 안전, 파지 작업에 적합
- ❌ Action 인터페이스 사용, Topic으로 직접 제어 불가
- ❌ 텔레오퍼레이션 시나리오에는 부적합

**사용 방법**:
```bash
# 실행 (기본 설정)
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    robot_controller:=forward_position_controller \
    control_mode:=mit

# 그리퍼 제어 (Action을 통해)
ros2 action send_goal /left_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.04, max_effort: 10.0}}"
```

**토픽**:
- Action: `/left_gripper_controller/gripper_cmd`
- Action: `/right_gripper_controller/gripper_cmd`

---

### 방식 2: ForwardCommandController (새 설정)

**설정 파일**: `openarmx_v10_bimanual_controllers_gripper_forward.yaml`

**특징**:
- ✅ Topic을 통한 직접 제어
- ✅ 실시간 반응이 빠름
- ✅ 텔레오퍼레이션 시나리오에 적합
- ✅ 팔 컨트롤러와 동일한 인터페이스 사용 가능
- ❌ 힘 피드백 없음
- ❌ 스톨 감지 없음
- ❌ 안전 보호를 직접 구현해야 함

**사용 방법**:
```bash
# 실행 (새 설정 사용)
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    robot_controller:=forward_position_controller \
    control_mode:=mit \
    controllers_file:=openarmx_v10_bimanual_controllers_gripper_forward.yaml

# 그리퍼 제어 (Topic을 통해)
ros2 topic pub /left_gripper_forward_position_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "data: [0.04]" --once

# 그리퍼 닫기
ros2 topic pub /left_gripper_forward_position_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "data: [0.0]" --once
```

**토픽**:
- Topic: `/left_gripper_forward_position_controller/commands`
- Topic: `/right_gripper_forward_position_controller/commands`

---

## 어떻게 선택해야 합니까?

### 방식 1(GripperActionController) 선택 기준:
- 파지 작업을 수행해야 하는 경우
- 힘 피드백과 스톨 감지가 필요한 경우
- Action 인터페이스의 복잡성을 수용할 수 있는 경우
- **권장 시나리오: 생산 환경, 정밀 파지, 안전 요구가 높은 시나리오**

### 방식 2(ForwardCommandController) 선택 기준:
- 텔레오퍼레이션을 수행하는 경우
- 그리퍼가 마스터 측을 실시간으로 따라가야 하는 경우
- 그리퍼와 팔이 통합된 제어 인터페이스를 사용하기를 원하는 경우
- **권장 시나리오: 텔레오퍼레이션, 티칭, 실시간 제어 시나리오**

---

## 텔레오퍼레이션 시나리오 전체 설정

듀얼암 텔레오퍼레이션(그리퍼 포함)을 구현하려면 방식 2를 권장합니다.

### 팔로워 측 실행

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 \
    left_can_interface:=can3 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    controllers_file:=openarmx_v10_bimanual_controllers_gripper_forward.yaml
```

### 그리퍼 토픽 검증

```bash
ros2 topic list | grep gripper
# 다음이 표시되어야 합니다:
# /left_gripper_forward_position_controller/commands
# /right_gripper_forward_position_controller/commands
```

### 텔레오퍼레이션 노드에 그리퍼 제어 추가

텔레오퍼레이션 노드에서 다음과 같이 그리퍼 명령을 퍼블리시할 수 있습니다.

```cpp
// 그리퍼 퍼블리셔 생성
gripper_left_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/left_gripper_forward_position_controller/commands", 10);

// 제어 루프에서 그리퍼 위치 퍼블리시
auto gripper_msg = std_msgs::msg::Float64MultiArray();
gripper_msg.data = {gripper_position};  // 0.0 (닫힘) ~ 0.04 (열림)
gripper_left_pub_->publish(gripper_msg);
```

---

## 중요 안내

### ⚠️ 두 컨트롤러를 동시에 실행할 수 없음

**잘못된 방법**:
```bash
# ❌ 이렇게 하면 실패합니다!
ros2 run controller_manager spawner left_gripper_controller
ros2 run controller_manager spawner left_gripper_forward_position_controller
# 두 번째 spawner가 오류를 보고합니다: 인터페이스가 이미 점유됨
```

**이유**:
- `left_gripper_controller`가 `openarmx_left_finger_joint1`의 position 명령 인터페이스를 선언
- `left_gripper_forward_position_controller`는 동일한 인터페이스를 다시 선언할 수 없음
- ROS2 Control은 독점 리소스 모델을 사용

### ⚠️ 컨트롤러 전환 시 재시작 필요

이미 `left_gripper_controller`를 실행 중이고 `left_gripper_forward_position_controller`로 전환하고자 하는 경우:

```bash
# 방법 1: 정지 후 시작 (설정 파일에 정의되지 않은 경우 동작하지 않을 수 있음)
ros2 control switch_controllers \
    --stop-controllers left_gripper_controller \
    --start-controllers left_gripper_forward_position_controller

# 방법 2: launch 파일 재실행 (권장)
# Ctrl+C로 현재 launch 중지
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    controllers_file:=openarmx_v10_bimanual_controllers_gripper_forward.yaml
```

---

## Python 예시: 텔레오퍼레이션에서 그리퍼 제어

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TeleopGripperNode(Node):
    def __init__(self):
        super().__init__('teleop_gripper_node')

        # 퍼블리셔 생성
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/left_gripper_forward_position_controller/commands',
            10
        )

        # 100Hz 타이머 생성
        self.timer = self.create_timer(0.01, self.control_callback)

    def control_callback(self):
        # 마스터 측에서 그리퍼 위치 읽기 (여기서는 0.02로 가정)
        leader_gripper_position = 0.02

        # 팔로워 측으로 전송
        msg = Float64MultiArray()
        msg.data = [leader_gripper_position]
        self.gripper_pub.publish(msg)

def main():
    rclpy.init()
    node = TeleopGripperNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## C++ 예시: 텔레오퍼레이션에서 그리퍼 제어

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TeleopGripperNode : public rclcpp::Node
{
public:
    TeleopGripperNode() : Node("teleop_gripper_node")
    {
        // 퍼블리셔 생성
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_gripper_forward_position_controller/commands", 10);

        // 100Hz 타이머 생성
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TeleopGripperNode::control_callback, this));
    }

private:
    void control_callback()
    {
        // 마스터 측에서 그리퍼 위치 읽기 (여기서는 0.02로 가정)
        double leader_gripper_position = 0.02;

        // 팔로워 측으로 전송
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {leader_gripper_position};
        gripper_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 요약

- **기본 설정**: `GripperActionController` 사용, 파지 작업에 적합
- **텔레오퍼레이션 설정**: `ForwardCommandController` (새 설정 파일) 사용, 실시간 제어에 적합
- **동시 실행 불가**: 하나의 관절은 하나의 컨트롤러로만 제어 가능
- **전환 방법**: `controllers_file` 파라미터로 다른 설정 파일 지정

적용 시나리오에 맞는 설정을 선택하시기 바랍니다!
