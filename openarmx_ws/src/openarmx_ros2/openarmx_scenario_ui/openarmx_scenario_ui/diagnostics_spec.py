"""What the Diagnostics tab monitors for the openarmx stack.

Shared by the ROS bridge (to create the rate-monitoring subscriptions before
spin starts) and by the Diagnostics tab (to build the table rows).
"""

from __future__ import annotations

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory

# QoS kinds resolved by the bridge.
QOS_SENSOR = "sensor"     # best_effort, volatile  (high-rate; compatible w/ reliable pubs)
QOS_LATCHED = "latched"   # reliable, transient_local, depth 1
QOS_DEFAULT = "default"   # reliable, volatile

# --- Nodes to monitor: (category, node_name, detail) ---
NODE_SPECS = [
    ("controller", "/controller_manager", "ros2_control 컨트롤러 매니저"),
    ("controller", "/joint_state_broadcaster", "joint_states 브로드캐스터"),
    ("controller", "/left_joint_trajectory_controller", "좌팔 트래젝토리 컨트롤러"),
    ("controller", "/right_joint_trajectory_controller", "우팔 트래젝토리 컨트롤러"),
    ("gripper", "/left_gripper_controller", "좌 그리퍼 컨트롤러"),
    ("gripper", "/right_gripper_controller", "우 그리퍼 컨트롤러"),
    ("description", "/robot_state_publisher", "URDF → TF 발행"),
    ("ui", "/openarmx_scenario_ui", "Scenario/Joint UI 노드"),
    ("viz", "/rviz2", "RViz2 (선택)"),
    ("scenario", "/scenario_player", "시나리오 플레이어 (선택)"),
]
# Nodes that are optional → absence shows IDLE (gray) instead of DOWN (red).
OPTIONAL_NODES = {"/rviz2", "/scenario_player"}

# --- Topics to monitor: (category, topic, msg_type, qos_kind, kind, detail) ---
# kind: "rate" (expects continuous publishing), "event" (sporadic), "latched".
TOPIC_SPECS = [
    ("state", "/joint_states", JointState, QOS_SENSOR, "rate", "관절 상태 (≈50 Hz)"),
    ("tf", "/tf", TFMessage, QOS_SENSOR, "rate", "TF 변환"),
    ("description", "/robot_description", String, QOS_LATCHED, "latched", "URDF (latched)"),
    ("command", "/left_joint_trajectory_controller/joint_trajectory",
     JointTrajectory, QOS_DEFAULT, "event", "좌팔 명령"),
    ("command", "/right_joint_trajectory_controller/joint_trajectory",
     JointTrajectory, QOS_DEFAULT, "event", "우팔 명령"),
    ("scenario", "/scenario_player/status", String, QOS_LATCHED, "event", "시나리오 상태"),
]

# Minimum Hz for a "rate" topic to count as OK; below → WARN.
RATE_OK_MIN_HZ = 1.0
