"""What the Diagnostics tab monitors for the openarmx stack.

Shared by the ROS bridge (to create the rate-monitoring subscriptions before
spin starts) and by the Diagnostics tab (to build the table rows).
"""

from __future__ import annotations

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped
from openarmx_scenario_player_msgs.msg import MoveL
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

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
    ("viz", "/openarmx_scenario_rviz", "RViz2 (선택)"),
    ("scenario", "/scenario_player", "시나리오 플레이어 (선택)"),
]
# Nodes that are optional → absence shows IDLE (gray) instead of DOWN (red).
OPTIONAL_NODES = {"/openarmx_scenario_rviz", "/scenario_player"}

# --- Topics to monitor: (category, topic, msg_type, qos_kind, kind, detail) ---
# kind: "rate" (expects continuous publishing), "event" (sporadic), "latched".
#
# Tailored to the ACTIVE cyclo MoveL backend. The joint_trajectory_controller
# (JTC) /joint_trajectory topics are intentionally NOT monitored: MoveIt/Pilz
# drive JTC via the FollowJointTrajectory *action* (not that topic), and the
# cyclo backend bypasses JTC entirely — so those topics are always idle and
# misleading. If/when a Pilz/JTC backend is used, re-add them (or monitor
# /<ctrl>/controller_state, which JTC streams continuously when active).
TOPIC_SPECS = [
    # robot state (continuous)
    ("state", "/joint_states", JointState, QOS_SENSOR, "rate", "관절 상태 (≈50 Hz)"),
    ("state", "/dynamic_joint_states", DynamicJointState, QOS_SENSOR, "rate",
     "ros2_control 상태 (≈20 Hz)"),
    ("tf", "/tf", TFMessage, QOS_SENSOR, "rate", "TF(Transform) 변환"),
    ("tf", "/tf_static", TFMessage, QOS_LATCHED, "latched", "정적 TF(Transform) (latched)"),
    ("description", "/robot_description", String, QOS_LATCHED, "latched", "URDF (latched)"),
    # cyclo MoveL command path (sporadic — on each MoveL goal)
    ("command", "/openarmx/left/movel", MoveL, QOS_DEFAULT, "event", "좌팔 cyclo MoveL 명령"),
    ("command", "/openarmx/right/movel", MoveL, QOS_DEFAULT, "event", "우팔 cyclo MoveL 명령"),
    # cyclo end-effector pose feedback (continuous ≈35 Hz)
    ("feedback", "/openarmx/left/ee_pose", PoseStamped, QOS_SENSOR, "rate",
     "좌팔 EE(End Effector) 자세 피드백"),
    ("feedback", "/openarmx/right/ee_pose", PoseStamped, QOS_SENSOR, "rate",
     "우팔 EE(End Effector) 자세 피드백"),
    # cyclo controller error (sporadic)
    ("error", "/openarmx_left_movel_controller/controller_error", String, QOS_DEFAULT,
     "event", "좌팔 cyclo 컨트롤러 에러"),
    ("error", "/openarmx_right_movel_controller/controller_error", String, QOS_DEFAULT,
     "event", "우팔 cyclo 컨트롤러 에러"),
    # scenario
    ("scenario", "/scenario_player/status", String, QOS_LATCHED, "event", "시나리오 상태"),
]

# Minimum Hz for a "rate" topic to count as OK; below → WARN.
RATE_OK_MIN_HZ = 1.0
