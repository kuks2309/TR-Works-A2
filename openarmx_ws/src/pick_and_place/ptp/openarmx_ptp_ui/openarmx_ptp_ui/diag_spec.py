"""What the Pipe Health tab monitors for the ptp pick-and-place pipeline.

Shared by the ROS bridge (to build the lazy rate-monitoring subscriptions) and
by the Pipe Health tab (to build the table rows). Modelled on the scenario UI's
diagnostics_spec.py, trimmed to the topics the ptp stack actually uses.
"""

from __future__ import annotations

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, JointState, PointCloud2, Range
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

# QoS kinds resolved by the bridge (subscriber side; chosen to stay compatible
# with each publisher: best_effort accepts reliable pubs; latched needs
# transient_local to receive the retained sample).
QOS_SENSOR = "sensor"     # best_effort, volatile   (high-rate streams)
QOS_LATCHED = "latched"   # reliable, transient_local, depth 1
QOS_DEFAULT = "default"   # reliable, volatile

# (category, topic, msg_type, qos_kind, kind, detail)
# kind: "rate" (continuous), "event" (sporadic), "latched".
TOPIC_SPECS = [
    # ---- robot state ----
    ("state", "/joint_states", JointState, QOS_SENSOR, "rate", "관절 상태 (≈50 Hz)"),
    ("tf", "/tf", TFMessage, QOS_SENSOR, "rate", "TF(Transform) 변환"),
    ("tf", "/tf_static", TFMessage, QOS_LATCHED, "latched", "정적 TF(Transform) (latched)"),
    ("description", "/robot_description", String, QOS_LATCHED, "latched", "URDF (latched)"),
    ("controller", "/left_joint_trajectory_controller/controller_state",
     JointTrajectoryControllerState, QOS_SENSOR, "rate", "좌팔 JTC(Joint Trajectory Controller) 상태"),
    ("controller", "/right_joint_trajectory_controller/controller_state",
     JointTrajectoryControllerState, QOS_SENSOR, "rate", "우팔 JTC(Joint Trajectory Controller) 상태"),
    # ---- sensors ----
    ("sensor", "/camera/camera/color/image_raw", Image, QOS_SENSOR, "rate",
     "D435 컬러 영상 (≈30 Hz)"),
    ("sensor", "/camera/camera/aligned_depth_to_color/image_raw", Image, QOS_SENSOR,
     "rate", "D435 정렬 깊이"),
    ("sensor", "/camera/camera/depth/color/points", PointCloud2, QOS_SENSOR, "rate",
     "포인트클라우드 (XYZRGB)"),
    ("sensor", "/tof/range", Range, QOS_SENSOR, "rate", "TOF(Time-of-Flight) 거리"),
    # ---- detection / perception ----
    ("detect", "/yolov8_node/detections", String, QOS_DEFAULT, "event",
     "YOLO(You Only Look Once) 검출 결과 (JSON)"),
    ("detect", "/yolov8_node/image_annotated", Image, QOS_SENSOR, "event",
     "YOLO(You Only Look Once) 오버레이 영상"),
    ("perception", "/detected_boxes", PoseArray, QOS_LATCHED, "event",
     "인지 박스 (base, latched)"),
    ("perception", "/detected_boxes_markers", MarkerArray, QOS_DEFAULT, "event",
     "박스 RViz 마커"),
]

# Minimum Hz for a "rate" topic to count as OK; below (but > 0) → WARN.
RATE_OK_MIN_HZ = 1.0
