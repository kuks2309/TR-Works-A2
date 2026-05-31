#!/usr/bin/env python3
"""Adapter: cyclo eef_interactive_marker (MoveL) -> vr_controller (PoseStamped).

cyclo's ``eef_interactive_marker_node`` publishes the RViz 6-DoF marker pose
as a ``robotis_interfaces/msg/MoveL`` message (which is just
``PoseStamped + Duration``). cyclo's ``vr_controller_node``, on the other
hand, subscribes to plain ``geometry_msgs/msg/PoseStamped`` on the
``r_goal_pose_topic`` / ``l_goal_pose_topic`` channels.

This thin adapter forwards the ``.pose`` field unchanged and drops the
``time_from_start`` which the VR controller does not use (the VR controller
is a velocity-resolved reactive controller, not a trajectory follower).

One instance per arm (left, right), remapped to the corresponding cyclo
marker output topic and vr_controller goal topic.

Parameters:
  input_topic  (str, default "/marker_movel")   -- cyclo MoveL from the marker
  output_topic (str, default "/marker_pose")    -- PoseStamped for vr_controller
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robotis_interfaces.msg import MoveL


class MarkerToPoseStamped(Node):
    def __init__(self):
        super().__init__("marker_to_posestamped")
        self.input_topic = self.declare_parameter("input_topic", "/marker_movel").value
        self.output_topic = self.declare_parameter("output_topic", "/marker_pose").value
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(MoveL, self.input_topic, self._on_movel, 10)
        self.get_logger().info(
            f"forwarding MoveL.pose: {self.input_topic} -> {self.output_topic}")

    def _on_movel(self, msg: MoveL) -> None:
        self.pub.publish(msg.pose)


def main(argv=None):
    rclpy.init(args=argv)
    node = MarkerToPoseStamped()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
