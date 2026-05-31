#!/usr/bin/env python3
"""Stand-alone verification for the OpenArmX MoveL solver (no real robot).

Publishes a fake ``/joint_states`` (7 left-arm joints at home), sends one MoveL
goal pose, and prints the resulting ``joint_command`` so you can confirm the QP
solver is closing the loop. The commanded joints should drift smoothly from the
home configuration toward an IK solution for the goal pose.

Run AFTER the solver launch is up:
    ros2 launch openarmx_pick openarmx_movel.launch.py
    python3 verify_solver.py
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from robotis_interfaces.msg import MoveL

LEFT_JOINTS = [f"openarmx_left_joint{i}" for i in range(1, 8)]
HOME = [0.0] * 7


class Verifier(Node):
    def __init__(self):
        super().__init__("openarmx_solver_verifier")
        self.q = list(HOME)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.movel_pub = self.create_publisher(MoveL, "/openarmx/movel", 10)
        self.create_subscription(
            JointTrajectory, "/openarmx/left_arm/joint_trajectory", self._on_cmd, 10)
        # Echo the latest command at the same rate joint_states is fed back, and
        # feed the command back as the new "measured" state to simulate a robot.
        self.create_timer(0.01, self._tick)
        self.create_timer(1.0, self._send_goal_once)
        self._goal_sent = False
        self._cmd_count = 0
        self.get_logger().info("Verifier up: feeding /joint_states, awaiting commands.")

    def _tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(LEFT_JOINTS)
        js.position = list(self.q)
        js.velocity = [0.0] * 7
        self.js_pub.publish(js)

    def _on_cmd(self, msg: JointTrajectory):
        if not msg.points:
            return
        # Treat the commanded positions as the next measured state (sim).
        idx = {n: i for i, n in enumerate(msg.joint_names)}
        for j, name in enumerate(LEFT_JOINTS):
            if name in idx:
                self.q[j] = msg.points[0].positions[idx[name]]
        self._cmd_count += 1
        if self._cmd_count % 50 == 0:
            pretty = ", ".join(f"{v:+.3f}" for v in self.q)
            self.get_logger().info(f"[cmd {self._cmd_count}] q = [{pretty}]")

    def _send_goal_once(self):
        if self._goal_sent:
            return
        self._goal_sent = True
        m = MoveL()
        m.pose.header.frame_id = "openarmx_body_link0"
        m.pose.header.stamp = self.get_clock().now().to_msg()
        # A reachable goal in front of / below the home EE (body_link0 frame).
        m.pose.pose.position.x = 0.10
        m.pose.pose.position.y = 0.15
        m.pose.pose.position.z = 0.00
        # Identity-ish orientation (gripper pointing down handled later in Stage B).
        m.pose.pose.orientation.w = 1.0
        m.time_from_start.sec = 3
        self.movel_pub.publish(m)
        self.get_logger().info("Sent MoveL goal (0.10, 0.15, 0.00) over 3 s.")


def main():
    rclpy.init()
    node = Verifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
