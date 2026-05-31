"""Motion backend adapter — Phase 0 no-op stub, Phase 1 hook for real motion.

`execute_step(step_def)` is called once per step from ScenarioPlayerNode's
execute loop (unless dry_run). In Phase 0 this is a no-op: the surrounding
sleep loop drives the simulated duration. In Phase 1, this dispatches on
step_def['type'] and calls into:

  - control_msgs/action/FollowJointTrajectory
      /left_joint_trajectory_controller/follow_joint_trajectory
      /right_joint_trajectory_controller/follow_joint_trajectory
  - control_msgs/action/GripperCommand
      /left_gripper_controller/gripper_cmd
      /right_gripper_controller/gripper_cmd
  - (optional) moveit_msgs/action/MoveGroupSequence (Pilz LIN) via move_group

Joint names: openarmx_{left,right}_joint{1..7},
             openarmx_{left,right}_finger_joint1.
HW bringup: ros2 launch openarmx_bringup openarmx.bimanual.launch.py
"""

from __future__ import annotations

from typing import Any, Dict

from rclpy.node import Node


class MotionBackendStub:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._logger = node.get_logger()

    def execute_step(self, step_def: Dict[str, Any]) -> None:
        """Phase 0: no-op. The execute loop's sleep simulates motion time.

        Phase 1 replacement sketch (do not implement here — separate PR):

            t = step_def.get('type')
            if t == 'movej':
                self._send_jtc(step_def)
            elif t == 'movel':
                self._send_pilz_lin(step_def)
            elif t == 'gripper':
                self._send_gripper(step_def)
            elif t == 'dual_movej':
                self._send_dual_jtc(step_def)
            elif t in (None, 'wait'):
                pass  # let the sleep loop handle it
            else:
                self._logger.warn(f"Unknown step type: {t!r}")
        """
        return None
