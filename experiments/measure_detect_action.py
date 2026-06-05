#!/usr/bin/env python3
"""Measure the REAL production DetectBox action path (/yolov8_node/detect).

Drives the live yolo_remote_node action N times and records, per goal:
  * action_ms  : wall-clock from send_goal to result (full ROS2 action round trip,
                 includes goal handshake, frame grab, JPEG encode, remote POST,
                 publish, result return)
  * remote_ms  : node-reported client->Pi->client RTT (from result.message)
  * infer_ms   : Pi-reported server infer time (from result.message)
parsed from result.message "ok (remote NNms, infer MMms)".
"""
import re
import statistics
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from yolov8_detection_msgs.action import DetectBox

N, WARM = 30, 5
MSG_RE = re.compile(r"remote\s+([\d.]+)ms,\s*infer\s+([\d.]+)ms")


class Client(Node):
    def __init__(self):
        super().__init__("detect_action_timer")
        self.cli = ActionClient(self, DetectBox, "/yolov8_node/detect")

    def one(self):
        goal = DetectBox.Goal()
        goal.confidence = 0.35
        goal.prompts = ""
        goal.publish_annotated = False
        t0 = time.monotonic()
        gh_future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_future)
        gh = gh_future.result()
        if not gh.accepted:
            return None
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        dt = (time.monotonic() - t0) * 1e3
        r = res_future.result().result
        return dt, r


def stat(xs):
    return (f"mean {statistics.mean(xs):6.1f}  median {statistics.median(xs):6.1f}  "
            f"min {min(xs):6.1f}  max {max(xs):6.1f}  std {statistics.pstdev(xs):5.1f} ms")


def main():
    rclpy.init()
    c = Client()
    if not c.cli.wait_for_server(timeout_sec=10.0):
        print("DetectBox server /yolov8_node/detect not available"); return
    act, rem, inf, nd = [], [], [], []
    for i in range(N + WARM):
        out = c.one()
        if out is None:
            print(f"goal {i} rejected"); continue
        dt, r = out
        m = MSG_RE.search(r.message or "")
        if i >= WARM:
            act.append(dt)
            nd.append(int(r.num_detections))
            if m:
                rem.append(float(m.group(1)))
                inf.append(float(m.group(2)))
    print(f"DetectBox action over {len(act)} goals  (msg sample: '{r.message}')")
    print(f"  detections/goal: mean {statistics.mean(nd):.1f} (min {min(nd)} max {max(nd)})\n")
    print(f"  [full] action round-trip : {stat(act)}")
    if rem: print(f"  [2b+3+4] node->Pi->node RTT: {stat(rem)}")
    if inf: print(f"  [3] Pi infer_ms (optim)   : {stat(inf)}")
    if act and inf:
        ros_ovh = [a - r for a, r in zip(act, rem)] if rem else []
        if ros_ovh:
            print(f"  [ROS] action+grab+encode+pub overhead (action-RTT): {stat(ros_ovh)}")
    c.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
