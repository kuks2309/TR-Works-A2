#!/usr/bin/env python3
"""Full live ROS2 pipeline timing through the REAL production nodes (remote Pi).

Drives /yolov8_node/detect (yolo_remote_node -> Pi Hailo) N times and, per goal,
records the whole chain:
  action_ms     : send_goal -> result  (capture+encode+POST+Pi infer+publish)
                  with remote_ms / infer_ms parsed from result.message
  det->marker_ms: /yolov8_node/detections arrival -> /detected_boxes_markers
                  arrival = box_perception_node depth ROI + TF(cam->base) + marker
                  build  (== stage 5 box-position calc + stage 6 marker publish,
                  excluding RViz GPU render which is display-refresh bound)
"""
import re
import statistics
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from yolov8_detection_msgs.action import DetectBox

N, WARM = 25, 5
MSG_RE = re.compile(r"remote\s+([\d.]+)ms,\s*infer\s+([\d.]+)ms")


class Meter(Node):
    def __init__(self):
        super().__init__("full_pipeline_meter")
        self.cli = ActionClient(self, DetectBox, "/yolov8_node/detect")
        self.create_subscription(String, "/yolov8_node/detections", self._on_det, 10)
        self.create_subscription(MarkerArray, "/detected_boxes_markers", self._on_mk, 10)
        self._det_t = None
        self._mk_t = None
        self._n_markers = 0

    def _on_det(self, _m):
        self._det_t = time.monotonic()

    def _on_mk(self, m):
        self._mk_t = time.monotonic()
        # DELETEALL marker + (cube,text) pairs per box
        self._n_markers = max(0, sum(1 for mk in m.markers if mk.ns == "boxes"))


def wait(fut, timeout=8.0):
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.002)
    return fut.result() if fut.done() else None


def stat(xs):
    return (f"mean {statistics.mean(xs):6.1f}  median {statistics.median(xs):6.1f}  "
            f"min {min(xs):6.1f}  max {max(xs):6.1f}  std {statistics.pstdev(xs):5.1f} ms")


def main():
    rclpy.init()
    node = Meter()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not node.cli.wait_for_server(timeout_sec=10.0):
        print("DetectBox /yolov8_node/detect not available"); return

    act, rem, inf, chain, nd, nm = [], [], [], [], [], []
    last_msg = ""
    for i in range(N + WARM):
        node._det_t = node._mk_t = None
        node._n_markers = 0
        g = DetectBox.Goal(); g.confidence = 0.35; g.prompts = ""; g.publish_annotated = False
        t0 = time.monotonic()
        gh = wait(node.cli.send_goal_async(g))
        if gh is None or not gh.accepted:
            print(f"goal {i} rejected/timeout"); continue
        rr = wait(gh.get_result_async())
        t1 = time.monotonic()
        if rr is None:
            print(f"goal {i} no result"); continue
        r = rr.result
        last_msg = r.message or ""
        # let box_perception publish markers from the just-published detections
        t_wait = time.monotonic()
        while (node._mk_t is None or node._det_t is None) and time.monotonic() - t_wait < 1.0:
            time.sleep(0.003)
        if i >= WARM:
            act.append((t1 - t0) * 1e3)
            nd.append(int(r.num_detections))
            m = MSG_RE.search(last_msg)
            if m:
                rem.append(float(m.group(1))); inf.append(float(m.group(2)))
            if node._det_t and node._mk_t and node._mk_t >= node._det_t:
                chain.append((node._mk_t - node._det_t) * 1e3)
                nm.append(node._n_markers)
        time.sleep(0.05)

    print(f"\nFULL LIVE PIPELINE over {len(act)} goals  (msg: '{last_msg}')")
    print(f"  detections/goal: mean {statistics.mean(nd):.1f}" if nd else "  no detections")
    if nm: print(f"  markers(boxes) in RViz: mean {statistics.mean(nm):.1f} (min {min(nm)} max {max(nm)})")
    print()
    print(f"  [1-4] DetectBox action round-trip : {stat(act)}" if act else "  no action timing")
    if rem: print(f"     |- node->Pi->node RTT          : {stat(rem)}")
    if inf: print(f"     |- [3] Pi infer_ms (optimised) : {stat(inf)}")
    if chain: print(f"  [5+6] detections->markers (perception+marker publish): {stat(chain)}")
    else: print("  [5+6] no marker chain captured (markers may be filtered by workspace bounds)")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
