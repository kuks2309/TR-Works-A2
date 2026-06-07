#!/usr/bin/env python3
"""오른팔 pick 시퀀스 N회 반복 테스트 + 결과 저장.

각 회: 붉은 박스 검출 -> ptp_pick_seq_v2_right.py --run -> stdout 파싱(박스/tilt/finger/결과)
-> experiments/pick_test_results.yaml 누적. 끝에 성공률/실패 요약.

사용: python3 run_pick_tests.py [--n 10]
(ROS2/3d_detect_ws 가 source 된 셸에서 실행)
"""
import os
import re
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
N = int(sys.argv[sys.argv.index("--n") + 1]) if "--n" in sys.argv else 10
RESULTS = os.path.join(HERE, "pick_test_results.yaml")
DETECT = ("ros2 action send_goal /yolov8_node/detect yolov8_detection_msgs/action/DetectBox "
          "\"{prompts: 'mini-box-red', confidence: 0.5, publish_annotated: true}\"")
SEQ = "python3 " + os.path.join(HERE, "ptp_pick_seq_v2_right.py") + " --run"


def sh(cmd, timeout=120):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return "<<TIMEOUT>>"


def detect(retries=3):
    """검출 1개를 얻을 때까지 최대 retries 회 재시도(일시적 0 검출 방지)."""
    for _ in range(retries):
        out = sh(DETECT, timeout=50)
        m = re.search(r"num_detections:\s*(\d+)", out)
        nd = int(m.group(1)) if m else 0
        if nd == 1:
            return 1
        time.sleep(0.6)
    return nd


def main():
    runs = []
    for i in range(N):
        nd = detect()
        if nd != 1:
            runs.append({"i": i + 1, "result": "no_box" if nd == 0 else f"multi({nd})"})
            print(f"[{i+1}/{N}] 검출 {nd} -> skip", flush=True)
            yaml.safe_dump({"n": N, "runs": runs}, open(RESULTS, "w"), allow_unicode=True, sort_keys=False)
            time.sleep(1.0); continue
        out = sh(SEQ, timeout=120)
        box = re.search(r"검출 박스 \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", out)
        tilt = re.search(r"기준자세 tilt=([\d.]+)", out)
        fing = re.search(r"finger=([\d.]+)m.*?(파지 OK|미파지)", out)
        total = re.search(r"TOTAL\s*:\s*([\d.]+)s", out)
        if "SAFE-ABORT" in out:
            res = "abort"
        elif fing and "파지 OK" in fing.group(2):
            res = "success"
        elif out == "<<TIMEOUT>>":
            res = "timeout"
        else:
            res = "fail"
        rec = {"i": i + 1,
               "box": [float(box.group(1)), float(box.group(2)), float(box.group(3))] if box else None,
               "tilt": float(tilt.group(1)) if tilt else None,
               "finger": float(fing.group(1)) if fing else None,
               "total_s": float(total.group(1)) if total else None,
               "result": res}
        runs.append(rec)
        print(f"[{i+1}/{N}] box={rec['box']} tilt={rec['tilt']} finger={rec['finger']} -> {res}", flush=True)
        yaml.safe_dump({"n": N, "runs": runs}, open(RESULTS, "w"), allow_unicode=True, sort_keys=False)
        time.sleep(1.5)

    succ = sum(1 for r in runs if r.get("result") == "success")
    print(f"\n=== 요약: 성공 {succ}/{N} ===", flush=True)
    for r in runs:
        if r.get("result") != "success":
            print(f"  실패#{r['i']}: {r.get('result')} box={r.get('box')} "
                  f"tilt={r.get('tilt')} finger={r.get('finger')}", flush=True)
    yaml.safe_dump({"n": N, "success": succ, "runs": runs},
                   open(RESULTS, "w"), allow_unicode=True, sort_keys=False)
    print(f"-> 저장 {RESULTS}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
