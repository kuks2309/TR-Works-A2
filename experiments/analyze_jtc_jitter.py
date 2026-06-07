#!/usr/bin/env python3
"""JTC 떨림(jitter) 정량 분석.

rosbag(db3)에서 JointTrajectoryControllerState 와 effort command 를 읽어
- 관절별 위치 feedback 의 떨림(방향반전율, 지배주파수, peak-to-peak)
- reference vs feedback 추종오차
- gravity-comp effort command 의 진동(주파수/진폭)
를 계산한다. effort 진동이 위치 떨림과 같은 주파수면 = 제어루프 리밋사이클.
"""
import sqlite3, glob, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir = sys.argv[1]
db3 = glob.glob(bag_dir.rstrip('/') + '/*.db3')[0]
con = sqlite3.connect(db3); cur = con.cursor()

topics = {}
for tid, name, typ in cur.execute("SELECT id,name,type FROM topics"):
    topics[tid] = (name, typ)
name2tid = {n: tid for tid, (n, t) in topics.items()}

def load(topic_name):
    tid = name2tid.get(topic_name)
    if tid is None:
        return [], []
    msgtype = get_message(topics[tid][1])
    ts, msgs = [], []
    for t, data in cur.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        ts.append(t); msgs.append(deserialize_message(bytes(data), msgtype))
    return np.array(ts, dtype=np.float64) / 1e9, msgs

def reversal_rate(x, dt_mean):
    """단위시간당 1차차분 부호반전 횟수 (떨림 지표). DC drift 제거."""
    d = np.diff(x)
    # 미세 노이즈 무시 임계: p2p 의 2%
    thr = 0.02 * (x.max() - x.min() + 1e-9)
    d[np.abs(d) < thr] = 0.0
    s = np.sign(d); s = s[s != 0]
    if len(s) < 2:
        return 0.0
    rev = np.sum(s[1:] != s[:-1])
    return rev / (len(x) * dt_mean)

def dom_freq(x, fs):
    """평균 제거 후 FFT 지배주파수 (0Hz 제외)."""
    x = x - x.mean()
    if len(x) < 16 or np.allclose(x, 0):
        return 0.0, 0.0
    w = np.hanning(len(x)); X = np.abs(np.fft.rfft(x * w))
    f = np.fft.rfftfreq(len(x), d=1.0/fs)
    X[0] = 0.0
    k = int(np.argmax(X))
    return f[k], float(X[k])

# ---- controller_state (좌/우 자동) ----
for side in ('left', 'right'):
    cs_topic = f"/{side}_joint_trajectory_controller/controller_state"
    ts, msgs = load(cs_topic)
    if not msgs:
        continue
    dt = np.diff(ts); dt_mean = dt.mean(); fs = 1.0/dt_mean
    jn = list(msgs[0].joint_names)
    nj = len(jn)
    # 필드 존재 확인 (humble: reference/feedback/error/output + desired/actual)
    m0 = msgs[0]
    def field(m, *names):
        for n in names:
            p = getattr(m, n, None)
            if p is not None and len(getattr(p, 'positions', [])) == nj:
                return n
        return None
    f_ref = field(m0, 'reference', 'desired')
    f_fb  = field(m0, 'feedback', 'actual')
    f_out = field(m0, 'output')
    print("="*78)
    print(f"[{side.upper()}] {cs_topic}")
    print(f"  samples={len(msgs)}  dur={ts[-1]-ts[0]:.1f}s  rate={fs:.1f}Hz  "
          f"jitter_dt(p99-p50)={np.percentile(dt,99)*1e3:.1f}/{np.percentile(dt,50)*1e3:.1f}ms")
    print(f"  fields: ref={f_ref} fb={f_fb} out={f_out}")
    ref = np.array([getattr(m, f_ref).positions for m in msgs]) if f_ref else None
    fb  = np.array([getattr(m, f_fb).positions  for m in msgs]) if f_fb  else None
    out = np.array([getattr(m, f_out).positions for m in msgs]) if (f_out and len(getattr(m0,f_out).positions)==nj) else None
    vel = None
    try:
        vel = np.array([getattr(m, f_fb).velocities for m in msgs]) if (f_fb and len(getattr(m0,f_fb).velocities)==nj) else None
    except Exception:
        vel = None
    DEG = 180.0/np.pi
    print(f"  {'joint':22s} {'fb_p2p°':>8s} {'rev/s':>7s} {'fHz':>6s} "
          f"{'err_mean°':>9s} {'err_std°':>8s} {'vel_rms°/s':>10s}")
    for j in range(nj):
        fbj = fb[:, j]
        p2p = (fbj.max()-fbj.min())*DEG
        rr = reversal_rate(fbj, dt_mean)
        fpk, _ = dom_freq(fbj, fs)
        if ref is not None:
            err = (ref[:, j]-fbj)*DEG
            em, es = err.mean(), err.std()
        else:
            em = es = float('nan')
        vr = (np.sqrt(np.mean((vel[:, j])**2))*DEG) if vel is not None else float('nan')
        flag = "  <== 떨림" if (rr > 3 and p2p > 0.3) else ""
        print(f"  {jn[j]:22s} {p2p:8.3f} {rr:7.1f} {fpk:6.2f} {em:9.3f} {es:8.3f} {vr:10.2f}{flag}")

# ---- effort command (gravity comp) ----
for side in ('left', 'right'):
    et = f"/{side}_forward_effort_controller/commands"
    ts, msgs = load(et)
    if not msgs:
        continue
    dt = np.diff(ts); fs = 1.0/dt.mean()
    arr = np.array([list(m.data) for m in msgs])
    print("="*78)
    print(f"[{side.upper()}] {et}  samples={len(msgs)} rate={fs:.1f}Hz  cols={arr.shape[1]}")
    print(f"  {'idx':>3s} {'mean':>9s} {'std':>9s} {'p2p':>9s} {'fHz':>6s}")
    for j in range(arr.shape[1]):
        c = arr[:, j]
        fpk, _ = dom_freq(c, fs)
        print(f"  {j:3d} {c.mean():9.3f} {c.std():9.3f} {c.max()-c.min():9.3f} {fpk:6.2f}")

con.close()
