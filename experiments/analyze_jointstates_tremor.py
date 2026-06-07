#!/usr/bin/env python3
"""/joint_states 기반 떨림(tremor) 스펙트럼 + 루프 타이밍 분석.

controller_state(저속) 대신 /joint_states(고속)의 position/velocity/effort 를 써서
- 관절별 속도 RMS / 속도 부호반전율 / 속도 지배주파수(FFT)
- effort(모터토크) std / 지배주파수  → 토크 채터(buzz) 검출
- 메시지 도착 간격(dt) 통계 → 제어/피드백 루프 타이밍 불규칙성
정지(hold) 구간만 골라 본다(움직임 제외). 떨림은 정지 시 가장 잘 보임.
"""
import sqlite3, glob, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_dir = sys.argv[1]
db3 = glob.glob(bag_dir.rstrip('/') + '/*.db3')[0]
con = sqlite3.connect(db3); cur = con.cursor()
name2 = {}
for tid, name, typ in cur.execute("SELECT id,name,type FROM topics"):
    name2[name] = (tid, typ)

tid, typ = name2['/joint_states']
M = get_message(typ)
ts, pos, vel, eff, names = [], [], [], [], None
for t, data in cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
    m = deserialize_message(bytes(data), M)
    if names is None:
        names = list(m.name)
    ts.append(t)
    pos.append(list(m.position)); vel.append(list(m.velocity)); eff.append(list(m.effort))
con.close()
ts = np.array(ts, np.float64)/1e9
pos = np.array(pos); vel = np.array(vel) if all(len(v)==len(names) for v in vel) else None
eff = np.array(eff) if all(len(e)==len(names) for e in eff) else None
DEG = 180/np.pi
dt = np.diff(ts); fs = 1.0/np.median(dt)
print(f"== {bag_dir}  /joint_states ==")
print(f"  N={len(ts)} dur={ts[-1]-ts[0]:.1f}s  rate med={fs:.1f}Hz  "
      f"dt ms p50={np.percentile(dt,50)*1e3:.1f} p95={np.percentile(dt,95)*1e3:.1f} "
      f"p99={np.percentile(dt,99)*1e3:.1f} max={dt.max()*1e3:.1f}")
print(f"  vel field: {'yes' if vel is not None else 'EMPTY'}   eff field: {'yes' if eff is not None else 'EMPTY'}")

def dom_freq(x, fs):
    x = x - x.mean()
    if len(x) < 16 or np.allclose(x, 0): return 0.0
    w = np.hanning(len(x)); X = np.abs(np.fft.rfft(x*w)); f = np.fft.rfftfreq(len(x), 1/fs)
    X[0] = 0
    return float(f[np.argmax(X)])

def sign_rev_rate(x, fs, thr):
    d = np.diff(x); d[np.abs(d) < thr] = 0
    s = np.sign(d); s = s[s != 0]
    return (np.sum(s[1:] != s[:-1]) / (len(x)/fs)) if len(s) > 1 else 0.0

# 정지(hold) 구간 검출: 관절 전체 속도크기가 작은 샘플
if vel is not None:
    speed = np.abs(vel).sum(axis=1)
    hold = speed < np.percentile(speed, 40)   # 하위 40% = 정지에 가까움
else:
    hold = np.ones(len(ts), bool)
print(f"  hold-구간 샘플 {hold.sum()}/{len(ts)}")
hdr = f"  {'joint':22s} {'vRMS°/s':>9s} {'vMAX°/s':>9s} {'vRev/s':>7s} {'vFHz':>6s}"
if eff is not None: hdr += f" {'effSTD':>7s} {'effP2P':>7s} {'effFHz':>7s}"
print(hdr)
order = [i for i,n in enumerate(names) if 'finger' not in n]
for i in order:
    vR = vM = vrev = vf = float('nan')
    if vel is not None:
        v = vel[hold, i]*DEG
        vR = np.sqrt(np.mean(v**2)); vM = np.max(np.abs(v))
        vrev = sign_rev_rate(v, fs, thr=0.05*(np.ptp(v)+1e-9))
        vf = dom_freq(v, fs)
    line = f"  {names[i]:22s} {vR:9.2f} {vM:9.2f} {vrev:7.1f} {vf:6.2f}"
    if eff is not None:
        e = eff[hold, i]
        line += f" {e.std():7.3f} {np.ptp(e):7.3f} {dom_freq(e,fs):7.2f}"
    flag = ""
    if vel is not None and vR > 2 and vrev > 5: flag = "  <== 떨림(정지중 속도진동)"
    print(line + flag)
