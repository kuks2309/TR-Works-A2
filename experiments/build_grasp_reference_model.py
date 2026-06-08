#!/usr/bin/env python3
"""grasp 기준자세 모델 생성 — 핸드 가이드로 모은 grasp 점들로
박스 위치(X, |Y|) -> 목표 grasp 자세(R) 를 만든다.

관찰(핸드 가이드 데이터):
  - yaw ~ 0, roll ~ ±180 (그리퍼 아래 지향, 뒤집힘) — 위치에 거의 무관.
  - pitch = -tilt, tilt 는 박스 X(거리)·|Y|(바깥)에 따라 증가.
모델: tilt(X,|Y|) = a*X + b*|Y| + c  (최소제곱 평면회귀).
목표 자세 R(X,Y) = rpyToMatrix(roll0, -tilt(X,|Y|), yaw0).

입력 : experiments/right_grasp_reference_dataset.yaml (pose_type 미지정/None = grasp)
출력 : experiments/right_grasp_reference_model.yaml
검증 : 적합 tilt 로 재구성한 R 과 실제 캡처 R 의 측지각 오차.
"""
import os
import sys

import numpy as np
import yaml
import pinocchio as pin

HERE = os.path.dirname(os.path.abspath(__file__))


def _arg_side(argv):
    for i, a in enumerate(argv):
        if a.startswith("--side="):
            return a.split("=", 1)[1]
        if a == "--side" and i + 1 < len(argv):
            return argv[i + 1]
    return "right"


SIDE = _arg_side(sys.argv)
DS = os.path.join(HERE, f"{SIDE}_grasp_reference_dataset.yaml")
OUT = os.path.join(HERE, f"{SIDE}_grasp_reference_model.yaml")


def geodesic_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, c)))))


def main():
    d = yaml.safe_load(open(DS))
    grasp = [e for e in d["poses"]
             if e.get("pose_type") in (None, "grasp") and e.get("detected_box_m")]
    if len(grasp) < 3:
        print(f"grasp 점 부족: {len(grasp)}"); return 1

    X = np.array([e["detected_box_m"][0] for e in grasp])
    Yabs = np.array([abs(e["detected_box_m"][1]) for e in grasp])
    tilt = np.array([e["gripper_tilt_deg"] for e in grasp])
    rpy = np.array([e["hand_tcp_rpy_deg"] for e in grasp])  # R,P,Y
    Rs = [np.array(e["hand_tcp_R"]) for e in grasp]

    # tilt = a*X + b*|Y| + c  (최소제곱)
    A = np.column_stack([X, Yabs, np.ones_like(X)])
    (a, b, c), *_ = np.linalg.lstsq(A, tilt, rcond=None)
    tilt_fit = A @ np.array([a, b, c])
    resid = tilt - tilt_fit
    rmse = float(np.sqrt(np.mean(resid ** 2)))

    # roll: 모두 ~±180 -> +180 으로 통일. yaw: ~0 -> 평균.
    roll0 = 180.0
    yaw0 = float(np.mean(rpy[:, 2]))

    # 검증: 적합 tilt 로 R 재구성 -> 실제 R 과 측지각
    geo = []
    for i, e in enumerate(grasp):
        R_recon = pin.rpy.rpyToMatrix(np.radians(roll0), np.radians(-tilt_fit[i]), np.radians(yaw0))
        geo.append(geodesic_deg(R_recon, Rs[i]))
    geo = np.array(geo)

    model = {
        "side": SIDE,
        "type": "grasp_orientation_from_box_xy",
        "made_from_points": len(grasp),
        "tilt_plane": {"a_X": round(float(a), 3), "b_absY": round(float(b), 3),
                       "c": round(float(c), 3),
                       "formula": "tilt_deg = a_X*box_X + b_absY*abs(box_Y) + c"},
        "roll_deg": roll0,
        "yaw_deg": round(yaw0, 2),
        "tilt_clamp_deg": [10.0, 60.0],
        "R_construction": "pin.rpy.rpyToMatrix(rad(roll_deg), rad(-tilt_deg), rad(yaw_deg))",
        "fit_quality": {"tilt_rmse_deg": round(rmse, 2),
                        "orient_geodesic_max_deg": round(float(geo.max()), 2),
                        "orient_geodesic_mean_deg": round(float(geo.mean()), 2)},
        "usage": ("박스 (X,Y) -> tilt=clamp(a_X*X+b_absY*|Y|+c) -> "
                  "R=rpyToMatrix(180,-tilt,yaw0). 접근/하강 모두 이 R 로 solve_pose(6D IK)."),
    }
    with open(OUT, "w") as f:
        yaml.safe_dump(model, f, allow_unicode=True, sort_keys=False)

    print("=== grasp 기준자세 모델 ===")
    print(f"tilt_deg = {a:+.2f}*X {b:+.2f}*|Y| {c:+.2f}")
    print(f"roll={roll0}  yaw={yaw0:+.2f}  tilt_clamp=[10,60]")
    print(f"적합도: tilt RMSE={rmse:.2f}°,  자세 측지각 max={geo.max():.2f}° mean={geo.mean():.2f}°")
    print("\n점별 검증:")
    print(" #  box(X, |Y|)      tilt실측  tilt적합  잔차   자세오차")
    for i, e in enumerate(grasp):
        print(f" {e.get('note','')[:4]:4} ({X[i]:.3f}, {Yabs[i]:.3f})   "
              f"{tilt[i]:5.1f}    {tilt_fit[i]:5.1f}   {resid[i]:+4.1f}   {geo[i]:4.1f}°")
    print(f"\n-> 저장: {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
