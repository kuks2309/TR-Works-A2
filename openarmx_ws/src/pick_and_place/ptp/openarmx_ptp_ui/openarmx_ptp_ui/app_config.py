"""App config loader for openarmx_ptp_ui (load-only · hand-edited YAML).

운용값(액션 이름 / 상태 타이머 / 검출 워크스페이스 setup / 하드웨어 CAN 인터페이스 /
AlignToBoxes 골 기본값)을 config/ptp_pnp_ui.yaml 에서 읽는다. 쓰기 없음 — 값 변경은
사용자가 YAML 을 직접 편집하고 UI 를 재기동한다.

누락된 키는 DEFAULTS 로 per-key 보완(merge)하므로, 파일이 일부만 있거나 (설치 누락 등으로)
없어도 안전하게 동작한다. launch 의 노드/프로세스 패턴·sweep 같은 내부 식별자는 운용값이
아니므로 여기서 다루지 않고 코드(main_window.build_presets)에 둔다.
"""

from __future__ import annotations

import os

import yaml

# config/ptp_pnp_ui.yaml 과 키가 1:1 대응. YAML 이 SSOT 이고, 이 표는 누락 보완용.
DEFAULTS = {
    "action_name": "/openarmx/ptp_align_to_boxes",
    "status_timer_ms": 2000,
    "detect_ws_setup":
        "/home/openarmx/TR-Works/kkw/China/3d_detect_ws/install/setup.bash",
    "hw_can": {"right": "can2", "left": "can3", "mode": "mit", "can_fd": False},
    "goal_defaults": {
        "z": 0.80, "roll_deg": 180.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
        "arms": "both",
    },
}


def _merge(base: dict, over: dict) -> dict:
    """Deep-merge `over` onto `base` (one level of nested dict)."""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = {**out[k], **v}
        else:
            out[k] = v
    return out


def load_config(path: str) -> dict:
    """Load YAML config at `path`, merged over DEFAULTS (per-key).

    Returns a dict that always has every DEFAULTS key. A missing or unreadable
    file yields DEFAULTS unchanged (the shipped YAML is the intended source; the
    merge only guards against partial/absent files)."""
    if path and os.path.isfile(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return _merge(DEFAULTS, data)
    return dict(DEFAULTS)
