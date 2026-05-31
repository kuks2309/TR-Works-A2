"""Managed subprocess for ROS2 launch / run commands triggered by GUI buttons.

Each ManagedProcess wraps a subprocess.Popen that runs in its own process group
so we can SIGINT (then SIGKILL on timeout) the whole tree on stop. Logs are
captured to /tmp/openarmx_scenario_ui_<name>.log.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
from typing import List, Optional


def _sanitize(name: str) -> str:
    return re.sub(r"[^a-z0-9_]", "_", name.lower())


class ManagedProcess:
    def __init__(self, name: str) -> None:
        self.name = name
        self._proc: Optional[subprocess.Popen] = None
        self._log_path = f"/tmp/openarmx_scenario_ui_{_sanitize(name)}.log"
        self._log_file = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def pid(self) -> Optional[int]:
        return self._proc.pid if self._proc is not None else None

    @property
    def log_path(self) -> str:
        return self._log_path

    def start(self, cmd: List[str], env: Optional[dict] = None) -> bool:
        if self.running:
            return False
        self._log_file = open(self._log_path, "w", buffering=1)
        self._proc = subprocess.Popen(
            cmd,
            env=env or os.environ.copy(),
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return True

    def stop(self, sigint_timeout: float = 5.0) -> None:
        if not self.running:
            return
        # Capture the process-group id up front: once the `ros2 launch` parent
        # exits on SIGINT, os.getpgid() would raise and orphaned children
        # (controller_manager, rviz, spawners) would survive. We always SIGKILL
        # the whole group at the end to reap those stragglers.
        try:
            pgid = os.getpgid(self._proc.pid)
        except (ProcessLookupError, PermissionError):
            pgid = None

        if pgid is not None:
            try:
                os.killpg(pgid, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                self._proc.wait(timeout=sigint_timeout)
            except subprocess.TimeoutExpired:
                pass
            # Force-kill the entire group regardless of whether the parent
            # already exited — the group lives as long as any member is alive.
            try:
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

        try:
            self._proc.wait(timeout=2.0)
        except Exception:
            pass
        self._proc = None
        if self._log_file:
            self._log_file.close()
            self._log_file = None
