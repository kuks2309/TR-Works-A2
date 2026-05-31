"""Managed subprocess for ROS2 launch / run commands triggered by GUI buttons.

Each ManagedProcess wraps a subprocess.Popen that runs in its own process group
so we can SIGINT (then SIGKILL on timeout) the whole tree on stop. Logs are
captured to /tmp/tr_works_scenario_ui_<name>.log.
"""

from __future__ import annotations

import os
import signal
import subprocess
from typing import List, Optional


class ManagedProcess:
    def __init__(self, name: str) -> None:
        self.name = name
        self._proc: Optional[subprocess.Popen] = None
        self._log_path = f"/tmp/tr_works_scenario_ui_{name.replace(' ', '_').lower()}.log"
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
            preexec_fn=os.setsid,
        )
        return True

    def stop(self, sigint_timeout: float = 5.0) -> None:
        if not self.running:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            try:
                self._proc.wait(timeout=sigint_timeout)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                self._proc.wait(timeout=2.0)
        except (ProcessLookupError, PermissionError):
            pass
        finally:
            self._proc = None
            if self._log_file:
                self._log_file.close()
                self._log_file = None
