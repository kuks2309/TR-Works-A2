"""Scenario discovery + resolve + sub-step loader.

Directory contract (mirrors the original tr_works_scenario_player layout):

    <scenario_search_path>/
    └── <scenario_name>/
        ├── scenario.json        # top-level: must have 'sequence' (list of sub names)
        └── <sub_name>/
            └── scenario.json    # sub: must have 'steps' (list of step dicts)

`resolve(name_or_path)` accepts:
  - an absolute path ending in .json   (box UI: QFileDialog result)
  - a bare scenario name               (main UI: ListScenarios.names[i])
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Sub:
    name: str
    steps: List[Dict[str, Any]] = field(default_factory=list)
    path: str = ""


@dataclass
class Scenario:
    name: str
    path: str               # absolute path to top-level scenario.json
    description: str = ""
    sub_names: List[str] = field(default_factory=list)
    subs: List[Sub] = field(default_factory=list)
    loaded: bool = False


class ScenarioRepo:
    def __init__(self, search_path: str) -> None:
        self._search_path = search_path or ""

    @property
    def search_path(self) -> str:
        return self._search_path

    # ------------------------------------------------------------------
    # scan: enumerate scenarios in search_path
    # ------------------------------------------------------------------

    def scan(self) -> List[Scenario]:
        if not self._search_path or not os.path.isdir(self._search_path):
            return []
        out: List[Scenario] = []
        for entry in sorted(os.listdir(self._search_path)):
            scenario_dir = os.path.join(self._search_path, entry)
            top = os.path.join(scenario_dir, "scenario.json")
            if not (os.path.isdir(scenario_dir) and os.path.isfile(top)):
                continue
            scenario = self._read_top(top)
            if scenario is not None:
                out.append(scenario)
        return out

    # ------------------------------------------------------------------
    # resolve: name OR absolute json path -> Scenario
    # ------------------------------------------------------------------

    def resolve(self, name_or_path: str) -> Optional[Scenario]:
        if not name_or_path:
            return None
        if os.path.isabs(name_or_path) and name_or_path.endswith(".json"):
            if not os.path.isfile(name_or_path):
                return None
            return self._read_top(name_or_path)
        for s in self.scan():
            if s.name == name_or_path:
                return s
        return None

    # ------------------------------------------------------------------
    # load_subs: populate scenario.subs[*].steps from disk
    # ------------------------------------------------------------------

    def load_subs(self, scenario: Scenario) -> Scenario:
        if scenario.loaded:
            return scenario
        parent_dir = os.path.dirname(scenario.path)
        subs: List[Sub] = []
        for sub_name in scenario.sub_names:
            sub_path = os.path.join(parent_dir, sub_name, "scenario.json")
            sub = Sub(name=sub_name, path=sub_path)
            if os.path.isfile(sub_path):
                try:
                    with open(sub_path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    if isinstance(data, dict) and isinstance(data.get("steps"), list):
                        sub.steps = [s for s in data["steps"] if isinstance(s, dict)]
                except (json.JSONDecodeError, OSError):
                    pass
            subs.append(sub)
        scenario.subs = subs
        scenario.loaded = True
        return scenario

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _read_top(path: str) -> Optional[Scenario]:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return None
        if not isinstance(data, dict):
            return None
        seq = data.get("sequence")
        if not isinstance(seq, list) or not seq:
            return None
        name = str(data.get("name") or os.path.basename(os.path.dirname(path)) or "scenario")
        return Scenario(
            name=name,
            path=os.path.abspath(path),
            description=str(data.get("description") or ""),
            sub_names=[str(x) for x in seq],
        )
