# openarmx_scenario_ui

PyQt5 GUI for the openarmx scenario player. Two windows:

| Executable | Window | Purpose |
|---|---|---|
| `scenario_ui.py` | Scenario Player (combo + Play/Cancel) | Process management + registered scenario playback |
| `scenario_box_ui.py` | Scenario Box UI | One-click sub-motion playback from a loaded `scenario.json` |

Both UIs share `ScenarioRosBridge` (a Qt-threaded rclpy node) and talk to
`openarmx_scenario_player` via the `scenario_player/play` action and
`scenario_player/list` service (defined in `openarmx_scenario_player_msgs`).

## Build

```bash
cd ~/openarmx_ws
colcon build --symlink-install \
  --packages-select openarmx_scenario_player_msgs openarmx_scenario_player openarmx_scenario_ui
source install/setup.bash
```

## Run

### Auto-start sequence (Scenario Player window)

```bash
ros2 run openarmx_scenario_ui scenario_ui.py
# or
ros2 launch openarmx_scenario_ui scenario_ui.launch.py
```

Auto-launches `openarmx_bringup openarmx.bimanual.launch.py` after 0.5 s,
`openarmx_scenario_player scenario_player_node.py` after 5 s, then refreshes
the scenario combo box. Pass `--no-auto` to disable.

### Manual sequence (Box UI)

Bring up hardware + player in separate terminals first:

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py
ros2 run openarmx_scenario_player scenario_player_node.py \
  --ros-args -p scenario_search_path:=$HOME/openarmx_ws/scenarios
```

Then:

```bash
ros2 run openarmx_scenario_ui scenario_box_ui.py
```

Click **Load Scenario...** and pick a top-level `scenario.json` (the file
that has a `sequence` array). Each sub-scenario gets a clickable box.

## Scenarios directory resolution

Resolved in order:

1. `$OPENARMX_SCENARIOS_DIR`
2. `~/openarmx_ws/scenarios`
3. Walk up to 10 parents from the installed module, looking for `scenarios/` or `Scenarios/`.

A smoke-test scenario lives at `openarmx_ws/scenarios/example_scenario/`.

## Stop All

`STOP ALL` button runs `~/kill_all_ros2.sh`. Install it once:

```bash
cp ~/openarmx_ws/scripts/kill_all_ros2.sh ~/kill_all_ros2.sh
chmod +x ~/kill_all_ros2.sh
```

## Notes

- `chmod +x` is required for `scripts/scenario_ui.py` and `scripts/scenario_box_ui.py`
  in source — `--symlink-install` mirrors the source mode.
- The `.ui` files' `objectName` attributes are a contract with the Python
  code. Do not rename widgets without updating both sides.
- This package depends on `openarmx_scenario_player_msgs` (interfaces) and
  `openarmx_scenario_player` (backend node, stub). Build msgs first.
