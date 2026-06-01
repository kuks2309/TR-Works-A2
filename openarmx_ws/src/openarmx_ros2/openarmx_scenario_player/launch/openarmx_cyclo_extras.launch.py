"""Optional cyclo follower extras: cyclo_sim (mock HW + JTC + RSP) +
vr_controller_node (QP+CBF bimanual follower).

Spawned by scenario_ui.py ONLY when --follower=cyclo. Decoupled from RViz +
ee_leader_markers (those are always spawned by openarmx_scenario_workflow)
so a missing cyclo_motion_controller_ros dependency does not take down RViz.

Requires openarmx_motion and (transitively) cyclo_motion_controller_ros to
be built in the workspace.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_pkg = FindPackageShare("openarmx_motion")

    cyclo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [motion_pkg, "launch", "openarmx_cyclo_sim.launch.py"])),
    )

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [motion_pkg, "launch", "openarmx_vr_bimanual.launch.py"])),
    )

    return LaunchDescription([cyclo_sim, follower])
