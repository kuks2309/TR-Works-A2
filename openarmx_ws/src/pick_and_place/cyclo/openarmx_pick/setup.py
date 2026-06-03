from glob import glob

from setuptools import find_packages, setup

package_name = "openarmx_pick"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/scripts", glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openarmx",
    maintainer_email="drko0808@gmail.com",
    description="Vision-driven single-arm pick for OpenArmX via cyclo_control QP MoveL.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Stage B: box_plane/info -> top-down grasp PoseStamped / MoveL
            "grasp_pose_node = openarmx_pick.grasp_pose_node:main",
        ],
    },
)
