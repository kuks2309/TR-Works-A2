from glob import glob
from setuptools import find_packages, setup

package_name = "box_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="amap",
    maintainer_email="kukwonko@gmail.com",
    description="Box / table-top plane detection from RealSense rosbag PointCloud2 streams.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "analyze_planes = box_detection.analyze_planes:main",
            "visualize_planes = box_detection.visualize_planes:main",
            "live_plane_detector = box_detection.live_plane_detector:main",
        ],
    },
)
