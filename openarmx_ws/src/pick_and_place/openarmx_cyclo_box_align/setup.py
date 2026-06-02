from glob import glob

from setuptools import find_packages, setup

package_name = "openarmx_cyclo_box_align"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openarmx",
    maintainer_email="drko0808@gmail.com",
    description="Bimanual box detect + left/right arm dispatch: move the assigned "
                "arm above its box at a commanded height and hand orientation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "box_align_node = openarmx_cyclo_box_align.box_align_node:main",
        ],
    },
)
