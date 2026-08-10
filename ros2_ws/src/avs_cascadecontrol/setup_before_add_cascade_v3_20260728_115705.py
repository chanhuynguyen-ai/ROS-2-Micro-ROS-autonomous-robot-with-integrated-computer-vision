from setuptools import find_packages, setup
import os
from glob import glob

package_name = "avs_cascadecontrol"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="avs",
    maintainer_email="avs@example.com",
    description="AVS cascade control package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'cascade_controller_v2 = avs_cascadecontrol.cascade_controller_v2:main',
            "lane_outer_pd_node = avs_cascadecontrol.lane_outer_pd_node:main",
            "wheel_inner_pd_node = avs_cascadecontrol.wheel_inner_pd_node:main",
            "cascade_control_monitor_node = avs_cascadecontrol.cascade_control_monitor_node:main",
            "cascade_controller_v1 = avs_cascadecontrol.cascade_controller_v1:main",
            "cascade_controller_avoid = avs_cascadecontrol.cascade_controller_avoid:main",
        ],
    },
)
