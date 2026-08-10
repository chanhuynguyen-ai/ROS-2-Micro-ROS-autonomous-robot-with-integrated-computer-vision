from glob import glob
import os

from setuptools import find_packages, setup

package_name = "avs_pdbackstepingcontrol"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="avs",
    maintainer_email="avs@example.com",
    description="PD-Backstepping cmd_vel controller for lane-following robot car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pd_backstepping_controller_v3 = avs_pdbackstepingcontrol.pd_backstepping_controller_v3:main",
            'pd_backstepping_controller_v2 = avs_pdbackstepingcontrol.pd_backstepping_controller_v2:main',
            "pd_backsteping_cmdvel_node = avs_pdbackstepingcontrol.pd_backsteping_cmdvel_node:main",
        ],
    },
)
