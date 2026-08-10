from setuptools import find_packages, setup
import os
from glob import glob

package_name = "avs_hybridcontrol"

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
    description="AVS Hybrid Backstepping + Cascade PD Controller",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hybrid_controller = avs_hybridcontrol.hybrid_controller:main",
        ],
    },
)
