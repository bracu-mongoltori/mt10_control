from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mt10_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mahir",
    maintainer_email="mahir@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diff_drive = mt10_control.diff_drive:main",
            "autonomous = mt10_control.autonomous_v1:main",
            "gps_pub = mt10_control.gps_pub:main",
            "joy_node = mt10_control.joy_node:main",
            "joy_jesan= mt10_control.getVelocity:main",
            "autonomous_v2= mt10_control.autonomous_v2:main",
            "aruco_tracker = mt10_control.aruco_follower:main",
            "dummy = mt10_control.dummy_sbg:main",
            "ar_fixed = mt10_control.ar_fixed:main",
        ],
    },
)
