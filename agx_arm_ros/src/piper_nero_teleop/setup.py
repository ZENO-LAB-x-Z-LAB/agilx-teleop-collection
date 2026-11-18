from glob import glob
import os

from setuptools import find_packages, setup


package_name = "piper_nero_teleop"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS2 Jazzy single-arm teleoperation for Piper Nero",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "leader_teleop_node = piper_nero_teleop.leader_teleop_node:main",
            "gravity_compensation_node = piper_nero_teleop.gravity_compensation_node:main",
            "nero_ctrl_node = piper_nero_teleop.nero_ctrl_node:main",
            "nero_gravity_node = piper_nero_teleop.nero_gravity_node:main",
            "joint_cmd_mux_node = piper_nero_teleop.joint_cmd_mux_node:main",
            "tactile_sensor_node = piper_nero_teleop.tactile_sensor_node:main",
        ],
    },
)
