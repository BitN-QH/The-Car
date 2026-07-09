from glob import glob
from setuptools import find_packages, setup

package_name = "rdk_transport_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@example.com",
    description="Mock base bridge for the RDK X5 transport robot bringup.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "base_bridge_node = rdk_transport_base.base_bridge_node:main",
            "manual_command_node = rdk_transport_base.manual_command_node:main",
            "segmented_nav_node = rdk_transport_base.segmented_nav_node:main",
            "target_pose_node = rdk_transport_base.target_pose_node:main",
            "demo_mission_node = rdk_transport_base.demo_mission_node:main",
            "mission_manager_node = rdk_transport_base.mission_manager_node:main",
            "person_follow_mission_node = rdk_transport_base.person_follow_mission_node:main",
            "yolo_web_monitor_node = rdk_transport_base.yolo_web_monitor_node:main",
            "h30_imu_odom_node = rdk_transport_base.h30_imu_odom_node:main",
            "scan_sanitizer_node = rdk_transport_base.scan_sanitizer_node:main",
        ],
    },
)
