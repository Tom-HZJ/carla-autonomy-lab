from glob import glob
from setuptools import setup

package_name = "carla_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tom",
    maintainer_email="tom@localhost",
    description="CARLA autonomy lab",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "carla_bridge = carla_autonomy.carla_bridge_node:main",
            "carla_control = carla_autonomy.simple_control_node:main",
            "pure_pursuit = carla_autonomy.pure_pursuit_node:main",
            "mpc_controller = carla_autonomy.mpc_controller:main",
            "obstacle_detector = carla_autonomy.obstacle_detector:main",
            "behavior_driver = carla_autonomy.behavior_driver:main",
            "yolo_detector = carla_autonomy.yolo_detector_node:main",
            "st_planner = carla_autonomy.st_planner_node:main",
            "make_route = carla_autonomy.make_route:main",
        ],
    },
)
