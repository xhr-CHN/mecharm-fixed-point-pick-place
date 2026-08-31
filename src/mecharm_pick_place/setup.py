from glob import glob
from setuptools import find_packages, setup


package_name = "mecharm_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xhr",
    maintainer_email="xhr@example.com",
    description="Fixed-point pick-and-place task for mechArm 270 Pi.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_place_task_node = mecharm_pick_place.task_node:main",
            "pick_place_recorder_node = mecharm_pick_place.recorder_node:main",
        ],
    },
)
