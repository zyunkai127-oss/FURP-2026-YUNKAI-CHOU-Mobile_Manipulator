from glob import glob
from setuptools import find_packages, setup


package_name = "eye_to_hand_grasp_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yunkai Chou",
    maintainer_email="yunkai@example.com",
    description="Eye-to-hand TF, IK validation, and Panda grasp demo for MoveIt 2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "grasp_demo = eye_to_hand_grasp_demo.grasp_demo:main",
        ],
    },
)
