import os
from glob import glob

from setuptools import find_packages, setup


package_name = "tianbot_drift_test"


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md", "LICENSE"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tianbot",
    maintainer_email="tianbot@todo.todo",
    description=(
        "MATLAB-derived independent 4WID drift controller for the "
        "Tianbot DYN chassis."
    ),
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "drift_4wid_speed_yaw_pid = "
            "tianbot_drift_test.drift_4wid_speed_yaw_pid:main",
        ],
    },
)
