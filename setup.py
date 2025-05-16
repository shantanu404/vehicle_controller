import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vehicle_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*")),
        ),
        (
            os.path.join("share", package_name, "description"),
            glob(os.path.join("description", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shantanu404",
    maintainer_email="rahmanshantanu69@gmail.com",
    description="Vehicle Controller Experiment based on Monocular Vision",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_control = vehicle_controller.mpc_control_node:main",
            "odometry_plotter = vehicle_controller.odometry_plotter_node:main",
        ],
    },
)
