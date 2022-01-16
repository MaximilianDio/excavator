from setuptools import setup
from glob import glob
import os

package_name = "excavator_working_arm"
submodules = "excavator_working_arm/submodules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join("share", package_name), glob("urdf/*")),
        (os.path.join("share", package_name), glob("meshes/*.dae")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mdio",
    maintainer_email="maximilian.dio@googlemail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "excavator_ui_to_joint_vel = excavator_working_arm.excavator_ui_to_joint_vel:main",
            "excavator_joint_state_publisher = excavator_working_arm.excavator_joint_state_publisher:main",
        ],
    },
)
