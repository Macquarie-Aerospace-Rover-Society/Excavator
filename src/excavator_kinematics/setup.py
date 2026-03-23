from glob import glob
from setuptools import setup

package_name = "excavator_kinematics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MARS",
    maintainer_email="mars@mq.edu.au",
    description="ROS 2 Humble kinematics and telemetry node for the Excavator bucket mechanism.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "excavator_state_node = excavator_kinematics.excavator_state_node:main",
            "excavator_visualization_node = excavator_kinematics.excavator_visualization_node:main",
            "excavator_keyboard_node = excavator_kinematics.excavator_keyboard_node:main",
        ],
    },
)
