from glob import glob
from setuptools import find_packages, setup

package_name = "spike_workshop_instrument"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config/patterns", glob("config/patterns/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workshop Team",
    maintainer_email="workshop@example.com",
    description="Canned instrument behaviors triggered from ROS topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "instrument_node = spike_workshop_instrument.instrument_node:main",
        ],
    },
)
