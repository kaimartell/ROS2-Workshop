from setuptools import find_packages, setup

package_name = "spike_workshop_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Workshop Team",
    maintainer_email="workshop@example.com",
    description="ROS 2 host-agent bridge for Spike workshop.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spike_hw_client_node = spike_workshop_hw.hw_client_node:main",
        ],
    },
)
