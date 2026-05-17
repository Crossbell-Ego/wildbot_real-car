from setuptools import setup

package_name = "lidar_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/self_filter.yaml"]),
        ("share/" + package_name + "/launch", ["launch/self_filter_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="You",
    maintainer_email="you@example.com",
    description="LaserScan cleanup and self-mask filter for Wildbot LiDAR.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_nan_value_filter_node = lidar_pkg.lidar_nan_value_filter_node:main",
        ],
    },
)
