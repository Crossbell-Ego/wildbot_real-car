from setuptools import setup
import os

package_name = 'arm_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Inverse kinematics example for a planar 2-link arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ik_node = arm_ik.ik_node:main'
        ],
    },
)
