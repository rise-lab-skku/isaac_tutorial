from setuptools import setup
from glob import glob
import os

package_name = 'isaac_tutorials'

setup(
    name=package_name,
    version='0.1.0',
    packages=["scripts"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/assets", glob("assets/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Sim',
    maintainer_email='isaac-sim-maintainers@nvidia.com',
    description='The isaac_tutorials package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pymoveit_example = scripts.pymoveit_example:main',
            'ros2_ackermann_publisher = scripts.ros2_ackermann_publisher:main',
            'ros2_publisher = scripts.ros2_publisher:main'
        ],
    },
)
