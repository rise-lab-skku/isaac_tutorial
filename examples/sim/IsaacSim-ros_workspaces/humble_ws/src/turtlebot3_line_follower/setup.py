from setuptools import setup
from glob import glob
import os

package_name = 'turtlebot3_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/assets", glob("assets/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeongmin Jeon',
    maintainer_email='nicky707@skku.edu',
    description='ROS2 Line follower package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = turtlebot3_line_follower.follower_node:main',            
            'test_node = turtlebot3_line_follower.test_topic:main'
        ],
    },
)
