from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_f110'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "world/"), glob('world/*.sdf')),
        (os.path.join('share', package_name, "model/"), glob('model/*.sdf')),
        (os.path.join('share', package_name, "model/"), glob('model/*.dae')),
        (os.path.join('share', package_name, "model/"), glob('model/*.config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            "ackermann_to_twist = gazebo_f110.ackermann_to_twist:main"
        ],
    },
)
