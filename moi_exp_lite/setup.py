from setuptools import setup
import os
from glob import glob

package_name = 'moi_exp_lite'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # only install the ament resource index and package.xml
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Moises Bolivar Vivas',
    maintainer_email='mbolvar11@gmail.com',
    description='ROS 2 launch scripts and logic for exploration and vision',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'explore_controller = moi_exp_lite.explore_controller:main',
            'detect_object = moi_exp_lite.detect_object:main',
        ],
    },
)

