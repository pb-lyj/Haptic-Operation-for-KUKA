from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'haptic'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'ruamel.yaml',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='lyj',
    maintainer_email='lyj6626076@gmail.com',
    description='ROS2 package for Tac3D sensor and data recording',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tac3d_r = haptic.tac3d_r:main',
            'tac3d_l = haptic.tac3d_l:main',
            'reset = haptic.reset:main',
            'dataset_recorder = haptic.dataset_recorder_improved:main',
            'dataset_recorder_launch = haptic.dataset_recorder_launch:main',
        ],
    },
)

