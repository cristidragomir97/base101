from setuptools import setup
import os
from glob import glob

package_name = 'base101_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scenes'), glob('scenes/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'mujoco'],
    zip_safe=True,
    maintainer='Cristian Dragomir',
    maintainer_email='cristidragomir97@gmail.com',
    description='MuJoCo simulation for the base101 mobile base.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_bridge = base101_mujoco.lidar_bridge:main',
        ],
    },
)
