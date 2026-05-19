from setuptools import setup
import os
from glob import glob

package_name = 'base101_isaac'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristian Dragomir',
    maintainer_email='cristidragomir97@gmail.com',
    description='Isaac Sim integration for the base101 mobile base.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The runner is NOT an entry_point. It must be executed inside
            # the Isaac Sim Python environment (which bootstraps Kit), so the
            # launch file spawns it as a subprocess via the `python` from
            # whatever isaacsim install is active.
        ],
    },
)
