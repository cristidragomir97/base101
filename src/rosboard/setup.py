from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rosboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'rosboard': [
            'html/*',
            'html/css/*',
            'html/css/images/*',
            'html/fonts/*',
            'html/js/*',
            'html/js/transports/*',
            'html/js/viewers/*',
            'html/js/viewers/meta/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='rosboard (reconstructed from install/)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosboard_node = rosboard.rosboard:main',
        ],
    },
)
