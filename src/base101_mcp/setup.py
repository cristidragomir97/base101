from setuptools import setup
import os
from glob import glob

package_name = 'base101_mcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'fastmcp'],
    zip_safe=True,
    maintainer='Cristian Dragomir',
    maintainer_email='cristidragomir97@gmail.com',
    description='MCP server for base101 robot - exposes ROS2 interfaces to LLMs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp_server = base101_mcp.server:main',
        ],
    },
)
