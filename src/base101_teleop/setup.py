from setuptools import find_packages, setup

package_name = 'base101_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='Single-page web teleop for the base101 dual-arm robot.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'server = base101_teleop.server:main',
        ],
    },
)
