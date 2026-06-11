from setuptools import find_packages, setup

package_name = 'robocore_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristi Dragomir',
    maintainer_email='cristidragomir97@gmail.com',
    description='robocore agent: robot-agnostic JSON-RPC bridge node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = robocore_agent.main:main',
        ],
    },
)
