from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'move_robots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'move_robots'), glob('move_robots/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lluis',
    maintainer_email='lluis.prior@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velControlledRobot = move_robots.velControlledRobot:main',
            'velControlledDrone = move_robots.velControlledDrone:main',
            'line = move_robots.line:main',
            'robot_follower = move_robots.robot_follower:main',
        ],
    },
)
