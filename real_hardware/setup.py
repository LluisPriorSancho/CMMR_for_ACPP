from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'real_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'real_hardware'), glob('real_hardware/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab',
    maintainer_email='lluis.3605@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                        'decawave = real_hardware.decawave:main',
                        'leo_control = real_hardware.leo_control:main', 
                        'system_control_tello = real_hardware.system_control_tello:main',
                        'system_control_leo = real_hardware.system_control_leo:main',
                        'PID = real_hardware.PID:main',
                        'demo = real_hardware.demo:main',
                        'simulation = real_hardware.simulation:main'
        ],
    },
)
