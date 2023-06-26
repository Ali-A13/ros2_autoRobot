from setuptools import setup
import os 
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools',
    'ackermann_steering_controller'],
    zip_safe=True,
    maintainer='ac',
    maintainer_email='ac@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_exe = my_robot_package.hello_world:main',
            'circle_motion = my_robot_package.circle_motion:main',
            'obstacle_avoidance_node = my_robot_package.obstacle_avoidance:main',
            'ackermann_controller_node = my_robot_package.ackermann_controller_node:main'
        ],
    },
)
