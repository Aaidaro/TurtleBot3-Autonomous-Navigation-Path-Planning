from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files in the install space
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include Gazebo worlds in the install space
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Include static maps in the install space
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahdi',
    maintainer_email='mahdielmi20001411@gmail.com',
    description='TurtleBot3 path planning project with A*, RRT*, and APF reactive control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'pure_pursuit_tracker = tb3_path_planning.pure_pursuit:main',
        'global_planner = tb3_path_planning.global_planner:main',
        'astar_planner = tb3_path_planning.global_planner:main_astar',
        'rrt_star_planner = tb3_path_planning.global_planner:main_rrt_star',
        'apf_reactive_controller = tb3_path_planning.apf_reactive_controller:main',
        ],
    },
)
