from setuptools import find_packages, setup

package_name = 'zumo_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_zumo.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mutasem Bader',
    maintainer_email='mutasem.bader@stud.fra-uas.de',
    description=(
        'Camera-based line-following and path-mapping for the Zumo robot '
        'using ROS 2, OpenCV, PID control, and PyQt5 GUI.'
    ),
    license='Proprietary — Copyright (c) 2026 Mutasem Bader. All Rights Reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node        = zumo_robot.camera_node_v2:main',
            'motors_node        = zumo_robot.motors_node:main',
            'encoder_node       = zumo_robot.encoder_node:main',
            'path_mapping_node  = zumo_robot.path_node:main',
            'tf2_node           = zumo_robot.tf2_node:main',
            'interface_node     = zumo_robot.interface_node:main',
            'log_node           = zumo_robot.log_node:main',
        ],
    },
)
