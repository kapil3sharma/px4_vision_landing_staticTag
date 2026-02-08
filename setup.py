from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'px4_vision_landing_staticTag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config',
            glob(os.path.join('config', '*.yaml'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='priyam22',
    maintainer_email='u1999097@campus.udg.edu',
    description='AprilTag-based precision landing for PX4 using vision feedback',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'landing_node = px4_vision_landing_staticTag.landing_node:main',
            'offboard_experiment_manager = px4_vision_landing_staticTag.offboard_experiment_manager:main',
            'apriltag_relative_pose = px4_vision_landing_staticTag.apriltag_relative_pose:main',
            'tf_state_logger = px4_vision_landing_staticTag.tf_state_logger:main',
            'tag_rviz_markers = px4_vision_landing_staticTag.tag_rviz_markers:main',
            'tag_pose_kf = px4_vision_landing_staticTag.tag_pose_kf:main',
            'vision_guidance_controller = px4_vision_landing_staticTag.vision_guidance_controller:main',
        ],
    },
)
