from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navibot_costmap'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wang Junhui',
    maintainer_email='wjh_9696@163.com',
    description='Global costmap generation from LIO point cloud for navigation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'geometric_traversability_costmap_node = navibot_costmap.geometric_traversability_costmap_node:main',
            'semantic_traversability_costmap_node = navibot_costmap.semantic_traversability_costmap_node:main',
            'directional_constraint_costmap_node = navibot_costmap.directional_constraint_costmap_node:main',
            'costmap_fusion_node = navibot_costmap.costmap_fusion_node:main',
            'constraint_compensation_node = navibot_costmap.constraint_compensation_node:main',
            'velocity_constraint_node = navibot_costmap.velocity_constraint_node:main',
        ],
    },
)
