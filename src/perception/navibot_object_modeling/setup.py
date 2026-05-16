from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navibot_object_modeling'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'scipy'],
    zip_safe=True,
    maintainer='Wang Junhui',
    maintainer_email='wjh_9696@163.com',
    description='Object modeling using vision-based detection masks and 3D point cloud processing',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_modeling = navibot_object_modeling.object_modeling_node:main',
        ],
    },
)
