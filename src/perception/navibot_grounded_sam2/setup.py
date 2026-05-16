from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navibot_grounded_sam2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'scripts']),
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
    description='Real-time object tracking using GroundingDINO and SAM2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grounded_sam2_tracker = navibot_grounded_sam2.grounded_sam2_tracker:main',
        ],
    },
)
