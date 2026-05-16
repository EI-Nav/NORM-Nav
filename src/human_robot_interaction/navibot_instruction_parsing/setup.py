from setuptools import find_packages, setup

package_name = 'navibot_instruction_parsing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/instruction_parsing.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/instruction_parsing_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rclpy_interfaces',
        'std_msgs',
        'geometry_msgs',
        'sensor_msgs',
        'nav2_msgs',
        'navibot_interfaces',
        'openai',
        'pydantic',
    ],
    zip_safe=True,
    maintainer='Wang Junhui',
    maintainer_email='wjh_9696@163.com',
    description='Behavioral instruction parsing module for natural language navigation commands',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'instruction_parsing_node = navibot_instruction_parsing.instruction_parsing_node:main',
            'instruction_publisher_node = navibot_instruction_parsing.instruction_publisher_node:main',
        ],
    },
)
