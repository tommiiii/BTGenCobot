from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bt_generator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 node for LLM-based behavior tree generation using Llama 3.2-1B',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_generator_node = bt_generator.bt_generator_node:main',
        ],
    },
)
