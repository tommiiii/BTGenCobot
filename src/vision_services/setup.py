from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_services'

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
    maintainer='BTGenCobot Team',
    maintainer_email='noreply@example.com',
    description='Vision service nodes for BTGenCobot (Grounding DINO object detection)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grounding_dino_service = vision_services.grounding_dino_service:main',
        ],
    },
)
