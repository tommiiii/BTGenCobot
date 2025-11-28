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
    description='Vision service nodes for BTGenCobot (Florence-2 + SAM)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'florence2_sam_service = vision_services.florence2_sam_service:main',
        ],
    },
)
