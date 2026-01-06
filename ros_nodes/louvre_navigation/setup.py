from setuptools import setup
import os
from glob import glob

package_name = 'louvre_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@example.com',
    description='Navigation package for Louvre robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'laser_scan_merger = louvre_navigation.laser_scan_merger:main',
            'depth_republisher = louvre_navigation.depth_republisher:main',
        ],
    },
)
