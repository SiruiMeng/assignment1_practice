import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'odom_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siruimeng',
    maintainer_email='sxm2337@student.bham.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_node = odom_localization.odometry_node:main',
            'logger_node = odom_localization.logger_node:main',
            'encoder_sim_node = odom_localization.encoder_sim_node:main',
            'drive_route_node = odom_localization.drive_route_node:main',
        ],
    },
)
