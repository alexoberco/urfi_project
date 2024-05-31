from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'starter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexoberco',
    maintainer_email='alejandro.bermudez.fajardo@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urfi_starter = starter.urfi_starter:main',
            'cmd_vel_replicator = starter.cmd_vel_replicator:main',
            'odom_replicator = starter.odom_replicator:main'
        ],
    },
)
