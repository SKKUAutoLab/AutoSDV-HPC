"""Package the AutoSDV cloud ROS nodes."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'autosdv_cloud_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoSDV Team',
    maintainer_email='autolab@example.com',
    description='AutoSDV MQTT diagnostics gateway and WHIP camera streamer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'cloud_gateway_node = autosdv_cloud_pkg.gateway_node:main',
        'camera_streamer_node = autosdv_cloud_pkg.streamer_node:main',
    ]},
)
