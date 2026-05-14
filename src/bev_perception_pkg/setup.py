from glob import glob

from setuptools import find_packages, setup

package_name = 'bev_perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SKKU AutoSDV',
    maintainer_email='autosdv@example.com',
    description='BEV/SVM perception package for AutoSDV.',
    license='GPL-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_node = bev_perception_pkg.bev_node:main',
            'svm_compositor_node = bev_perception_pkg.svm_compositor_node:main',
        ],
    },
)
