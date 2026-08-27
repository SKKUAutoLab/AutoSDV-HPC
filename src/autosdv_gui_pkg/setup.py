import os

from setuptools import find_packages, setup


package_name = 'autosdv_gui_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'),
         ['config/nodes.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoSDV Team',
    maintainer_email='autolab@example.com',
    description='Integrated HPC control GUI and safety supervisor for AutoSDV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'autosdv_gui = autosdv_gui_pkg.main:main',
            'safety_supervisor_node = autosdv_gui_pkg.safety_supervisor_node:main',
        ],
    },
)
