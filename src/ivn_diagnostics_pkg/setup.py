import os

from setuptools import setup


package_name = 'ivn_diagnostics_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         ['launch/ivn_diagnostics.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoSDV Team',
    maintainer_email='autolab@example.com',
    description='Continuous and on-demand IVN diagnostics for AutoSDV.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'IVN_diagnostics_node = '
            'ivn_diagnostics_pkg.ivn_diagnostics_node:main',
        ],
    },
)
