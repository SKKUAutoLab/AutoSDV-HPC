from setuptools import find_packages
from setuptools import setup

setup(
    name='ota_update_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('ota_update_interfaces', 'ota_update_interfaces.*')),
)
