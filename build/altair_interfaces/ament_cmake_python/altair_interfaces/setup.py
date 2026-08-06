from setuptools import find_packages
from setuptools import setup

setup(
    name='altair_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('altair_interfaces', 'altair_interfaces.*')),
)
