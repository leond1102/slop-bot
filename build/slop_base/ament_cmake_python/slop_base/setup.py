from setuptools import find_packages
from setuptools import setup

setup(
    name='slop_base',
    version='0.1.0',
    packages=find_packages(
        include=('slop_base', 'slop_base.*')),
)
