from setuptools import find_packages
from setuptools import setup

setup(
    name='robotdog_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('robotdog_msgs', 'robotdog_msgs.*')),
)
