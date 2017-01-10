#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nextage_moveit_config'],
    package_dir={'': 'src'}
)

setup(**d)
