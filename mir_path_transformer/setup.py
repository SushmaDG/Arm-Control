#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['mir_path_transformer_ros'],
   package_dir={'mir_path_transformer_ros': 'ros/src/mir_path_transformer_ros'}
)

setup(**d)