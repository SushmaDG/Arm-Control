#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mir_rockin_control_fbm_launcher_ros'],
    package_dir={'mir_rockin_control_fbm_launcher_ros': 'ros/src/mir_rockin_control_fbm_launcher_ros'}
)

setup(**d)
