#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['bin/arbotix_gui', 'bin/arbotix_terminal','bin/arbotix_fk','bin/arbotix_ik_end_xyzp','bin/arbotix_ik_gxyzp','bin/arbotix_ik_position','bin/arbotix_ik_xyzp'],
    packages=['arbotix_python'],
    package_dir={'': 'src'},
    )

setup(**d)
