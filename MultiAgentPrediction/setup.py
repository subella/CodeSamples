#!/usr/bin/env python3

import os
from setuptools import setup

with open('README.md', 'r') as f:
    readme = f.read()

setup(
    name='Multi Agent Prediction Final Project',
    description='Cognitive Robotics Final Project',
    long_description=readme,
    long_description_content_type='text/markdown',
    version='1.0',
    author='Sam Ubellacker, Dagmawi',
    author_email='subella@mit.edu',
    packages=['multiagent'],
    include_package_data=True,
    python_requires=">=3.6.*",
    install_requires=['numpy','scipy','matplotlib','pygame','shapely'],
)
