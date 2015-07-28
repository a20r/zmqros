#!/usr/bin/env python2
from setuptools import setup
from setuptools import find_packages


setup(
    name="zmqros",
    version="0.1",
    description="A robotic middleware for swarms",
    author="Alex Wallar",
    author_email="wallar@mit.edu",
    packages=find_packages(),
    install_requires=[
        "pyzmq",
        "flask",
        "flask-classy"
    ]
)
