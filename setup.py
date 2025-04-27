#!/usr/bin/env python3
"""
Setup script for the MultiTrack package.
"""
from setuptools import setup, find_packages

setup(
    name="multitrack",
    version="0.1.0",
    description="Unicycle model simulation with Kalman filter and MPPI controller",
    author="MultiTrack Team",
    packages=find_packages(),
    install_requires=[
        "pygame",
        "numpy",
        "matplotlib",
        "scipy",
        "torch",
    ],
    python_requires=">=3.6",
)