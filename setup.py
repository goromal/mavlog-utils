#!/usr/bin/env python

from setuptools import setup, find_packages

# package configuration - for reference see:
# https://setuptools.readthedocs.io/en/latest/setuptools.html#id9
setup(
    name="mavlog_utils",
    description="Assorted tools for processing mavlink .bin logs",
    version="0.0.0",
    author="goromal",
    author_email="andrew.torgesen@gmail.com",
    packages=find_packages(),
    include_package_data=True,
    entry_points={"console_scripts":[
        "mav-dump-mission=mavlog_utils.dump_mission:main"
    ]},
    install_requires = [
        "pymavlink",
        "progressbar2"
    ]
)
