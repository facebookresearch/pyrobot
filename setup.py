# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os

from setuptools import setup, find_packages

dir_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(dir_path, "src", "pyrobot", "version.py")) as fp:
    exec(fp.read())


def read_requirements_file(filename):
    req_file_path = "%s/%s" % (dir_path, filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]


packages = find_packages("src")
# Ensure that we don't pollute the global namespace.
for p in packages:
    assert p == "pyrobot" or p.startswith("pyrobot.")

import sys

if sys.version_info > (3, 0):
    # Python 3 code in this block
    requirements_file = "requirements_python3.txt"
else:
    # Python 2 code in this block
    requirements_file = "requirements_python2.txt"

setup(
    name="pyrobot",
    version=__version__,
    author="PyRobot Team",
    url="https://github.com/facebookresearch/pyrobot.git",
    license="MIT",
    packages=packages,
    package_data={"pyrobot": ["cfg/*.yaml"],},
    package_dir={"": "src"},
    install_requires=read_requirements_file(requirements_file),
)
