#!/bin/bash

set -euxo pipefail
apt-get update && apt-get install -y python3-tk libopenblas-dev && \
     rm -rf /var/lib/apt/lists/*
pip install --ignore-installed PyYAML
pip install ipython PyOpenGL
pip install --upgrade scipy
pip install numpy pyyaml scipy ipython cython matplotlib gym
pip install git+https://github.com/ROBOTIS-GIT/DynamixelSDK.git#subdirectory=python