#!/bin/bash

set -euxo pipefail

apt-get update

rm -rf /var/lib/apt/lists/*

# install pytorch
pip install torch==1.11.0+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html
