#!/bin/bash

set -euxo pipefail

update-alternatives --install /usr/bin/python python /usr/bin/python3.8 10
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py
rm get-pip.py
pip install setuptools==59.5.0