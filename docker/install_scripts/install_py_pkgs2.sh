#!/bin/bash

set -euxo pipefail

pip install Pillow opencv-python open3d scikit-learn absl-py pytest-cov
pip install pytest-html yacs guzzle-sphinx-theme mock pyassimp pytz bezier