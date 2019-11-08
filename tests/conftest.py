# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import logging

def pytest_addoption(parser):
    parser.addoption('--botname', metavar='botname', default='locobot', help="which robot to test")
    logging.info(parser)
