#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Client script for keyboard teleoperation
"""
from locobot_control.teleoperation import KeyboardTeleoperationClient

if __name__ == "__main__":
    client = KeyboardTeleoperationClient()
    client.run()
