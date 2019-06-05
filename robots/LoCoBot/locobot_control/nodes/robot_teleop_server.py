#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Server script for robot teleoperation
"""
import signal

from locobot_control.teleoperation import RobotTeleoperationServer

if __name__ == "__main__":
    server = RobotTeleoperationServer()
    signal.signal(signal.SIGINT, server.signal_handler)
    server.run()
