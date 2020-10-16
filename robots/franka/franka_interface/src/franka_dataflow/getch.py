#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import termios
import tty
from select import select


def getch(timeout=0.01):
    """
    Retrieves a character from stdin.

    Returns None if no character is available within the timeout.
    Blocks if timeout < 0.
    """
    # If this is being piped to, ignore non-blocking functionality
    if not sys.stdin.isatty():
        return sys.stdin.read(1)
    fileno = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fileno)
    ch = None
    try:
        tty.setraw(fileno)
        rlist = [fileno]
        if timeout >= 0:
            [rlist, _, _] = select(rlist, [], [], timeout)
        if fileno in rlist:
            ch = sys.stdin.read(1)
    except Exception as ex:
        print "getch", ex
        raise OSError
    finally:
        termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
    return ch
