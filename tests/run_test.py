#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import shutil
import time
import webbrowser
from subprocess import Popen, PIPE, STDOUT

from absl import flags, app

FLAGS = flags.FLAGS

flags.DEFINE_string("out_dir", None, "")
flags.DEFINE_bool(
    "nobrowser",
    False,
    "if True, disable showing the coverage " "result (if exist) in a web browser",
)
flags.DEFINE_bool(
    "test_real", False, "if True, run only subset of tests for real robot"
)

FNULL = open(os.devnull, "w")


def launch_gazebo(args, wait_time=12):
    print("Launching Gazebo ...")
    args = args.split()
    p = Popen(
        ["roslaunch", "locobot_control", "main.launch"] + args,
        stdin=PIPE,
        stdout=FNULL,
        stderr=STDOUT,
    )
    time.sleep(wait_time)
    return p


def run_test(testing_cmds, html_file=None, show_in_browser=True):
    if FLAGS.out_dir is not None:
        html_file = os.path.join(FLAGS.out_dir, html_file)
    for test_file in testing_cmds:
        cmd = ["pytest", "--cov=pyrobot", "-v"]
        if html_file is not None:
            cmd += ["--html={:s}".format(html_file), "--self-contained-html"]
        cmd += ["--cov-append"]
        t1 = Popen(cmd + test_file.split())
        t1.wait()
    if show_in_browser and not FLAGS.nobrowser:
        webbrowser.open(html_file)


def gen_html_anno(show_in_browser=True):
    if FLAGS.out_dir is not None:
        cov_dir = os.path.join(FLAGS.out_dir, "coverage")
        p = Popen(["coverage", "html", "-d", cov_dir])
        p.wait()
        if show_in_browser and not FLAGS.nobrowser:
            webbrowser.open(os.path.join(cov_dir, "index.html"))
        print("Coverage report generation done!")


def exit_gazebo(gp):
    gp.terminate()
    print("Exiting Gazebo...")
    gp.wait()
    # # `rosnode cleanup` will give error: ERROR: Unable to communicate with master!
    # # if the gazebo is already shutdown correctly
    # # so this error is expected!
    p = Popen(["rosnode", "cleanup"])
    p.wait()
    print("Gazebo exit successfully!")


def main(_):
    # # delete old coverage reports
    # # all the coverage reports generated below
    # # will be appended
    cov_file = ".coverage"
    if os.path.exists(cov_file):
        os.remove(cov_file)
    if FLAGS.out_dir is not None:
        if os.path.exists(FLAGS.out_dir):
            shutil.rmtree(FLAGS.out_dir)
        os.makedirs(FLAGS.out_dir)

    # # Tests that do not need gazebo
    test_cmds = ["test_pyrobot_classes.py test_base_position_control_inits.py"]
    run_test(test_cmds, "basic.html")

    if FLAGS.test_real:
        # TODO: Assume real robot launch file has already been started
        # - TODO maybe check these nodes have been launched programmatically?
        print(
            "Make sure real robot has already been launched: "
            "roslaunch locobot_control main.launch use_arm:=true use_camera:=true"
        )
        # TODO: Need to add a simple base velocity control test as well
        test_cmds = ["test_camera.py test_arm_controls.py"]
        run_test(test_cmds, "real.html")

    else:
        p = Popen(["rosnode", "cleanup"])
        p.wait()

        # # Create base tests
        args = "base:=create use_rviz:=false use_sim:=true"
        create_p = launch_gazebo(args)
        test_cmds = [
            "test_make_robots.py test_arm_utils.py test_arm_controls.py "
            "test_camera.py --botname locobot_lite"
        ]
        run_test(test_cmds, "locobot-lite.html")
        exit_gazebo(create_p)

        args = "base:=create use_rviz:=false use_sim:=true"
        create_p = launch_gazebo(args)
        test_cmds = [
            "test_base_velocity_control.py test_base_controllers.py"
            " --botname locobot_lite"
        ]
        run_test(test_cmds, "locobot-lite-base.html")
        exit_gazebo(create_p)

        # Kobuki base tests. s-gupta: I had to split the following tests into
        # multiple calls to pytest, and starting gazebo multiple times, in
        # order for things to work. Ditto for create.
        args = "base:=kobuki use_rviz:=false use_sim:=true use_arm:=true"
        kobuki_p = launch_gazebo(args)
        test_cmds = [
            "test_make_robots.py test_arm_utils.py test_arm_controls.py"
            " test_camera.py --botname locobot"
        ]
        run_test(test_cmds, "locobot.html")
        exit_gazebo(kobuki_p)

        args = "base:=kobuki use_rviz:=false use_sim:=true"
        kobuki_p = launch_gazebo(args)
        test_cmds = [
            "test_base_velocity_control.py test_base_controllers.py"
            " --botname locobot"
        ]
        run_test(test_cmds, "locobot-base.html")
        exit_gazebo(kobuki_p)

        # # Write put coverage results.
        gen_html_anno()


if __name__ == "__main__":
    app.run(main)
