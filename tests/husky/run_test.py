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

flags.DEFINE_string('out_dir', None, '')
flags.DEFINE_bool('nobrowser', True, 'if True, disable showing the coverage '
                                      'result (if exist) in a web browser')
flags.DEFINE_bool('test_real', False, 'if True, run only subset of tests for real robot')

FNULL = open(os.devnull, 'w')


def launch_gazebo(args, wait_time=12):
    print('Launching Gazebo ...')
    args = args.split()
    p = Popen(['roslaunch', 'husky_gazebo', 'husky_empty_world.launch'] + args,
              stdin=PIPE, stdout=FNULL, stderr=STDOUT)
    time.sleep(wait_time)
    return p


def run_test(testing_cmds, html_file=None, show_in_browser=True):
    if FLAGS.out_dir is not None:
        html_file = os.path.join(FLAGS.out_dir, html_file)
    for test_file in testing_cmds:
        cmd = ['pytest', '--cov=pyrobot', '-v']
        if html_file is not None:
            cmd += ['--html={:s}'.format(html_file), '--self-contained-html']
        cmd += ['--cov-append']
        t1 = Popen(cmd + test_file.split())
        t1.wait()
    if show_in_browser and not FLAGS.nobrowser:
        webbrowser.open(html_file)


def gen_html_anno(show_in_browser=True):
    if FLAGS.out_dir is not None:
        cov_dir = os.path.join(FLAGS.out_dir, 'coverage')
        p = Popen(['coverage', 'html', '-d', cov_dir])
        p.wait()
        if show_in_browser and not FLAGS.nobrowser:
            webbrowser.open(os.path.join(cov_dir, 'index.html'))
        print('Coverage report generation done!')


def exit_gazebo(gp):
    gp.terminate()
    print('Exiting Gazebo...')
    gp.wait()
    # # `rosnode cleanup` will give error: ERROR: Unable to communicate with master!
    # # if the gazebo is already shutdown correctly
    # # so this error is expected!
    p = Popen(['rosnode', 'cleanup'])
    p.wait()
    print('Gazebo exit successfully!')


def main(_):
    # # delete old coverage reports
    # # all the coverage reports generated below
    # # will be appended
    cov_file = '.coverage'
    if os.path.exists(cov_file):
        os.remove(cov_file)
    if FLAGS.out_dir is not None:
        if os.path.exists(FLAGS.out_dir):
            shutil.rmtree(FLAGS.out_dir)
        os.makedirs(FLAGS.out_dir)
###Test on simulation
    p = Popen(['rosnode', 'cleanup'])
    p.wait()

    # # Tests that do not need gazebo
    args = 'base:=husky use_rviz:=false use_sim:=true'
    args = args.split()
    husky_p = Popen(['roslaunch', 'husky_gazebo', 'husky_empty_world.launch'] + args)
    test_cmds = ['test_make_robot.py test_base_velocity_control.py test_base_controllers.py' ' --botname husky']
    run_test(test_cmds, 'basics.html')
    exit_gazebo(husky_p)



    # # Write put coverage results.
    gen_html_anno()


if __name__ == '__main__':
    app.run(main)
