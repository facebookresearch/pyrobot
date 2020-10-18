#!/usr/bin/env python

import os
import shutil
import time
import signal
import webbrowser
from subprocess import Popen, PIPE, STDOUT

from absl import flags, app

FLAGS = flags.FLAGS

flags.DEFINE_string('out_dir', None, '')
flags.DEFINE_bool(
    'nobrowser',
    False,
    'if True, disable showing the coverage result (if exist) in a web browser')

flags.DEFINE_bool(
    'test_real',
    False,
    'if True, run only subset of tests for real robot')

FNULL = open(os.devnull, 'w')


def start_roscore(wait_time=3):
    """
    Method running a roscore. A roscore is required to be able to start the
    qibullet ros wrapper
    """
    print('Launching roscore...')
    process = Popen(['roscore'], stdin=PIPE, stdout=FNULL, stderr=STDOUT)
    time.sleep(wait_time)
    return process


def start_virtualenv(wait_time=3):
    """
    Starting a virtual environment for Pepper: a qibullet simulation and a ROS
    wrapper. Python 2 is used to launch that script
    """
    print('Launching virtual environment...')
    folder = os.path.dirname(os.path.realpath(__file__))
    virtualenv_script = os.path.join(folder, "virtualenv.py")

    print(virtualenv_script)
    process = Popen(
        ["python2", "virtualenv.py"],
        stdin=PIPE,
        stdout=FNULL,
        stderr=STDOUT)

    time.sleep(wait_time)
    return process


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


def stop_roscore(process):
    """
    Method stopping a previously launched roscore process
    """
    process.terminate()
    print("Stopping the roscore...")
    process.wait()
    p = Popen(['rosnode', 'cleanup'])
    p.wait()
    print('Rosnode successfully stopped!')


def stop_virtualenv(process):
    """
    Stops the virtual environment
    """
    os.kill(process.pid, signal.SIGINT)
    process.wait()
    print('Stopping virtual environment...')


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

    # Tests that do not need a robot (simulated or real)
    test_cmds = ['test_pyrobot_classes.py']
    run_test(test_cmds, 'basic.html')

    if FLAGS.test_real:
        # TODO: Assume real robot launch file has already been started
        # - TODO maybe check these nodes have been launched programmatically?
        print(
            'Make sure real robot has already been launched (replace your_ip '
            'by the ip of the robot and network_interface by your interface, '
            'eth0, wlan1, ...): roslaunch naoqi_driver naoqi_driver.launch '
            'nao_ip:=your_ip network_interface:=your_interface')

        # TODO: Need to add a simple base velocity control test as well
        # test_cmds = ['test_camera.py test_arm_controls.py']
        test_cmds = [
            'test_base_pepper.py test_gripper_pepper.py test_camera_pepper.py'
            'test_arm_pepper.py --virtualenv 0']
        run_test(test_cmds, 'pepper_real.html')

    else:
        p = Popen(['rosnode', 'cleanup'])
        p.wait()

        # Assume that the naoqi driver workspace is sourced
        # TODO: maybe check that programmatically?

        # Launching a roscore
        roscore_process = start_roscore()
        virtualenv_process = start_virtualenv()

        test_cmds = [
            'test_base_pepper.py test_gripper_pepper.py test_camera_pepper.py '
            'test_arm_pepper.py --virtualenv 1']

        run_test(test_cmds, 'pepper_simulated.html')

        stop_virtualenv(virtualenv_process)
        stop_roscore(roscore_process)

        # # Write put coverage results.
        gen_html_anno()


if __name__ == '__main__':
    app.run(main)
