# /***************************************************************************
# Modified from: Saif Sidhik franka_interface
# 
# @package: PyRobot
# @author: Zisu Dong <jdong1021@fb.com>
# 
# **************************************************************************/


import errno

import rospy


def wait_for(test, timeout=1.0, raise_on_error=True, rate=100,
             timeout_msg="timeout expired", body=None):
    """
    Waits until some condition evaluates to true.

    @param test: zero param function to be evaluated
    @param timeout: max amount of time to wait. negative/inf for indefinitely
    @param raise_on_error: raise or just return False
    @param rate: the rate at which to check
    @param timout_msg: message to supply to the timeout exception
    @param body: optional function to execute while waiting
    """
    end_time = rospy.get_time() + timeout
    rate = rospy.Rate(rate)
    notimeout = (timeout < 0.0) or timeout == float("inf")
    while not test():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS Shutdown")
            return False
        elif (not notimeout) and (rospy.get_time() >= end_time):
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            rospy.loginfo(timeout_msg)
            return False
        if callable(body):
            body()
        rate.sleep()
    return True
