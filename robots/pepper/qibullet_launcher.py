#!/usr/bin/env python
# coding: utf-8

import rospy
from qibullet import PepperVirtual
from qibullet import PepperRosWrapper
from qibullet import SimulationManager


def main():
    """
    Creates a virtual Pepper in the qiBullet simulation, and launches a ROS
    wrapper on top of it. Please note that this script is an example, and can
    be modified to launch a simulation that suits your needs (subscribing to
    another camera, changing the environment, running in headless mode, etc.).
    More details can be found in the qiBullet repo and in the qiBullet wiki:
    * https://github.com/softbankrobotics-research/qibullet
    * https://github.com/softbankrobotics-research/qibullet/wiki

    Please note that in order for this script to work, qiBullet needs to be
    installed, along with ROS and the essential ROS packages for the Pepper
    robot (see the provided README)
    """
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    wrapper = PepperRosWrapper()

    wrapper.launchWrapper(pepper, "/naoqi_driver")
    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass
    finally:
        wrapper.stopWrapper()
        simulation_manager.stopSimulation(client)


if __name__ == "__main__":
    main()
