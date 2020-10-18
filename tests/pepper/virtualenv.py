#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import signal
from qibullet import SimulationManager
from qibullet import PepperRosWrapper
from qibullet import PepperVirtual
from qibullet import Camera


simulation_manager = None
client = None
pepper = None
wrapper = None


def kill_virtualenv(sig, frame):
    global simulation_manager, client, wrapper
    wrapper.stopWrapper()
    simulation_manager.stopSimulation(client)
    sys.exit(0)


def main():
    global simulation_manager, client, pepper, wrapper

    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=False)
    pepper = simulation_manager.spawnPepper(
        client,
        spawn_ground_plane=True)

    wrapper = PepperRosWrapper()
    wrapper.launchWrapper(pepper, "/naoqi_driver")

    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, Camera.K_QVGA)
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM, Camera.K_QVGA)
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_DEPTH, Camera.K_QVGA)

    signal.signal(signal.SIGINT, kill_virtualenv)
    signal.pause()


if __name__ == "__main__":
    main()
