#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import threading as th


kill_switch = False
Z = 1.0 # hover height
sleepRate = 25 # Hz


def key_capture_thread():
    global kill_switch
    input()
    kill_switch = True


def twinCircle(timeHelper, cfs, period, total_time):
    """
    Make two crazyflie drones follow a circle trajectory, 
    centering at their initial center and keeping the distance

    input: 
    - timeHelper
    - cfs: two crazyflie references (cfs[0] -> cf3, cfs[1] -> cf4)
    - period: the time duration of completing a circle
    - total_time: the duration of the entire demonstration 
    """
    global kill_switch
    num_robots = 2

    # desired positions of the quadrotors
    desiredPos = []

    # get the center of the circle
    center = np.zeros(3)
    for i in range(num_robots):
        center += cfs[i].position()
        desiredPos.append(cfs[i].position())
    center /= 2.0

    # get the radius of the circle
    radius = np.linalg.norm(cfs[0].position() - center)
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

    # angular velocity
    omega = 2 * np.pi / period

    # main loop
    start_time = timeHelper.time()
    while (not kill_switch) and (timeHelper.time() - start_time < total_time):
        time = timeHelper.time() - start_time
        # phase_shift = omega * time
        # if phase_shift >= np.pi:
        #     phase_shift -= 2*np.pi
        # elif phase_shift <= -np.pi:
        #     phase_shift += 2*np.pi

        for i in range(num_robots):
            desiredPos[i] = center + radius * np.array(
                [np.cos(omega * time + (-1)**(i)*np.pi/2), 
                 np.sin(omega * time + (-1)**(i)*np.pi/2), 
                 0])

            # cfs[i].goTo(goal=desiredPos[i], yaw=omega * time, duration=0.2)
            cfs[i].goTo(desiredPos[i], omega * time, 1/sleepRate)
        timeHelper.sleepForRate(sleepRate)


if __name__ == "__main__":
    swarm = Crazyswarm("/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/launch/bendable_cfs.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(5 + Z)

    twinCircle(timeHelper, allcfs.crazyflies, period=10, total_time=30)

    allcfs.land(targetHeight=0.05, duration=1.0+Z, groupMask=0)
