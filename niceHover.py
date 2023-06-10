#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import os

Z = 1.0

if __name__ == "__main__":
    swarm = Crazyswarm("/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/launch/bendable_cfs.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # cfs = allcfs.crazyflies
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z, groupMask=0)
    # timeHelper.sleep(1.5+Z)
    # cfs[1].takeoff(targetHeight=Z, duration=1.0+Z, groupMask=0)
    timeHelper.sleep(1.5+Z)
    # pos = allcfs.crazyflies[0].position()
    # allcfs.crazyflies[0].goTo(goal=pos + np.array([-0.2, 0, 0]), yaw=0, duration=2)

    input("input anything to continue")

    allcfs.land(targetHeight=0.05, duration=1.0+Z, groupMask=0)
    # timeHelper.sleep(1.0+Z)
    # cfs[1].land(targetHeight=0.05, duration=1.0+Z, groupMask=0)
    timeHelper.sleep(1.0+Z)
