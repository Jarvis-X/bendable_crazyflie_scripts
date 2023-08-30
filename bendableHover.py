#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import threading as th
from numpy.polynomial.polynomial import Polynomial


kill_switch = False
Z = 1.0  # hover height
L = 1.0  # rod length
K = 0.5  # initial guess of linear elasticity
window_size = 10  # the number of data points for polynomial fit
lam = 0.5  # between 0 and 1, how much we want to rely on the polynomial fit
eta = 0.5  # additional linear gain for positional error to compensate 
           # for the linear elasticity estimation
D = 3  # degree of polynomial fit
sleepRate = 5  # Hz
rod_mass = 0.013
g = 9.81


def key_capture_thread():
    global kill_switch
    k = input()
    kill_switch = True
    if k == "k":
        allcfs.emergency()



def multi_dim_polynomial_fit(x, y, degree):
    """
    Fit a polynomial to each dimension of multi-dimensional data.

    Parameters:
        data (ndarray): Multi-dimensional data.
        degrees (int): polynomial degrees for each dimension.

    Returns:
        list: List of Polynomial objects for each dimension.
    """
    var = np.array(x)
    exp = np.array(y)
    num_dimensions = var.shape[1]
    poly_list = []

    for dim in range(num_dimensions):
        poly = Polynomial.fit(var[:, dim], exp[:, dim], degree, [-1, 0])
        poly_list.append(poly)

    return poly_list


Delta = []
Force = []
def adaptive_polyfit(delta_p, err):
    """
    adaptive polynomial fit for the desired additional force to compensate
    for the elastic force from the bendable rod 

    input: 
    - delta_p: the multiplication of the displacement of the bendable rod and
               the direction from one end of the rod to the other
    - err: positional error of the crazyflie to make sure the polynomial fit 
           is aware of the PID of the robots
    """
    if len(Delta) < D + 1:
        force = K * delta_p
    else:
        poly_list = multi_dim_polynomial_fit(Delta + [np.array([0, 0, 0])], Force + [np.array([0, 0, 0])], D)
        force = np.array([lam * poly_list[i](delta_p[i]) for i in range(3)]) + (1 - lam)*eta*err
    
    Delta.append(delta_p)
    Force.append(force)
    if len(Delta) > window_size:
        Delta.pop(0)
        Force.pop(0)
    
    return force

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

    # make the robots hover at their initial positions
    for i in range(num_robots):
        # cfs[i].goTo(goal=desiredPos[i], yaw=omega * time, duration=0.2)
        cfs[i].goTo(cfs[i].position(), 0, 2.0)
    
    timeHelper.sleep(3)

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
    # Multiplier of the radius
    MULT = 0.6
    OFFSET = -0.35
    th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

    # angular velocity
    omega = 2 * np.pi / period

    # main loop
    start_time = timeHelper.time()
    while (not kill_switch) and (timeHelper.time() - start_time < total_time):
        time = timeHelper.time() - start_time
        # # robot 1 (cf3)
        # desiredPos[0] = center + radius * np.array([
        #     np.cos(omega * time + np.pi/2), 
        #     np.sin(omega * time + np.pi/2), 
        #     0])
        
        # # robot 2 (cf4)
        # desiredPos[1] = center + radius * np.array([
        #     np.cos(omega * time - np.pi/2), 
        #     np.sin(omega * time - np.pi/2), 
        #     0])
        
        # for i in range(num_robots):
        #     cfs[i].goTo(desiredPos[i], omega * time, 1/sleepRate)

        # find position difference between the two drones
        p = cfs[1].position() - cfs[0].position()
        delta = np.linalg.norm(p) - L
        p_hat = p/np.linalg.norm(p)
        delta_p = delta*p_hat

        delta_z = 0.5 * p[2]
        r = 0.5 * np.linalg.norm(p)
        theta = np.arccos(delta_z/r)

        ms = [rod_mass * (np.pi - 2*theta)/(2 * np.pi), rod_mass * (np.pi + 2*theta)/(2 * np.pi)]

        multiplier = 1.0 + MULT*np.sin(omega * time) + OFFSET
        for i in range(num_robots):
            desiredPos[i] = center + multiplier * radius * np.array(
                [np.cos(omega * time + (-1)**(i)*np.pi/2), 
                 np.sin(omega * time + (-1)**(i)*np.pi/2), 
                 0])

            cfs[i].goTo(desiredPos[i], omega * time, duration=1.5) # duration needs to be sufficiently high
            f_e = ms[i]*g*np.array([0, 0, -1]) + (-1)**(i)*adaptive_polyfit(delta_p, desiredPos[0] - cfs[0].position())
            cfs[i].setParams({
                "ctrlMel/fx_e": float(f_e[0]),
                "ctrlMel/fy_e": float(f_e[1]),
                "ctrlMel/fz_e": float(f_e[2]),
            })

        timeHelper.sleepForRate(sleepRate)


def reset_fe(allcfs):
    """
    Reset the elastic force compensation

    input: 
    - allcfs: the group for all crazyflies
    """
    for crazyflie in allcfs.crazyflies:
        crazyflie.setParams({
            "ctrlMel/fx_e": float(0.0),
            "ctrlMel/fy_e": float(0.0),
            "ctrlMel/fz_e": float(0.0),
        })

if __name__ == "__main__":
    swarm = Crazyswarm("/home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/launch/bendable_cfs.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=3.0+Z)
    timeHelper.sleep(2 + Z)

    twinCircle(timeHelper, allcfs.crazyflies, period=20, total_time=40)
    reset_fe(allcfs)
    
    allcfs.land(targetHeight=0.05, duration=1.0+Z, groupMask=0)
