#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

# --- Szilárd declarations --- #

# Reading initial positions
import yaml
import os
cwd = os.getcwd()
cwd = cwd
with open(cwd + "/bspline_static/yaml/initialPosition.yaml", "r") as file:
    initialPosition = yaml.load(file, Loader=yaml.FullLoader)
# Setting targetHeight
targetHeight = 1.3
targetHeight = np.array(initialPosition['crazyflies'][0]['initialPosition']).tolist()[-1]
# Loading trajectories
import uav_trajectory
traj1 = uav_trajectory.Trajectory()
traj1.loadcsv(cwd + "/bspline_static/csv/vehicle0.csv")
traj2 = uav_trajectory.Trajectory()
traj2.loadcsv(cwd + "/bspline_static/csv/vehicle1.csv")
traj3 = uav_trajectory.Trajectory()
traj3.loadcsv(cwd + "/bspline_static/csv/vehicle2.csv")
trajectories = [traj1, traj2, traj3]

# --- Peti declarations --- #
H = 1.4
U1, Th1, T1 = 0.5259, -42.346, 0.08219
U2, Th2, T2 = 0.3795, 297.346, 0.22265
U3, Th3, T3 = 0.17489, 0.0, 0.1209
U4, Th4, T4 = 0.3795, -297.346, 0.22193
U5, Th5, T5 = 0.50265, 59.2655, 0.05508
# Adjust flip parameters
U1 = 0.4
T1 = 0.2
Th1 = -10
T2 = np.array([0.165, 0.17, 0.18])
T3 = 0
Th2 = Th2 * 31000 / 24400
Th4 = -Th2
T4 = 0.2
U5 = U1
T5 = 0
Th5 = -Th1


def flipInitParams(cf, T1, T2, T3, T4):

    # Last working set: 0.5, 0.2, 0, 0.14, 0 ...
    #                   0.45, 0.3, -10, 0.16, 0 ... for cf5 (with better feedforward)
    #                   0.45, 0.2, -10, 0.177, 0 ... for cf3 (with better feedforward)
    #                   0.45, 0.3, -10, 0.166, 0 ... for cf2 (with better feedforward)
    # With starting momentum: 0.4, 0.2, -10, 0.18, 0 ... for cf5
    #                         0.4, 0.2, -10, 0.16, 0... for cf2
    #                         0.4, 0.2, -10, 0.175, 0... for cf3
    #                         0.4, 0.2, -10, 0.17, 0... for cf4

    # send the parameters
    cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
    cf.setParam('stabilizer/controller', 1)  # PID controller
    params = {'controller/U1': U1, 'controller/Th1': Th1, 'controller/T1': T1,
              'controller/U2': U2, 'controller/Th2': Th2, 'controller/T2': float(T2),
              'controller/U3': U3, 'controller/Th3': Th3, 'controller/T3': T3,
              'controller/U4': U4, 'controller/Th4': Th4, 'controller/T4': T4,
              'controller/U5': U5, 'controller/Th5': Th5, 'controller/T5': T5}
    cf.setParams(params)


def executeFlip(cf, T1, T2, T3, T4):

    position = cf.position()

    # average motor PWMs
    cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM correction
    cf.setParam('motorPowerSet/isAv', 0)
    cf.setParam('motorPowerSet/isFF', 1)
    timeHelper.sleep(2)

    # start to lift
    cf.goTo(np.array([position[0], position[1], initHeight + 2]), 0, 2)
    timeHelper.sleep(1)

    ## Begin flip #############################
    cf.setParams({'pid_attitude/yaw_kp': 0, 'pid_attitude/yaw_ki': 0, 'pid_attitude/yaw_kd': 0,
                  'pid_rate/yaw_kp': 0, 'pid_rate/yaw_ki': 0, 'posCtlPid/xKp': 0, 'posCtlPid/yKp': 0})
    cf.goTo(np.array([position[0], position[1], 0.6]), 0, 1)
    cf.setParam('controller/isFlipControl', 1)
    timeHelper.sleep(T1 + T2 + T3 + T4)  # wait until the maneuver is over

    # recover
    timeHelper.sleep(1)
    cf.setParams({'posCtlPid/xKp': 0.2, 'posCtlPid/yKp': 0.2})
    cf.cmdPosition(np.array([position[0], position[1], 0.6]), 0)
    cf.setParam('controller/wasFlipControl', 0)

    # start to get back to initial position
    for k in range(20):
        cf.cmdPosition(np.array([position[0], position[1], 0.6]), 0)
        timeHelper.sleepForRate(20)

    # switch on yaw control
    cf.setParams({'pid_attitude/yaw_kp': 6.0, 'pid_attitude/yaw_ki': 1.0,
                  'pid_attitude/yaw_kd': 0.35, 'pid_rate/yaw_kp': 120.0, 'pid_rate/yaw_ki': 16.7})
    timeHelper.sleep(1)
    cf.setParams({'posCtlPid/xKp': 2, 'posCtlPid/yKp': 2})

    # really get back to initial position
    for k in range(40):
        cf.cmdPosition(np.array([position[0], position[1], 0.6]), 0)
        timeHelper.sleepForRate(20)
    cf.goTo(np.array([position[0], position[1], 0.6]), 0, 1)
    timeHelper.sleep(1)


if __name__ == "__main__":
    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Szilard code here ######################################################
    # Take off
    for j, cf in enumerate(allcfs.crazyflies):
        cf.takeoff(targetHeight=targetHeight + j * 0.1 - 0.1, duration=3)
    timeHelper.sleep(3.0)
    
    # Go to initial position
    for j, cf in enumerate(allcfs.crazyflies):
        pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
        cf.goTo(pos + np.array([0, 0, j * 0.1 - 0.1]), 0, 2.0)
    timeHelper.sleep(2)

    # Upload trajectories to cf
    for j, cf in enumerate(allcfs.crazyflies):
        cf.uploadTrajectory(0, 0, trajectories[j])

    # SAFETY CHECK
    print("Starting position okay? Press button to continue...")
    swarm.input.waitUntilButtonPressed()

    # Trajectory following
    reverse = True
    allcfs.startTrajectory(0, timescale=1.0, reverse=False)
    timeHelper.sleep(traj1.duration * 1.0 + 2.0)
    if reverse == True:
        allcfs.startTrajectory(0, timescale=1.0, reverse=True)
        timeHelper.sleep(traj1.duration * 1.0 + 2.0)
        
    # Landing
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
    
    # SAFETY CHECK
    print("Can we do the flip already? :D :P Press button to continue...")
    swarm.input.waitUntilButtonPressed()
    
    
    # Peti code here ######################################################
    # Initialize flip parameters
    for i, cf in enumerate(allcfs.crazyflies):
        flipInitParams(cf, T1, T2[i], T3, T4)
    # start flying!
    numDrone = 0
    initHeight = 0.4
    allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    timeHelper.sleep(3)
    executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    allcfs.crazyflies[numDrone].goTo(np.array([0.0, 0.6, 0.6]), 0, 3)
    timeHelper.sleep(3)
    allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)

    numDrone = 1
    allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    timeHelper.sleep(3)
    executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    allcfs.crazyflies[numDrone].goTo(np.array([0.0, -0.6, 0.6]), 0, 3)
    timeHelper.sleep(3)
    allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)

    numDrone = 2
    allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    timeHelper.sleep(3)
    executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    # allcfs.crazyflies[numDrone].goTo(np.array([0.0, -0.4, 0.6]), 0, 3)
    # timeHelper.sleep(3)
    allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)

    # land
    #allcfs.land(targetHeight=0.06, duration=3)
