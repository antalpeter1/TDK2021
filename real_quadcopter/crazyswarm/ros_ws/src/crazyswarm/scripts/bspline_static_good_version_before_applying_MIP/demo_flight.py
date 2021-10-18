#!/usr/bin/env python



import numpy as np
from pycrazyswarm import *




# Reading initial positions
import yaml
import os
cwd = os.getcwd()
cwd = cwd
with open(cwd + "/bspline_static/yaml/initialPosition.yaml", "r") as file:
    initialPosition = yaml.load(file, Loader=yaml.FullLoader)
# Setting targetHeight
targetHeight = 1.3
# Loading trajectories
import uav_trajectory
traj1 = uav_trajectory.Trajectory()
traj1.loadcsv(cwd + "/bspline_static/csv/vehicle0.csv")
traj2 = uav_trajectory.Trajectory()
traj2.loadcsv(cwd + "/bspline_static/csv/vehicle1.csv")
traj3 = uav_trajectory.Trajectory()
traj3.loadcsv(cwd + "/bspline_static/csv/vehicle2.csv")
trajectories = [traj1, traj2, traj3]
        
if __name__ == "__main__":


    "Running crazyswarm"
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs



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
