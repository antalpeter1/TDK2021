#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory
import yaml
import os

# Run the demo optimization
try:
    from demo_optimization import run_optimization
except:
    pass

take_initialPosition = True
# Get the initial position for each drone
cwd = os.getcwd()
with open(cwd + "/yaml/initialPosition.yaml", "r") as file:
    initialPosition = yaml.load(file, Loader=yaml.FullLoader)


if __name__ == "__main__":


    "Running crazyswarm"
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # TODO: check if number of trajectories == the number of drones in the swarm


    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):

        if take_initialPosition == True:
            # All take off
            allcfs.takeoff(targetHeight = 1.0, duration = 2.0)
            timeHelper.sleep(2.5)
            for j, cf in enumerate(allcfs.crazyflies):
                # Go to position
                pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
                cf.goTo(pos, 0, 5.0)

                print('Goto:' + str(pos))

            timeHelper.sleep(5)

            # Land
            allcfs.land(targetHeight=0.06, duration=3.0)
            timeHelper.sleep(3.0)

        # Push button to let the other drone to fly to its position as well.
        print("press button to continue...")
        swarm.input.waitUntilButtonPressed()


        # Check the last figure
        from PIL import Image
        f = Image.open(cwd + "/figures/04.png").show()
        print("press button to continue...")
        swarm.input.waitUntilButtonPressed()

        # Load trajectories
        traj1 = uav_trajectory.Trajectory()
        traj1.loadcsv(cwd + "/csv/vehicle0.csv")
        traj2 = uav_trajectory.Trajectory()
        traj2.loadcsv(cwd + "/csv/vehicle1.csv")
        traj3 = uav_trajectory.Trajectory()
        traj3.loadcsv(cwd + "/csv/vehicle2.csv")

        trajectories = [traj1, traj2, traj3]

        # Upload trajectories to cf
        for j, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajectories[j])

        # Takeoff
        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)

        # Trajectory following
        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)

        # Landing
        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
