#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.8

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # send the parameters
    for cf in allcfs.crazyflies:
        cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
        cf.setParam('stabilizer/controller', 1)  # PID controller
        # TODO set feedforward coefficients in "power_distribution_stock" to 1

    # allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    # timeHelper.sleep(5)


    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    print("Land")
    # allcfs.cmdStop()
    allcfs.land(targetHeight=0.3, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)


    N = 100  # number of measurements
    average = np.zeros(4)
    M = average
    for cf in allcfs.crazyflies:
        for i in range(N):
            M[0] = cf.getParam('motorPowerSet/logm1')
            M[1] = cf.getParam('motorPowerSet/logm2')
            M[2] = cf.getParam('motorPowerSet/logm3')
            M[3] = cf.getParam('motorPowerSet/logm4')
            for motor in range(4):
                average[motor] = i / (i + 1) * average[motor] + 1 / (i + 1) * M[motor]
            timeHelper.sleepForRate(10)
            print(average[0])

            U1 = 0.45  # * 65000 / 60000
            T1 = 0.3
            Th1 = -10
            T2 = 0.14  # for plus config: 0.15, for x config: 0.142
            T3 = 0
            Th2 = Th2 * 31000 / 24400
            Th4 = -Th2
            T4 = 0.2
            U5 = U1
            T5 = 0  # .1
            Th5 = -Th1