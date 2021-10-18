#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


if __name__ == "__main__":
    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    h = 0.9
    len_x = 1.37
    len_y = 1.04
    dur = 2
    sl = 3
    allcfs.takeoff(targetHeight=h, duration=3)
    timeHelper.sleep(sl)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([len_x, len_y, h]), 0, dur)
    timeHelper.sleep(sl)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([-len_x, len_y, h]), 0, dur)
    timeHelper.sleep(sl)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([-len_x, -len_y-0.3, h]), 0, dur)
    timeHelper.sleep(sl)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([len_x, -len_y-0.3, h]), 0, dur)
    timeHelper.sleep(sl)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([len_x, len_y, h]), 0, dur)
    timeHelper.sleep(sl)
    allcfs.land(targetHeight=0.06, duration=3)