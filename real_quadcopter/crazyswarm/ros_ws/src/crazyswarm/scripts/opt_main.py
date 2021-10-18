"""
Main function

Ways to improve:
    - argparser
    - time-based inter-vehicle collision avoidance
    - hyperparam optimization
"""

import time
from group import Group
from obstacle import Obstacle
import matplotlib.pyplot as plt
import numpy as np
#from matplotlib.pyplot import cm
#from frenet_path import FrenetPath
#from numpy import interp
#import argparse
from environment import Environment

import random
import sys
import time


def opt(n_iterations : int = 5):
    
    # plt.close('all')
    seed = random.randrange(sys.maxsize) % 100
    seed = 26
    rng = random.Random(seed)
    print("Seed was:", seed)
    
    
    
    environment = Environment()
    
    
    def writing_parameters_to_file(iteration_times):
        """
        Writing parameteres to file
        """
        log_file = open("log.txt", "w")
    
        # Writing time required for iteration into file
        log_file.writelines('\n')
        log_file.writelines('Iteration times: \n')
        for i in range(len(iteration_times)):
            log_file.writelines(str(iteration_times[i]) + '\n')
    
        log_file.writelines('Final time: \n')
        log_file.writelines(str(np.sum(iteration_times)))
        
    
    
    
    # Randomly placed obstacles
    
    # obstacles = [Obstacle(ID=i) for i in range(5)]
    
    # Narrow corridor obstacles
    obstacles = []
    gate1 = [[-0.25, -1],
              [0.25, -1],
              [0.25, -0.2],
              [-0.25, -0.2]]
    
    gate2  = [[-0.25, 1],
              [0.25, 1],
              [0.25, 0.2],
              [-0.25, 0.2]]
    
    obstacle = Obstacle(ID = 0, corners = gate1)
    obstacles += [obstacle]
    obstacle = Obstacle(ID = 1, corners = gate2)
    obstacles += [obstacle]
    
    
    # Frenet path
    # ...
    
    group = Group(n_vehicles=3)
    group.set_group_position(
        position=environment.start_position, position_type='initial')
    group.set_group_position(
        position=environment.goal_position, position_type='final')
    group.add_obstacles(obstacles)
    group.organise_neighbours()
    group.prepare()
    
    t_iter = time.time()
    iteration_times = []
    elapsed = 0
    
    for i in range(n_iterations):
        group.solve()
    
        # Time-related things
        iteration_times += [time.time() - t_iter]
        print(str(i) + "th iteration time: " +
              str(time.time() - t_iter) + " seconds")
        t_iter = time.time()
    
        # group.plotter(iternum=i, folder='figures/', seed=seed)
    
    
    # group.plot_moovie_frames(iternum=i, folder='figures/video/', seed=seed)
    
    writing_parameters_to_file(iteration_times)
    
    for i in range(len(group.vehicles)):
        print(group.vehicles[i].solution['f'])
        
    return [group.vehicles[i].solution['f'].full().reshape(-1,).tolist()[0] for i in range(len(group.vehicles))]

# main()