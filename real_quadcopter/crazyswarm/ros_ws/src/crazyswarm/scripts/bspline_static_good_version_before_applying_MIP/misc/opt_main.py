"""
Main function

Ways to improve:
    - hyperparam optimization
"""

import time
from group import Group
from obstacle import Obstacle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.pyplot import cm
from frenet_path import FrenetPath
from numpy import interp
import argparse
from environment import Environment

import random
import sys
import time

# Function definition
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


def opt(n_iterations : int = 5, obstacles = None, plot_environment = False, start_position = None, goal_position = None):
    
    # plt.close('all')
    seed = random.randrange(sys.maxsize) % 100
    seed = 26
    rng = random.Random(seed)
    print("Seed was:", seed)
    
    
    
    environment = Environment()
    
    # Randomly placed obstacles
    # obstacles = [Obstacle(ID=i) for i in range(5)]
    
    # Narrow corridor obstacles if no input
    if obstacles is None:
        obstacles = []
        gate1 = [[-0.1, -1.1],
                  [0.1, -1.1],
                  [0.1, -0.23],
                  [-0.1, -0.23]]
        
        gate2  = [[-0.1, 1.1],
                  [0.1, 1.1],
                  [0.1, 0.17],
                  [-0.1, 0.17]]
        
        obstacle = Obstacle(ID = 0, corners = gate1)
        obstacles += [obstacle]
        obstacle = Obstacle(ID = 1, corners = gate2)
        obstacles += [obstacle]
    # Random obstacles if input is a string 'random'
    elif obstacles == 'random':
        obstacles = [Obstacle(ID=i) for i in range(2)]
    # Build obstacle if obstacle coordinates were given
    else:
        obstacles_tmp = []
        for i, corners in enumerate(obstacles):
            obstacle = Obstacle(ID = i, corners = corners)
            obstacles_tmp += obstacle
        obstacles = obstacles_tmp
    
    
    # Frenet path
    # ...
    
    group = Group(n_vehicles=3, start_position = start_position, goal_position = goal_position)
    group.set_group_position(
        position=group.start_position, position_type='initial')
    group.set_group_position(
        position=group.goal_position, position_type='final')
    group.add_obstacles(obstacles)
    group.organise_neighbours()
    
    
    if plot_environment == True:
        group.plot_setup()
        n_iterations = 0
        return 0
    else:
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
        
            group.plotter(iternum=i, folder='figures/', seed=seed)
        
        
        group.plot_moovie_frames(iternum=i, folder='figures/video/', seed=seed)
        
        writing_parameters_to_file(iteration_times)
        
        for i in range(len(group.vehicles)):
            print(group.vehicles[i].solution['f'])
            
        return [group.vehicles[i].solution['f'].full().reshape(-1,).tolist()[0] for i in range(len(group.vehicles))]


plt.close('all')
opt(n_iterations = 5, plot_environment=False, start_position=[-1,0.5], goal_position=[1, 0.5])

