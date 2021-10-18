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
# 
# Function definition
import os
os.system("mkdir log")
os.system("mkdir urdf")
os.system("mkdir csv")
os.system("mkdir figures")
os.system("mkdir video")
os.system("mkdir yaml")

def writing_parameters_to_file(iteration_times):
    """
    Writing parameteres to file
    """
    cwd = os.getcwd()
    log_file = open(cwd + "/log/" + "log.txt", "w")

    # Writing time required for iteration into file
    log_file.writelines('\n')
    log_file.writelines('Iteration times: \n')
    for i in range(len(iteration_times)):
        log_file.writelines(str(iteration_times[i]) + '\n')

    log_file.writelines('Final time: \n')
    log_file.writelines(str(np.sum(iteration_times)))



n_iterations = 5
obstacles = None
plot_environment=False
start_position=[-1.1, 0]
goal_position=[1.2, 0]
# goal_position=[-1.1, 0]
# start_position=[1.2, 0]

plt.close('all')
seed = random.randrange(sys.maxsize) % 100
# seed = 26
rng = random.Random(seed)
print("Seed was:", seed)



environment = Environment()

# Randomly placed obstacles
# obstacles = [Obstacle(ID=i) for i in range(2)]

# Narrow corridor obstacles if no input
if obstacles is None:
    obstacles = []
    gate1 = [[-0.5, -2.0],
              [-0.3, -2.0],
              [-0.3, -0.25],
              [-0.5, -0.25]]
    
    gate2  = [[-0.5, 0.25],
              [-0.3, 0.25],
              [-0.3, 2],
              [-0.5, 2]]
    
    obstacle = Obstacle(ID = 0, corners = gate1, obstacle_type = "torus")
    obstacles += [obstacle]
    obstacle = Obstacle(ID = 1, corners = gate2, obstacle_type = "torus")
    obstacles += [obstacle]
    obstacles[-2].generate_urdf_torus(obstacles[-1].corners)
    obstacle = Obstacle(ID = 3, obstacle_type="pole") # Randomly placed obstacle
    obstacles += [obstacle]
    obstacle = Obstacle(ID = 4, obstacle_type="pole") # Randomly placed obstacle
    obstacles += [obstacle]
    
# Random obstacles if input is a string 'random'
elif obstacles == 'random':
    obstacles = [Obstacle(ID=i, obstacle_type="pole") for i in range(2)]
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
    # return 0
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
    
        group.plotter(iternum=i, seed=seed)
    
    
    group.plot_moovie_frames(iternum=i, seed=seed)
    
    writing_parameters_to_file(iteration_times)
    
    for i in range(len(group.vehicles)):
        print(group.vehicles[i].solution['f'])
        
#     return [group.vehicles[i].solution['f'].full().reshape(-1,).tolist()[0] for i in range(len(group.vehicles))]

# opt()