"""
Main function

Ways to improve:
    - hyperparam optimization
"""

import time
# from group import Group
# from obstacle import Obstacle
from src import *
import matplotlib.pyplot as plt
import numpy as np

import random
import sys

import os
os.system("mkdir log")
os.system("mkdir urdf")
os.system("mkdir csv")
os.system("mkdir figures")
os.system("mkdir video")
os.system("mkdir yaml")





def run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage):


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


    targetHeight = 0.8
    # n_iterations = 50
    obstacles = []
    # start_position=[-1.1, 0.5]
    # goal_position=[1.0, -0.5]
    # start_position=[-0.7, 0.4]
    # goal_position=[1.1, -0.4]
    
    plt.close('all')
    seed = random.randrange(sys.maxsize) % 100
    random.Random(seed)
    print("Seed was:", seed)

    
    # Torus
    if len(corners_list) > 0:
        obstacle = Obstacle(ID = 0, corners = corners_list[0], obstacle_type = "torus")
        obstacles += [obstacle]
        obstacle = Obstacle(ID = 1, corners = corners_list[1], obstacle_type = "torus")
        obstacles += [obstacle]
        obstacles[-2].generate_urdf_torus(obstacles[-1].corners)
    

    try:
        1/0
        # Poles read from optitrack
        import rospy
        from geometry_msgs.msg import PoseStamped
        rospy.init_node("optitrack_vrpn_listener", anonymous = False)
        for i in range(10):
            topic = '/vrpn_client_node/object_' + str(i) + '/pose'
            res = rospy.wait_for_message(topic = topic, topic_type = PoseStamped, timeout = 0.1)
            middle_x = res.x
            middle_y = res.y
            
            size = 0.1
            delta = size/2
            corner1 = [middle_x - delta, middle_y - delta]
            corner2 = [middle_x + delta, middle_y - delta]
            corner3 = [middle_x + delta, middle_y + delta]
            corner4 = [middle_x - delta, middle_y + delta]
            corners_list = [corner1] + [corner2] + [corner3] + [corner4]
            
            obstacle = Obstacle(ID = 2 + i, corners = corners_list[0], obstacle_type = "pole")
            obstacles += [obstacle]
            
            
    except:
            # Poles randomly placed
            # obstacle = Obstacle(ID = 3, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            # obstacle = Obstacle(ID = 4, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            # obstacle = Obstacle(ID = 5, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            # obstacle = Obstacle(ID = 6, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            # obstacle = Obstacle(ID = 7, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            # obstacle = Obstacle(ID = 8, obstacle_type="pole") # Randomly placed obstacle
            # obstacles += [obstacle]
            
            # read from csv
            obstacle_positions, vehicle_positions = read_optitrack()
            for i, key in enumerate(obstacle_positions):
                
                # This will be our torus
                if key == "obst0":
                    torus_middle = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
                    radious = 0.80/2
                    length = 2
                    width = 0.02
                    targetHeight = obstacle_positions[key]['y'] + 0.85/2 # 0.85m is the diameter of the hula hoop
                    
                    # First create the bottom gate
                    corner1 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([-width/2, -length/2])).tolist()
                    corner2 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([width/2, -length/2])).tolist()
                    corner3 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([width/2, length/2])).tolist()
                    corner4 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([-width/2, length/2])).tolist()
                    corners = [corner1, corner2, corner3, corner4]
                    
                    obstacle = Obstacle(ID = 0, corners = corners, obstacle_type = "torus")
                    obstacles += [obstacle]
                    
                    # Now the second part, the top gate:
                    corner1 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([-width/2, -length/2])).tolist()
                    corner2 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([width/2, -length/2])).tolist()
                    corner3 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([width/2, length/2])).tolist()
                    corner4 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([-width/2, length/2])).tolist()
                    corners = [corner1, corner2, corner3, corner4]
                    
                    obstacle = Obstacle(ID = 1, corners = corners, obstacle_type = "torus")
                    obstacles += [obstacle]
                    obstacles[-2].generate_urdf_torus(obstacles[-1].corners)
                
                 # Rest of the obstacles :)        
                else:
                    corners = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
                    obstacle = Obstacle(ID = i+3, corners = corners,obstacle_type="pole") # Randomly placed obstacle
                    obstacles += [obstacle]
    
    
    
    # Create group
    group = Group(n_vehicles=3, start_position = start_position, goal_position = goal_position, stage = stage)
    group.set_group_position(
        position=group.start_position,targetHeight = targetHeight, position_type='initial')
    group.set_group_position(
        position=group.goal_position, targetHeight = targetHeight, position_type='final')
    group.add_obstacles(obstacles)
    group.organise_neighbours()
    
    
    # if plot_environment == True:
    #     group.plot_setup()
    #     n_iterations = 0
    #     # return 0
    # else:
        
    group.prepare()
    
    t_iter = time.time()
    iteration_times = []
    collision_happened = True
    i = 0
    while collision_happened or i < min_iterations:
        group.solve()
    
        # Time-related things
        iteration_times += [time.time() - t_iter]
        # print(str(i) + "th iteration time: " +
        #       str(time.time() - t_iter) + " seconds")
        t_iter = time.time()
    
        # group.plotter(iternum=i, seed=seed)
        collision_happened = group.check_collision()
        
        i = i + 1
        if i > max_iterations:
            collision_happened = group.check_collision(plot_distances = True)
            # group.plot_vehicle_trajectories_gradient()
            group.save_trajectory_to_csv(t_desired = 2.5, t_hover = 0)
            break
        
        if collision_happened == False and i >= min_iterations:
            collision_happened = group.check_collision(plot_distances = True)
            # group.plot_vehicle_trajectories_gradient()
            group.save_trajectory_to_csv(t_desired = 2.5, t_hover = 0)

        if i == 50:
            for j in range(len(group.vehicles)):
                group.vehicles[j].rho = group.vehicles[j].rho / 2
        if i == 100:
            for j in range(len(group.vehicles)):
                group.vehicles[j].rho = group.vehicles[j].rho / 5
        # if i == 150:
        #     for j in range(len(group.vehicles)):
        #         group.vehicles[j].rho = group.vehicles[j].rho / 2

    group.plot_moovie_frames(iternum=i, seed=seed)
    
    writing_parameters_to_file(iteration_times)
    
    print("x update times")
    for vehicle in group.vehicles:
        print(np.mean(np.array(vehicle.x_update_time)))
    print("z update times")
    for vehicle in group.vehicles:
        print(np.mean(np.array(vehicle.z_update_time)))
    
    # for vehicle in group.vehicles:
    #     for line in vehicle.maze_printable:
    #         print(line)
            
    for i in range(len(group.vehicles)):
        print(group.vehicles[i].solution['f'])
        
    return [vehicle.solution['f'].full().reshape(1, -1).tolist()[0][0] for vehicle in group.vehicles], group
        
        
# gate1 = [[-0.3, -2.0],
#           [-0.2, -2.0],
#           [-0.2, -0.25],
#           [-0.3, -0.25]]

# gate2  = [[-0.3, 0.25],
#           [-0.2, 0.25],
#           [-0.2, 2],
#           [-0.3, 2]]
corners_list = []
# corners_list += [gate1]
# corners_list += [gate2]


group_stages = []
min_iterations = 2
max_iterations = 2


# # Stage 0
# start_position=[-0.65, -0.35]
# goal_position=[1.05, -0.35]
# stage = 0
# costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)
# group_stages += [group]


# Stage 0
stage = 0
start_position = [-0.8, -0.35]
goal_position = [1.05, -0.35]
costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)
group_stages += [group]
group_stages[stage].save_trajectory_to_csv(t_desired = 3, t_hover = 0)

# # Next stage
# start_position = goal_position
# goal_position = [-0.6, 0.35]
# stage = stage + 1
# costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)
# group_stages += [group]
# group_stages[stage].save_trajectory_to_csv(t_desired = 3, t_hover = 0)

# # Next stage
# start_position = goal_position
# goal_position = [1.05, 0.35]
# stage = stage + 1
# costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)
# group_stages += [group]
# group_stages[stage].save_trajectory_to_csv(t_desired = 3, t_hover = 0)
  
# # Next stage  
# start_position = goal_position
# goal_position = [-0.6, -0.35]
# stage = stage + 1
# costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)
# group_stages += [group]
# group_stages[stage].save_trajectory_to_csv(t_desired = 3, t_hover = 0)







# for i in range(1):
#     costs = run_optimizaiton(corners_list)
#     if any(cost > 50 for cost in costs):
#         print("There is a problem")
#         print(costs)
#         print("iteration: " + str(i))
#         break
#     else:
#         print("**************")
#         print("**************")
#         print("**************")
#         print("**************")
#         print(costs)
#         print("iteration: " + str(i))

