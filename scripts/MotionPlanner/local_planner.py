#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Assignments Solution Author: Engin Bozkurt
Motion Planning for Self-Driving Cars
Aug 24, 2019
"""

# Author: Ryan De Iaco
# Additional Comments: Carlos Wang
# Date: October 29, 2018

import numpy as np
import copy
import MotionPlanner.path_optimizer as path_optimizer
import MotionPlanner.collision_checker as collision_checker
import MotionPlanner.velocity_planner as velocity_planner
from math import sin, cos, pi, sqrt

class LocalPlanner:
    def __init__(self, num_paths, path_offset, circle_offsets, circle_radii, 
                 path_select_weight, time_gap, a_max, slow_speed, 
                 stop_line_buffer):
        self._num_paths = num_paths
        self._path_offset = path_offset
        self._path_optimizer = path_optimizer.PathOptimizer()
        self._collision_checker = \
            collision_checker.CollisionChecker(circle_offsets,
                                               circle_radii,
                                               path_select_weight)
        self._velocity_planner = \
            velocity_planner.VelocityPlanner(time_gap, a_max, slow_speed, 
                                             stop_line_buffer)
        self._prev_best_path = None

    def get_goal_state_set(self, goal_index, goal_state, waypoints, ego_state):
        
        if goal_index < len(waypoints)-1:
            delta_x = waypoints[goal_index+1][0] - waypoints[goal_index][0]
            delta_y = waypoints[goal_index+1][1] - waypoints[goal_index][1]

        else: 
            delta_x = waypoints[goal_index][0] - waypoints[goal_index-1][0]
            delta_y = waypoints[goal_index][1] - waypoints[goal_index-1][1]
        



        heading = np.arctan2(delta_y,delta_x)
  

        
        goal_state_local = copy.copy(goal_state)

        goal_state_local[0] -= ego_state[0] 
        goal_state_local[1] -= ego_state[1] 
        
        #for smoothing, but you have to very dense waypoints
        theta = -ego_state[2]

        goal_x = goal_state_local[0] * cos(theta) - goal_state_local[1] * sin(theta)
        goal_y = goal_state_local[0] * sin(theta) + goal_state_local[1] * cos(theta)
       
        goal_t = heading - ego_state[2]
    

       
        goal_v = goal_state[2]

        if goal_t > pi:
            goal_t -= 2*pi
        elif goal_t < -pi:
            goal_t += 2*pi

        goal_state_set = []
        for i in range(self._num_paths):
           
            offset = (i - self._num_paths // 2) * self._path_offset

            x_offset = offset * cos(goal_t + pi/2)
            y_offset = offset * sin(goal_t + pi/2)

            goal_state_set.append([goal_x + x_offset, 
                                   goal_y + y_offset, 
                                   goal_t, 
                                   goal_v])
        

        return goal_state_set  
              
    def plan_paths(self, goal_state_set):
        paths         = []
        path_validity = []
        for goal_state in goal_state_set:
            path = self._path_optimizer.optimize_spiral(goal_state[0], 
                                                        goal_state[1], 
                                                        goal_state[2])
            if np.linalg.norm([path[0][-1] - goal_state[0], 
                               path[1][-1] - goal_state[1], 
                               path[2][-1] - goal_state[2]]) > 1000:
                path_validity.append(False)
            else:
                paths.append(path)
                path_validity.append(True)
        
        return paths, path_validity


def transform_paths(paths, ego_state):
    
    transformed_paths = []
    for path in paths:
        x_transformed = []
        y_transformed = []
        t_transformed = []

        for i in range(len(path[0])):
            x_transformed.append(ego_state[0] + path[0][i]*cos(ego_state[2]) - \
                                                path[1][i]*sin(ego_state[2]))
            y_transformed.append(ego_state[1] + path[0][i]*sin(ego_state[2]) + \
                                                path[1][i]*cos(ego_state[2]))
            t_transformed.append(path[2][i] + ego_state[2])

        transformed_paths.append([x_transformed, y_transformed, t_transformed])

    return transformed_paths
