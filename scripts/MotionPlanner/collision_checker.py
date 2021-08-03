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
import scipy.spatial
from math import sin, cos, pi, sqrt
import time
class CollisionChecker:
    def __init__(self, circle_offsets, circle_radii, weight):
        self._circle_offsets = circle_offsets
        self._circle_radii   = circle_radii
        self._weight         = weight

    
    def collision_check(self, paths, obstacles):
        
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path           = paths[i]

            
            for j in range(len(path[0])):
                
                circle_locations = np.zeros((len(self._circle_offsets), 2))

               
                circle_offset = np.array(self._circle_offsets)
                circle_locations[:, 0] = path[0][j] + circle_offset * cos(path[2][j])
                circle_locations[:, 1] = path[1][j] + circle_offset * sin(path[2][j])
                #print("circle", circle_locations)
                for k in range(len(obstacles)):
                    collision_dists = \
                        scipy.spatial.distance.cdist([obstacles[k]], 
                                                     circle_locations)
                    collision_dists = np.subtract(collision_dists, 
                                                  self._circle_radii)
                    collision_free = collision_free and \
                                     not np.any(collision_dists < 0)

                    if not collision_free:
                        break
                if not collision_free:
                    break

            collision_check_array[i] = collision_free

        return collision_check_array

   
    def select_best_path_index(self, paths, collision_check_array, goal_state):
        best_index = None
        best_score = float('Inf')
        for i in range(len(paths)):
            # Handle the case of collision-free paths.
            if collision_check_array[i]:
              
                score = self._weight * sqrt((paths[i][1][-1] - goal_state[1]) ** 2 + (paths[i][0][-1] - goal_state[0]) ** 2)
               
                for j in range(len(paths)):
                    if j == i:
                        continue
                    else:
                        if not collision_check_array[j]:
                            score += sqrt((paths[i][1][-1] - paths[j][1][-1]) ** 2 + (paths[i][0][-1] - paths[j][0][-1]) ** 2)
            else:
                score = float('Inf')
            
            if score < best_score:
                best_score = score
                best_index = i

        return best_index
