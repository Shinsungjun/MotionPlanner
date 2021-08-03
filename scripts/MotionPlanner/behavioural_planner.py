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
# Date: November 21, 2018

import numpy as np
import math

# State machine states
FOLLOW_LANE = 0
DECELERATE_TO_STOP = 1
STAY_STOPPED = 2
# Stop speed threshold
STOP_THRESHOLD = 0.02
# Number of cycles before moving from stop sign.
STOP_COUNTS = 10

class BehaviouralPlanner:
    def __init__(self, lookahead):
        self._lookahead                     = lookahead

        self._state                         = FOLLOW_LANE

        self._goal_state                    = [0.0, 0.0, 0.0]
        self._goal_index                    = 0

    def set_lookahead(self, lookahead):
        self._lookahead = lookahead

    def transition_state(self, waypoints, ego_state, closed_loop_speed):
        
        if self._state == FOLLOW_LANE:
            
            closest_len, closest_index = get_closest_index(waypoints, ego_state)
            #print('closest_len, index', closest_len, closest_index)
            goal_index = self.get_goal_index(waypoints, ego_state, closest_len, closest_index)
            #while waypoints[goal_index][2] <= 0.1: goal_index += 1
            self._goal_index = goal_index
            self._goal_state = waypoints[goal_index]
                
        else:
            raise ValueError('Invalid state value.')

    
    def get_goal_index(self, waypoints, ego_state, closest_len, closest_index):
        arc_length = closest_len
        wp_index = closest_index
        
        if arc_length > self._lookahead:
            return wp_index

        if wp_index == len(waypoints) - 1:
            return wp_index
        count = 0

        while wp_index < len(waypoints) - 1:
            arc_length += np.sqrt((waypoints[wp_index][0] - waypoints[wp_index+1][0])**2 + (waypoints[wp_index][1] - waypoints[wp_index+1][1])**2)
            if arc_length > self._lookahead: break
            wp_index += 1
        
        return wp_index

def get_closest_index(waypoints, ego_state):
    
    closest_len = float('Inf')
    closest_index = 0
    for i in range(len(waypoints)):
        temp = (waypoints[i][0] - ego_state[0])**2 + (waypoints[i][1] - ego_state[1])**2
        if temp < closest_len:
            closest_len = temp
            closest_index = i
    closest_len = np.sqrt(closest_len)
    return closest_len, closest_index
      
def pointOnSegment(p1, p2, p3):
    if (p2[0] <= max(p1[0], p3[0]) and (p2[0] >= min(p1[0], p3[0])) and \
       (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
        return True
    else:
        return False
