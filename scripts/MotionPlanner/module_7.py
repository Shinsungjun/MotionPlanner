from __future__ import print_function
from __future__ import division
#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Assignments Solution Author: Engin Bozkurt
Motion Planning for Self-Driving Cars
Aug 24, 2019
"""

"""
CARLA waypoint follower assessment client script.

A controller assessment to follow a given trajectory, where the trajectory
can be defined using way-points.

STARTING in a moment...
"""


# System level imports
import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv
import matplotlib.pyplot as plt
import configparser 
import MotionPlanner.local_planner as local_planner
import MotionPlanner.behavioural_planner as behavioural_planner
import MotionPlanner.controller2d as controller2d
import time

# Script level imports
sys.path.append(os.path.abspath(sys.path[0] + '/..'))


"""
Configurable params
"""
ITER_FOR_SIM_TIMESTEP  = 10     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 1.00   # game seconds (time before controller start)
TOTAL_RUN_TIME         = 100.00 # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300    # number of frames to buffer after total runtime
NUM_PEDESTRIANS        = 0      # total number of pedestrians to spawn
NUM_VEHICLES           = 2      # total number of vehicles to spawn
SEED_PEDESTRIANS       = 0      # seed for pedestrian spawn randomizer
SEED_VEHICLES          = 0      # seed for vehicle spawn randomizer
CLIENT_WAIT_TIME       = 3      # wait time for client before starting episode
                                # used to make sure the server loads
                                # consistently

WEATHERID = {
    "DEFAULT": 0,
    "CLEARNOON": 1,
    "CLOUDYNOON": 2,
    "WETNOON": 3,
    "WETCLOUDYNOON": 4,
    "MIDRAINYNOON": 5,
    "HARDRAINNOON": 6,
    "SOFTRAINNOON": 7,
    "CLEARSUNSET": 8,
    "CLOUDYSUNSET": 9,
    "WETSUNSET": 10,
    "WETCLOUDYSUNSET": 11,
    "MIDRAINSUNSET": 12,
    "HARDRAINSUNSET": 13,
    "SOFTRAINSUNSET": 14,
}
SIMWEATHER = WEATHERID["CLEARNOON"]     # set simulation weather

PLAYER_START_INDEX = 1      # spawn index for player (keep to 1)
FIGSIZE_X_INCHES   = 8      # x figure size of feedback in inches
FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
PLOT_LEFT          = 0.1    # in fractions of figure width and height
PLOT_BOT           = 0.1    
PLOT_WIDTH         = 0.8
PLOT_HEIGHT        = 0.8

WAYPOINTS_FILENAME = 'waypoints.txt'  # waypoint file to load #!!!!
DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
                                       # simulation ends

# Planning Constants
NUM_PATHS = 7
BP_LOOKAHEAD_BASE      = 6.0            # m
BP_LOOKAHEAD_TIME      = 1.0              # s
PATH_OFFSET            = 0.7           # m
CIRCLE_OFFSETS         = [0.1, 0.1, 0.1] # m
CIRCLE_RADII           = [0.4, 0.4, 0.4]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 100
A_MAX                  = 1.5              # m/s^2
SLOW_SPEED             = 2.0              # m/s
STOP_LINE_BUFFER       = 3.5              # m
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 2                # Frequency divisor to make the 
                                          # local planner operate at a lower
                                          # frequency than the controller
                                          # (which operates at the simulation
                                          # frequency). Must be a natural
                                          # number.

# Course 4 specific parameters
C4_STOP_SIGN_FILE        = 'stop_sign_params.txt'
C4_STOP_SIGN_FENCELENGTH = 5        # m
C4_PARKED_CAR_FILE       = 'parked_vehicle_params.txt'

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # selected path
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

# controller output directory
CONTROLLER_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
                           '/controller_output/'

waypoints =None
waypoints_file = WAYPOINTS_FILENAME
waypoints_filepath =\
        os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                        WAYPOINTS_FILENAME)
with open(waypoints_filepath) as waypoints_file_handle:
    waypoints = list(csv.reader(waypoints_file_handle, 
                                delimiter=',',
                                quoting=csv.QUOTE_NONNUMERIC))
controller = controller2d.Controller2D(waypoints)

class Autonomous: 
    def __init__(self):
        self.current_timestamp = 0 
        
        self.local_waypoints = None
        self.path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)
        self.lp = local_planner.LocalPlanner(NUM_PATHS,
                                        PATH_OFFSET,
                                        CIRCLE_OFFSETS,
                                        CIRCLE_RADII,
                                        PATH_SELECT_WEIGHT,
                                        TIME_GAP,
                                        A_MAX,
                                        SLOW_SPEED,
                                        STOP_LINE_BUFFER)
        self.waypoints = waypoints
        self.controller = controller
        self.bp = behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE)
        self.times = []
        self.curr_time = 0
        self.prev_time = 0


    def get_current_pose(self,position,yaw_rad):
        """Obtains current x,y,yaw pose from the client measurements
        
        Obtains the current x,y, and yaw pose from the client measurements.

        Args:
            measurement: The CARLA client measurements (from read_data())

        Returns: (x, y, yaw)
            x: X position in meters
            y: Y position in meters
            yaw: Yaw position in radians
        """
        x   = position.x
        y   = position.y
        yaw = yaw_rad

        return (x, y, yaw_rad)





    def exec_waypoint_nav_demo(self,position, yaw_rad, nsecs, speed, scan_data):
        self.prev_time = time.time()
        # Update pose and timestamp
        prev_timestamp = self.current_timestamp
        current_x, current_y, current_yaw = \
            self.get_current_pose(position,yaw_rad)
        current_speed = speed
        self.current_timestamp = nsecs



        open_loop_speed = self.lp._velocity_planner.get_open_loop_speed(self.current_timestamp - prev_timestamp)
        #print("open", open_loop_speed)

        ego_state = [current_x, current_y, current_yaw, open_loop_speed]
        #print('ego_state', ego_state)

        #self.bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)

        prevtime = time.time()
        # Perform a state transition in the behavioural planner.
        self.bp.transition_state(waypoints, ego_state, current_speed)

        currtime = time.time()
        #print('bp trainsition :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)
        prevtime = time.time()

        goal_state_set = self.lp.get_goal_state_set(self.bp._goal_index, self.bp._goal_state, waypoints, ego_state)
        currtime = time.time()
        #print('state set  :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)


        print("goal_index :",self.bp._goal_index)
        # Calculate planned paths in the local frame.
        prevtime = time.time()

        paths, self.path_validity = self.lp.plan_paths(goal_state_set,ego_state)
        currtime = time.time()
        #print('plan path  :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)


        #print("val", self.path_validity)
        #print("goal_state :", self.bp._goal_state)
        # for i in range(len(paths)):
        #     print("path last x, y :",paths[i][0][-1], paths[i][1][-1])
        # Transform those paths back to the global frame.
        prevtime = time.time()

        paths = local_planner.transform_paths(paths, ego_state)
        currtime = time.time()
        #print('transform :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)


        # Compute the best local path.
        prevtime = time.time()


          #  # Perform collision checking.
        collision_check_array = self.lp._collision_checker.collision_check(paths, scan_data)
        print(collision_check_array)

        best_index = self.lp._collision_checker.select_best_path_index(paths, collision_check_array, self.bp._goal_state)
        currtime = time.time()
        #print('select_best_path_index :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)


        #print("best_idx: ", best_index)

        # If no path was feasible, continue to follow the previous best path.
        if best_index == None:
            best_path = self.lp._prev_best_path
        else:
            best_path = paths[best_index]
            self.lp._prev_best_path = best_path

        desired_speed = self.bp._goal_state[2]
        
        #decelerate_to_stop = self.bp._state == behavioural_planner.DECELERATE_TO_STOP
        prevtime = time.time()

        self.local_waypoints = self.lp._velocity_planner.compute_velocity_profile(best_path, desired_speed, ego_state, current_speed, False, False)
        currtime = time.time()
        #print('compute_velocity_profile :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)

        
        prevtime = time.time()

        if self.local_waypoints != None:
            wp_distance = []   # distance array
            self.local_waypoints_np = np.array(self.local_waypoints)
            for i in range(1, self.local_waypoints_np.shape[0]):
                wp_distance.append(
                        np.sqrt((self.local_waypoints_np[i, 0] - self.local_waypoints_np[i-1, 0])**2 +
                                (self.local_waypoints_np[i, 1] - self.local_waypoints_np[i-1, 1])**2))
            wp_distance.append(0)  # last distance is 0 because it is the distance
                                    # from the last waypoint to the last waypoint

            # Linearly interpolate between waypoints and store in a list
            wp_interp      = []    # interpolated values 
                                    # (rows = waypoints, columns = [x, y, v])
            for i in range(self.local_waypoints_np.shape[0] - 1):
                wp_interp.append(list(self.local_waypoints_np[i]))
                num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                                float(INTERP_DISTANCE_RES)) - 1)
                wp_vector = self.local_waypoints_np[i+1] - self.local_waypoints_np[i]
                wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                for j in range(num_pts_to_interp):
                    next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                    wp_interp.append(list(self.local_waypoints_np[i] + next_wp_vector))
            # add last waypoint at the end
            wp_interp.append(list(self.local_waypoints_np[-1]))

            # Update the other controller values and controls
            self.controller.update_waypoints(wp_interp)
        currtime = time.time()
        #print('update_waypoints :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)

        
        prevtime = time.time()

        if self.local_waypoints != None and self.local_waypoints != []:
            self.controller.update_values(current_x, current_y, current_yaw, 
                                        current_speed,
                                        self.current_timestamp)
            self.controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = self.controller.get_commands()
        else:
            cmd_throttle = 0.0
            cmd_steer = 0.0
        currtime = time.time()
        #print('update_values :', 1/(currtime-prevtime))
        self.times.append(currtime-prevtime)
        
        #print("Alltimes : ")
        # for t in self.times:
        #     print(1/t)
        self.times = []
        self.curr_time = time.time()
        #print("AAAAAAAAa : ", 1/(self.curr_time - self.prev_time))
        return paths, cmd_throttle,cmd_steer,cmd_brake, best_index, collision_check_array, self.waypoints


    
