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
NUM_PATHS = 3
BP_LOOKAHEAD_BASE      = 3.0              # m
BP_LOOKAHEAD_TIME      = 1.0              # s
PATH_OFFSET            = 1.5              # m
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.5, 1.5, 1.5]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10
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
           


    def get_current_pose(self,position,car_yaw_degree):
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
        yaw = car_yaw_degree

        return (x, y, yaw)


    # def send_control_command(self, throttle, steer, brake, 
    #                         hand_brake=False, reverse=False):
    #     """Send control command to CARLA client.
        
    #     Send control command to CARLA client.

    #     Args:
    #         client: The CARLA client object
    #         throttle: Throttle command for the sim car [0, 1]
    #         steer: Steer command for the sim car [-1, 1]
    #         brake: Brake command for the sim car [0, 1]
    #         hand_brake: Whether the hand brake is engaged
    #         reverse: Whether the sim car is in the reverse gear
    #     """
    
    #     # Clamp all values within their limits
    #     steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    #     throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    #     brake = np.fmax(np.fmin(brake, 1.0), 0)

    #     # HERE SEND CONTROL SIGNALS TO Dummy_agent.py 

    #     # control.steer = steer
    #     # control.throttle = throttle
    #     # control.brake = brake
    #     # control.hand_brake = hand_brake
    #     # control.reverse = reverse
    #     # client.send_control(control)


    def exec_waypoint_nav_demo(self,position, car_yaw_degree, nsecs, speed):
        
        # Update pose and timestamp
        prev_timestamp = self.current_timestamp
        current_x, current_y, current_yaw = \
            self.get_current_pose(position,car_yaw_degree)
        current_speed = speed
        self.current_timestamp = nsecs



        open_loop_speed = self.lp._velocity_planner.get_open_loop_speed(self.current_timestamp - prev_timestamp)

        ego_state = [current_x, current_y, current_yaw, open_loop_speed]
        print('ego_state', ego_state)

        self.bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)

        # Perform a state transition in the behavioural planner.
        self.bp.transition_state(waypoints, ego_state, current_speed)


        goal_state_set = self.lp.get_goal_state_set(self.bp._goal_index, self.bp._goal_state, waypoints, ego_state)

        # Calculate planned paths in the local frame.
        paths, self.path_validity = self.lp.plan_paths(goal_state_set)

        # Transform those paths back to the global frame.
        paths = local_planner.transform_paths(paths, ego_state)
        print('in local paths[0] length', len(paths[0][0]))
        # Compute the best local path.
        best_index = self.lp._collision_checker.select_best_path_index(paths, self.bp._goal_state)

        # If no path was feasible, continue to follow the previous best path.
        if best_index == None:
            best_path = self.lp._prev_best_path
        else:
            best_path = paths[best_index]
            self.lp._prev_best_path = best_path

        desired_speed = self.bp._goal_state[2]
        
        decelerate_to_stop = self.bp._state == behavioural_planner.DECELERATE_TO_STOP
        self.local_waypoints = self.lp._velocity_planner.compute_velocity_profile(best_path, desired_speed, ego_state, current_speed, decelerate_to_stop, False)
        

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
        

        if self.local_waypoints != None and self.local_waypoints != []:
            self.controller.update_values(current_x, current_y, current_yaw, 
                                        current_speed,
                                        self.current_timestamp)
            self.controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = self.controller.get_commands()
        else:
            cmd_throttle = 0.0
            cmd_steer = 0.0
        
        return paths, cmd_throttle,cmd_steer,cmd_brake


    
