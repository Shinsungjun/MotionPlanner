#!/usr/bin/env python3
"""
Assignments Solution Author: Engin Bozkurt
Motion Planning for Self-Driving Cars
Aug 24, 2019
"""

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import MotionPlanner.cutils as cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = True
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp



    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('v_error_integral', 0.0)
        self.vars.create_var('heading_error_integral', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            
            kp = 0.2
            ki = 0.05
            kd = 0.01

            v_error = v_desired - v

            dt = t - self.vars.t_previous
            v_error_derivative = v_error/dt

            self.vars.v_error_integral += v_error * dt

            feedback = kp * v_error + ki * self.vars.v_error_integral + kd * v_error_derivative

            look_ahead = waypoints[len(waypoints)-1]
            v_desired_forward = look_ahead[2]
            if v_desired_forward <= 6:
            	feedforward = 0.15 + v_desired_forward/6*(0.6-0.15)
            elif v_desired <= 11.5:
            	feedforward = 0.6 + (v_desired_forward-6)/(11.5-6)*(0.8-0.6)
            else:
            	feedforward = 0.8 + (v_desired_forward-11.5)/85
            
            throttle_output = feedforward + feedback
            throttle_output = min(throttle_output,1)
            throttle_output = max(throttle_output,0)

            brake_output    = 0

            L = 1.5
            kp_lat = 1.5
            ki_lat = 0.2
            kd_lat = 0.5
           
            look_ahead_index = len(waypoints)//2
            look_ahead = waypoints[look_ahead_index]

            heading_error = yaw - np.arctan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])
            while heading_error > np.pi: heading_error -= np.pi*2
            while heading_error < -np.pi: heading_error += np.pi*2

            heading_error_derivative = heading_error/dt
            self.vars.heading_error_integral += heading_error * dt
            feedback_lateral = kp_lat * heading_error + ki_lat * self.vars.heading_error_integral + kd_lat * heading_error_derivative


            ld = np.sqrt((look_ahead[0] - x)**2 + (look_ahead[1] - y)**2)

            vector_look_ahead = [look_ahead[0] - x,look_ahead[1] - y]
            vector_car = [np.cos(yaw),np.sin(yaw)]
            cross_track_error = np.cross(vector_look_ahead, vector_car)

            curvature = 2/ld/ld*cross_track_error
            
            steer_output = np.arctan(curvature*L) + feedback_lateral
            steer_output = -steer_output
            steer_output = min(steer_output,1.22)
            steer_output = max(steer_output,-1.22)
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)
            
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t  # Store forward speed to be used in next step
