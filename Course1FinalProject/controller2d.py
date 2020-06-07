#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
from math import sin, cos, atan2, sqrt, pi
from copy import deepcopy

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
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

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

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('e_integration', 0.0)
        self.vars.create_var('t_previous', 0.0)


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                More about waypoints:
                the waypoints array that is used in the update_controls function is a roughly 2200 rows long, 
                and is continuously updated as the vehicle moves based on the nearest waypoint from the racetrack_waypoints file. 
                Its intended to give you better fidelity of points to navigate with so you don't have to do the interpolation yourself.
                
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            """
                A simple PI controller:
                u = Kp * e(t) + Ki * integration(e(t))
                Assuming that the acceleration output is the throttle output (not true in practice)
                
            """
            k_p = 1.0
            k_i = 0.2

            e = v_desired - v
            dt = t - self.vars.t_previous
            e_integration = self.vars.e_integration + e * dt

            u = k_p * e + k_i * e_integration

            throttle_output = u
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller.
            """
                A simple pure persuit controller:
                delta = atan(2 * L * sin (alpha) / (K_da * v_f))
                    
                
                delta: steering angle
                L: vehicle body length
                l_d: look_ahead distance
                alpha: the angle between look_ahead distance line and vehicle heading vector
                K_da: tuning parameter
                vf: forward velocity
                
                Both waypoints (x, y) and the current vehicle (x, y, yaw) positions are in the global coordinate frame.
                
                the angle between look-ahead line and global cordinate 
                alpha_head = arctan( (waypoint[TBD][1] - y) / (waypoints[TBD][0] - x) )
                

            """
            L = 3.0 # vehicle body length
            k_da = 2.0
            ld = k_da * v


            #x_rear = x_cg - 0.5 * L * cos(yaw)
            #y_rear = y_cg - 0.5 * L * sin(yaw)
            # todo: Pure Persuite should use rear wheel postion


            dist2point_max = 0
            dist2end = sqrt((x - waypoints[-1][0]) * (x - waypoints[-1][0]) +
                              (y - waypoints[-1][1]) * (y - waypoints[-1][1]))

            if dist2end > ld:
                for i in range(len(waypoints)):
                    dist2point = sqrt((x - waypoints[i][0]) * (x - waypoints[i][0]) +
                                  (y - waypoints[i][1]) * (y - waypoints[i][1]))
                    if dist2point_max < ld:
                        if dist2point > dist2point_max:
                            dist2point_max = dist2point
                    else:
                        break
                waypoint_lookahead = waypoints[i]
                alpha_head = atan2(waypoint_lookahead[1] - y, waypoint_lookahead[0] - x)
                alpha = alpha_head - yaw
                delta = atan2(2 * L * sin(alpha), (k_da * v))

            else:
                alpha_head = atan2(waypoints[-1][1] - y, waypoints[-1][0] - x)
                alpha = alpha_head - yaw
                delta = atan2(2 * L * sin(alpha), dist2end)

            steer_output = delta










            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t   # Store the current time step to be used in next step
        self.vars.e_integration = e_integration  # Store the error integration to be used in next step

