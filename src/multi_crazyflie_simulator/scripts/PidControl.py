#!/usr/bin/env python

import numpy as np
from math import cos, sin, tan
from PyQt4.QtCore import QThread

from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID


class PidControl(QThread):

    def __init__(self, name, crazyflie):

        QThread.__init__(self)

        self.topic_name = name

        self.cf = crazyflie

        self.cf_physical_params = CF_parameters()

        # Import the PID gains (from the firmware)
        self.cf_pid_gains = CF_pid_params()

        # Out from the PIDs, values of
        # r, p, y, thrust
        self.desired_rpy = np.zeros(3)

        # Comes from the external position controller
        self.desired_thrust = 0.0

        self.mode = ""

        self.setPoint = np.zeros(4)

        self.stop_flag = True
        ######################
        # Position
        ######################

        self.x_pid = PID(self.cf_pid_gains.KP_X,
                         self.cf_pid_gains.KI_X,
                         self.cf_pid_gains.KD_X,
                         self.cf_pid_gains.INT_MAX_X,
                         self.cf_pid_gains.X_DT)

        self.y_pid = PID(self.cf_pid_gains.KP_Y,
                         self.cf_pid_gains.KI_Y,
                         self.cf_pid_gains.KD_Y,
                         self.cf_pid_gains.INT_MAX_Y,
                         self.cf_pid_gains.Y_DT)

        self.z_pid = PID(self.cf_pid_gains.KP_Z,
                         self.cf_pid_gains.KI_Z,
                         self.cf_pid_gains.KD_Z,
                         self.cf_pid_gains.INT_MAX_Z,
                         self.cf_pid_gains.Z_DT)

        self.desired_pos = np.zeros(3)

        ######################
        # Linear velocities
        ######################

        self.vx_pid = PID(self.cf_pid_gains.KP_VX,
                          self.cf_pid_gains.KI_VX,
                          self.cf_pid_gains.KD_VX,
                          self.cf_pid_gains.INT_MAX_VX,
                          self.cf_pid_gains.VX_DT)

        self.vy_pid = PID(self.cf_pid_gains.KP_VY,
                          self.cf_pid_gains.KI_VY,
                          self.cf_pid_gains.KD_VY,
                          self.cf_pid_gains.INT_MAX_VY,
                          self.cf_pid_gains.VY_DT)

        self.vz_pid = PID(self.cf_pid_gains.KP_VZ,
                          self.cf_pid_gains.KI_VZ,
                          self.cf_pid_gains.KD_VZ,
                          self.cf_pid_gains.INT_MAX_VZ,
                          self.cf_pid_gains.VZ_DT)

        self.desired_lin_vel = np.zeros(3)

        ######################
        # Angular velocities
        ######################

        self.wx_pid = PID(self.cf_pid_gains.KP_WX,
                          self.cf_pid_gains.KI_WX,
                          self.cf_pid_gains.KD_WX,
                          self.cf_pid_gains.INT_MAX_WX,
                          self.cf_pid_gains.WX_DT)

        self.wy_pid = PID(self.cf_pid_gains.KP_WY,
                          self.cf_pid_gains.KI_WY,
                          self.cf_pid_gains.KD_WY,
                          self.cf_pid_gains.INT_MAX_WY,
                          self.cf_pid_gains.WY_DT)

        self.wz_pid = PID(self.cf_pid_gains.KP_WZ,
                          self.cf_pid_gains.KI_WZ,
                          self.cf_pid_gains.KD_WZ,
                          self.cf_pid_gains.INT_MAX_WZ,
                          self.cf_pid_gains.WZ_DT)

        self.desired_ang_vel = np.zeros(3)

        ######################
        # Attitudes
        ######################

        self.roll_pid = PID(self.cf_pid_gains.KP_ROLL,
                            self.cf_pid_gains.KI_ROLL,
                            self.cf_pid_gains.KD_ROLL,
                            self.cf_pid_gains.INT_MAX_ROLL,
                            self.cf_pid_gains.ROLL_DT)

        self.pitch_pid = PID(self.cf_pid_gains.KP_PITCH,
                             self.cf_pid_gains.KI_PITCH,
                             self.cf_pid_gains.KD_PITCH,
                             self.cf_pid_gains.INT_MAX_PITCH,
                             self.cf_pid_gains.PITCH_DT)

        self.yaw_pid = PID(self.cf_pid_gains.KP_YAW,
                           self.cf_pid_gains.KI_YAW,
                           self.cf_pid_gains.KD_YAW,
                           self.cf_pid_gains.INT_MAX_YAW,
                           self.cf_pid_gains.YAW_DT)

        self.desired_att = np.zeros(3)


    def run_pos_pid(self):
        self.desired_lin_vel = np.array([self.x_pid.update(self.desired_pos[0], self.cf.cf_state[0]),
                                       self.y_pid.update(self.desired_pos[1], self.cf.cf_state[1]),
                                       self.z_pid.update(self.desired_pos[2], self.cf.cf_state[2])])

    def run_lin_vel_pid(self):
        self.desired_att = np.array([self.y_pid.update(self.desired_lin_vel[1], self.cf.cf_state[4]),
                                     self.x_pid.update(self.desired_lin_vel[0], self.cf.cf_state[3]),
                                   self.z_pid.update(self.desired_lin_vel[2], self.cf.cf_state[5])])
        self.desired_thrust = self.desired_att[2]*10000

    def run_yaw_pid(self):
        self.desired_yaw_rate = np.array([self.yaw_pid.update(self.desired_att[2], self.cf.cf_state[8])])


    def run_att_pid(self):
        self.desired_ang_vel = np.array([self.roll_pid.update(self.desired_att[0], self.cf.cf_state[6]),
                                         self.pitch_pid.update(self.desired_att[1], self.cf.cf_state[7]),
                                         self.desired_ang_vel[2]])

    def run_ang_vel_pid(self):
        self.desired_rpy = np.array([self.wx_pid.update(self.desired_ang_vel[0], self.cf.cf_state[9]),
                                     self.wy_pid.update(self.desired_ang_vel[1], self.cf.cf_state[10]),
                                     self.wz_pid.update(self.desired_ang_vel[2], self.cf.cf_state[11])])

        self.rpyt_2_motor_pwm()

    def rpyt_2_motor_pwm(self):

        # Inputs
        r = 0 * self.desired_rpy[0]
        p = 0 * self.desired_rpy[1]
        y = 0 * self.desired_rpy[2]
        thrust = self.desired_thrust

        ##########################
        # Function that transform the output
        # r, p, y, thrust of the PIDs,
        # into the values of the PWM
        # applied to each motor
        ##########################
        R = r / 2.0
        P = p / 2.0
        Y = y

        self.cf.addPWM([self.limit(thrust - R + P + Y),
                        self.limit(thrust - R - P - Y),
                        self.limit(thrust + R - P + Y),
                        self.limit(thrust + R + P - Y)])

    def limit(self, value):
        VAL_MAX = 65535
        if (value > VAL_MAX):
            value = VAL_MAX

        if (value < 0):
            value = 0

        return value

    def setStopFlag(self, bool_in):
        self.stop_flag = bool_in

    def setMode(self, mode):
        self.mode = mode

    def setSetPoint(self, setpoint, mode_set):
        if mode_set == "pos_pid":
            self.desired_pos = np.array(setpoint[0:3])
            self.desired_att[2] = setpoint[3]
        else:
            self.desired_att[0:2] = np.array(setpoint[0:2])
            self.desired_thrust = setpoint[3]
            self.desired_ang_vel[2] = setpoint[2]

    def run(self):
        if not self.stop_flag:
            if self.mode == "ATT":
                self.run_att_pid()
                self.run_ang_vel_pid()

            elif self.mode == "POS":
                self.run_pos_pid()
                self.run_lin_vel_pid()
                self.run_yaw_pid()
