#!/usr/bin/env python


class PID():

    def __init__(self, _kp, _ki, _kd, _max_i_value, _dt):

        # _kp, _ki ,_kd are the pid gains
        self.kp = _kp
        self.ki = _ki
        self.kd = _kd
        self.max_i_value = _max_i_value
        ### Y max_pid_value??
        self.dt = _dt

        self.prev_error = 0

        # The control output that we want to
        # estimate
        self.output = 0
        self.control_out_P = 0
        self.control_out_D = 0

        self.deriv = 0
        self.integ = 0

    def constrain(self, val, min_val, max_val):

        if val < min_val: return min_val
        if val > max_val: return max_val
        return val

    def update(self, desired, measurement):

        # The error between the desired state and
        # the measured one
        error = desired - measurement

        ########################
        # PID EQUATIONS
        ########################

        #P:
        ### self.output? aunque no necesario
        self.output = 0
        self.control_out_P = self.kp * error
        self.output += self.control_out_P
       
        #D:
        self.deriv = (error - self.prev_error) / self.dt
        self.control_out_D = self.kd * self.deriv 
        self.output += self.control_out_D

        #I
        self.integ += error * self.dt
        if(self.max_i_value != 0):
            self.integ = self.constrain(self.integ, -self.max_i_value, self.max_i_value)

        self.control_out_I = self.ki * self.integ
        self.output += self.control_out_I

        #if(self.max_pid_value != 0):
        #    self.output = self.constraint(self.output, -self.max_pid_value, self.max_pid_value)
        self.prev_error = error

        return self.output
