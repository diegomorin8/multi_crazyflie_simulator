import numpy as np
from math import cos, sin

#######################
#### CF Parameters ####
#######################
class CF_parameters():

    def __init__(self):
            # Update rate
            self.DT_CF = 0.001
            self.DT_ATT_PIDS = 0.002
            self.DT_POS_PIDS = 0.01
            self.DT_COMMUNICATION = 0.01

            # Thrust base used for the altitude algorithm in the firmware
            self.BASE_THRUST = 36000

            # PWM saturation points
            self.PWM_MAX = 65000
            self.PWM_MIN = 20000

            ####################
            # Drone parameters #
            ####################

            # Mass in grams
            self.DRONE_MASS = 0.027

            # Size of the propellersz
            self.ROTOR_SIZE = 23.1348e-3

            # Experimental constants
            self.KT = 0.2025
            self.KD = 0.11
            self.air_density = 1.225

            self.CT = self.KT*self.air_density*(2*self.ROTOR_SIZE)**4/3600
            self.CD = self.KD*self.air_density*(2*self.ROTOR_SIZE)**5/(2*np.pi*3600)

            # Intertia Matrix declaration
            # IXX = 1.3950e-05
            # IYY = 1.4360e-05
            # IZZ = 2.1730e-05
            # IXY = 0
            # IXZ = 0
            # IYZ = 0
            self.IXX = 16.5717e-06
            self.IYY = 26.6556e-06
            self.IZZ = 29.808e-06
            self.IXY = 1.2432e-06
            self.IXZ = 0.7168e-06
            self.IYZ = 2.0831e-06

            self.INERTIA_MATRIX = np.array([[self.IXX, self.IXY, self.IXZ], [self.IXY, self.IYY, self.IYZ], [self.IXZ, self.IYZ, self.IZZ]])
            self.INV_INERTIA_MATRIX = np.linalg.inv(self.INERTIA_MATRIX)
            # Arm length to the center of mass
            # CHECK THIS DISTANCE
            self.L = 39.73e-3

            # Gravity value and vector
            self.G = 9.81

            #########################################
            #### Motor distribution in single CF ####
            #########################################
            self.NUM_MOTORS = 4
            self.motors = np.zeros((self.NUM_MOTORS,3))

            # TODO: CHECK THIS TRIGONOMETRIC RELATION
            for i in range(self.NUM_MOTORS):
                self.motors[i][0] = self.L * cos((45 * np.pi / 180) + i * (90 * np.pi / 180))
                self.motors[i][1] = self.L * sin((45 * np.pi / 180) + i * (90 * np.pi / 180))