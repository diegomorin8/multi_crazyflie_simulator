#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class CF_parameters{

  public:

    double DT_CF = 0.001;
    double DT_ATT_PIDS = 0.002;
    double DT_POS_PIDS = 0.01;
    double DT_COMMUNICATION = 0.01;

    // Thrust base used for the altitude algorithm in the firmware
    double BASE_THRUST = 36000;

    // PWM saturation points
    double PWM_MAX = 65000;
    double PWM_MIN = 20000;

    //###################
    // Drone parameters #
    //###################

    // Mass in grams
    double DRONE_MASS = 0.027;

    // Size of the propellersz
    double ROTOR_SIZE = 23.1348e-3;

    // Experimental constants
    double KT = 0.2025;
    double KD = 0.11;
    double air_density = 1.225;

    //#########################################
    //#### Motor distribution in single CF ####
    #########################################

    double CT = KT*air_density*pow(2*ROTOR_SIZE, 4)/3600;
    double CD = KD*air_density*pow(2*ROTOR_SIZE, 5)/(2*3600*_PI));

    double INERTIA_MATRIX[3][3] = {{IXX, IXY, IXZ},{IXY, IYY, IYZ}, {IXZ, IYZ, IZZ}};

    // Intertia Matrix declaration
    double IXX = 16.5717e-06;
    double IYY = 26.6556e-06;
    double IZZ = 29.808e-06;
    double IXY = 1.2432e-06;
    double IXZ = 0.7168e-06;
    double IYZ = 2.0831e-06;

    // Arm length to the center of mass
    double L = 39.73e-3;

    //Gravity value and vector
    double G = 9.81;
    int NUM_MOTORS = 4;

    float motors[NUM_MOTORS][3];

public:

   CF_parameters();
}

CF_parameters::CF_parameters(){
      for (int i = 0; i < NUM_MOTORS; i++){
          motors[i][0] = L * cos((45 * M_PI / 180.0) + i * (90.0 * M_PI / 180));
          motors[i][1] = L * sin((45 * M_PI / 180.0) + i * (90.0 * M_PI / 180));
          motors[i][2] = 0;
      }
}


    self.CT = self.KT*self.air_density*(2*self.ROTOR_SIZE)**4/3600
    self.CD = self.KD*self.air_density*(2*self.ROTOR_SIZE)**5/(2*np.pi*3600)



    self.INERTIA_MATRIX = np.array([[self.IXX, self.IXY, self.IXZ], [self.IXY, self.IYY, self.IYZ], [self.IXZ, self.IYZ, self.IZZ]])
    self.INV_INERTIA_MATRIX = np.linalg.inv(self.INERTIA_MATRIX)


    self.motors = np.zeros((self.NUM_MOTORS,3))

    # TODO: CHECK THIS TRIGONOMETRIC RELATION
