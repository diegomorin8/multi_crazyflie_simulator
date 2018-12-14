#ifndef CF_ParametersH
#define CF_ParametersH
//-------------------------------------------------------------------
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
//-------------------------------------------------------------------
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
    //#########################################

    double CT = KT*air_density*pow(2*ROTOR_SIZE, 4)/3600;
    double CD = KD*air_density*pow(2*ROTOR_SIZE, 5)/(2*3600*M_PI);
	
    // Intertia Matrix declaration
    double IXX = 16.5717e-06;
    double IYY = 26.6556e-06;
    double IZZ = 29.808e-06;
    double IXY = 1.2432e-06;
    double IXZ = 0.7168e-06;
    double IYZ = 2.0831e-06;

	cMatrix3d INERTIA_MATRIX = cMatrix3d(IXX, IXY, IXZ, IXY, IYY, IYZ, IXZ, IYZ, IZZ);
	cMatrix3d INV_INERTIA_MATRIX;

    // Arm length to the center of mass
    double L = 39.73e-3;

    //Gravity value and vector
    double G = 9.81;

  private: 
    static const int NUM_MOTORS = 4;

  public:
	cVector3d motors[NUM_MOTORS];


public:

   CF_parameters();
   int getNumMotors();
};

CF_parameters::CF_parameters(){

	INERTIA_MATRIX.invertr(INV_INERTIA_MATRIX);

    for (int i = 0; i < NUM_MOTORS; i++){
		motors[i] = cVector3d(L * cos((45 * M_PI / 180.0) + i * (90.0 * M_PI / 180)), L * sin((45 * M_PI / 180.0) + i * (90.0 * M_PI / 180)), 0.0f); 
    }
}

int CF_parameters::getNumMotors(){
   return NUM_MOTORS;
}

#endif

