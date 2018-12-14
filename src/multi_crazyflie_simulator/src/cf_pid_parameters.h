#ifndef CF_PID_PARAMSH
#define CF_PID_PARAMSH

//##################
//## POSITION PID ##
//##################

//# X POS
#define KP_X 2
#define KI_X 0
#define KD_X 0.0
#define INT_MAX_X 5000.0
#define X_DT 0.01

//# Y POS
#define KP_Y 2
#define KI_Y 0.0
#define KD_Y 0.0
#define INT_MAX_Y 5000.0
#define Y_DT 0.01

//# Z POS
#define KP_Z 2.0
#define KI_Z 0.5
#define KD_Z 0.0
#define INT_MAX_Z 5000.0
#define Z_DT 0.01


//##################
//## VELOCITY PID ##
//##################

//# X VEL
#define KP_VX 25
#define KI_VX 1.0
#define KD_VX 0.0
#define VX_DT 0.01
#define INT_MAX_VX 5000.0
#define MAX_ROLL 20

//# Y VEL
#define KP_VY 25
#define KI_VY 1.0
#define KD_VY 0.0
#define VY_DT 0.01
#define INT_MAX_VY 5000.0
#define MAX_PITCH 20

//# Z VEL
#define KP_VZ 25.0
#define KI_VZ 15.0
#define KD_VZ 0.0
#define VZ_DT 0.01
#define INT_MAX_VZ 5000.0
#define THRUST_SCALE 1000

//#############
//## ATT PID ##
//#############

//# PITCH
#define KP_ROLL 6.0
#define KI_ROLL 3.0
#define KD_ROLL 0.0
#define INT_MAX_ROLL 20
#define ROLL_DT 0.002

//# PITCH
#define KP_PITCH 6.0
#define KI_PITCH 3.0
#define KD_PITCH 0.0
#define INT_MAX_PITCH 20
#define PITCH_DT 0.002

//# YAw
#define KP_YAW 6.
#define KI_YAW 1.
#define KD_YAW 0.0
#define INT_MAX_YAW 360.0
#define YAW_DT 0.002

#define MAX_ATT 25.0

//#################
//## Ang vel PID ##
//#################

//# WX VEL
#define KP_WX 250.0
#define KI_WX 500.0
#define KD_WX 0 //2.5
#define INT_MAX_WX 33.3
#define WX_DT 0.002

//# WY VEL
#define KP_WY 250.0
#define KI_WY 500.0
#define KD_WY 0 * 2.5
#define INT_MAX_WY 33.3
#define WY_DT 0.002

//# WZ VEL
#define KP_WZ 120.
#define KI_WZ 16.7
#define KD_WZ 0.0
#define INT_MAX_WZ 166.7
#define WZ_DT 0.002

#endif
