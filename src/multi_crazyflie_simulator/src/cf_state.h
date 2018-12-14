#include "Utils/CConstants.h"
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
#include "cf_physical_parameters.h"

class CF_state {
	public:
		CF_state();
		CF_state(int _num_motors);
		void getMotorRotationSpeed();
		void addMotorRotationSpeed();
		void getForces();
		void getMomentums();
public:
		cVector3d position;
		cVector3d lin_vel;
		cVector3d attitude;
		cVector3d attitude_deg;
		cVector3d ang_vel_deg;
		cVector3d ang_vel;
		int * motor_pwm;
		double * motor_rotation_speed;
		double sum_motor_rotations;
		cVector3d forces;
		cVector3d momentums;
		int num_motors; 
		double CT; 
		double CD; 
		double L; 
};

CF_state::CF_state() {
	position.zero();
	lin_vel.zero();
	attitude.zero();
	attitude_deg.zero();
	ang_vel_deg.zero();
	ang_vel.zero();
	forces.zero(); 
	momentums.zero();
	sum_motor_rotations = 0; 
	num_motors = CF_parameters().getNumMotors(); 
	motor_pwm = new int[num_motors]();
	motor_rotation_speed = new double[num_motors]();
	for (int i = 0; i < num_motors; i++) {
		motor_pwm[i] = 0;
		motor_rotation_speed[i] = 0.0;
	}
	CT = CF_parameters().CT;
	CD = CF_parameters().CD;
	L = CF_parameters().L;
}

void CF_state::getMotorRotationSpeed() {
	for (int i = 0; i < num_motors; i++) {
		motor_rotation_speed[i] = 0.2685*motor_pwm[i] + 4070.3;
	}
}
	
void CF_state::addMotorRotationSpeed() {
	sum_motor_rotations = 0; 
	for (int i = 0; i < num_motors; i++) {
		sum_motor_rotations += motor_rotation_speed[i] * motor_rotation_speed[i]; 
	}
}

void CF_state::getForces() {
	forces.set(0.0, 0.0, CT*sum_motor_rotations);
}

void CF_state::getMomentums() {
	momentums.x((L*CT/sqrt(2))*(-(motor_rotation_speed[0]* motor_rotation_speed[0]) - (motor_rotation_speed[1]* motor_rotation_speed[1]) + motor_rotation_speed[2]*motor_rotation_speed[2] + motor_rotation_speed[3]* motor_rotation_speed[3]));
	momentums.y((L*CT / sqrt(2))*(-(motor_rotation_speed[0] * motor_rotation_speed[0]) + (motor_rotation_speed[1] * motor_rotation_speed[1]) + motor_rotation_speed[2] * motor_rotation_speed[2] - motor_rotation_speed[3] * motor_rotation_speed[3]));
	momentums.z(CD*(-(motor_rotation_speed[0] * motor_rotation_speed[0]) + (motor_rotation_speed[1] * motor_rotation_speed[1]) - motor_rotation_speed[2] * motor_rotation_speed[2] + motor_rotation_speed[3] * motor_rotation_speed[3]));
}
