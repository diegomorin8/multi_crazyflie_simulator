#include "Utils/CConstants.h"
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
#include "cf_physical_parameters.h"
#include "cf_pid.h"

class CF_pid_handler {
	public:
		CF_pid_handler();
	public: 
		void setVelCmd(double roll, double pitch, double yawRate, int thrust);
		void setVelCmd(double x, double y, double z, double yaw);
		void runPosPID(double currentX, double currentY, double currentZ);
		void runLinVelPID(double currentVX, double currentVY, double currentVZ, double currentYaw);
		void runAttPID(double currentRoll, double currentPitch);
		void runAngVelPID(double currentWX, double currentWY, double currentWZ, double* a_PWM);

	private: 
		double min(double val1, double val2);
		double max(double val1, double val2);
		int limitPWM(double value);
		void rpyt2MotorPWM(double* a_PWM);

	private:

		//######################
		//# Initialize PID
		//######################
		double desiredThrust = 0.0; 

		//# Out from the PIDs, values of
		//# r, p, y, thrust
		cVector3d desiredRPY;

		//######################
		//# Angular velocities
		//######################
		PID wPID(KP_WX, KI_WX, KD_WX, INT_MAX_WX, WX_DT);
		PID wyPID(KP_WY, KI_WY, KD_WY, INT_MAX_WY, WY_DT);
		PID wzPID(KP_WZ, KI_WZ, KD_WZ, INT_MAX_WZ, WZ_DT);
		cVector3d desiredAngVel;

		//######################                                                  
		//# Attitudes
		//######################
		PID rollPID(KP_ROLL, KI_ROLL, KD_ROLL, INT_MAX_ROLL, ROLL_DT);
		PID pitchPID(KP_PITCH, KI_PITCH, KD_PITCH, INT_MAX_PITCH, PITCH_DT);
		PID yawPID(KP_YAW, KI_YAW, KD_YAW, INT_MAX_YAW, YAW_DT);
		cVector3d desiredAtt; 
		int attPidCounterMax = (int)(cf_physical_params.DT_ATT_PIDS / cf_physical_params.DT_CF) - 1;

		//######################
		//# Velocities
		//######################
		PID vxPID(KP_VX, KI_VX, KD_VX, INT_MAX_VX, VX_DT);
		PID vyPID(KP_VY, KI_VY, KD_VY, INT_MAX_VY, VY_DT);
		PID vzPID(KP_VZ, KI_VZ, KD_VZ, INT_MAX_VZ, VZ_DT);
		cVector3d desiredLinVel;

		//######################
		//# Positions
		//######################
		PID vxPID(KP_X, KI_X, KD_X, INT_MAX_X, X_DT);
		PID vyPID(KP_Y, KI_Y, KD_Y, INT_MAX_Y, Y_DT);
		PID vzPID(KP_Z, KI_Z, KD_Z, INT_MAX_Z, Z_DT);
		cVector3d desiredPos;
		int posPidCounterMax = (int)(cf_physical_params.DT_POS_PIDS / cf_physical_params.DT_CF) - 1;

}

//#######################
//# New setpoints
//#######################
void CF_pid_handler::setVelCmd(double roll, double pitch, double yawRate, int thrust) {
	desiredAtt.set(roll, pitch, 0.0);
	desiredAngVel.z(yawRate);
	desiredThrust = thrust; 
}

void CF_pid_handler::setVelCmd(double x, double y, double z, double yaw) {
	desiredPos.set(x, y, z);
	desiredAtt.z(yaw);
}

double CF_pid_handler::min(double val1, double val2) {
	if (val1 > val2) {
		return val2;
	}
	else if (val1 <= val2) {
		return val1;
	}
}

double CF_pid_handler::max(double val1, double val2) {
	if (val1 > val2) {
		return val1;
	}
	else if (val1 <= val2) {
		return val2;
	}
} 

int CF_pid_handler::limitPWM(double value) {
	return (int)max(min(cf_physical_params.PWM_MAX, value), cf_physical_params.PWM_MIN);
}
	
void CF_pid_handler::runPosPID(double currentX, double currentY, double currentZ) {
	desiredLinVel.set(	xPID.update(desired_pos.x(), currentX),
						yPID.update(desired_pos.y(), currentY),
						zPID.update(desired_pos.z(), currentZ));
}

void CF_pid_handler::runLinVelPID(double currentVX, double currentVY, double currentVZ, double currentYaw) {
	double rawPitch = vyPID.update(desiredLinVel.y(), currentVY);
	double rawRoll = vxPID.update(self.desiredLinVel.x(), currentVX);

	//# Current YAW
	double rawYaw = -cf_state.attitude.z();

	//# Transformation to the drone body frame
	desiredAtt.y() = -(rawRoll * cos(rawYaw)) - (rawPitch * sin(rawYaw));
	desiredAtt.x() = -(rawPitch * cos(rawYaw)) + (rawRoll * sin(rawYaw));

	desiredAtt.x() = max(min(MAX_ATT, desiredAtt.x()), -MAX_ATT);
	desiredAtt.y() = max(min(MAX_ATT, desiredAtt.y()), -MAX_ATT);

	double rawThrust = vzPID.update(desiredLinVel.z(), currentVZ);
	desiredThrust = (int)max((rawThrust * 1000 + cf_physical_params.BASE_THRUST), cf_physical_params.PWM_MIN);

	desiredAngVel.z(yawPID.update(self.desiredAtt.z(), currentYaw));

}

void CF_pid_handler::runAttPID(double currentRoll, double currentPitch) {

	desiredAngVel.x(rollPID.update(desiredAtt.x(), currentRoll);
	desiredAngVel.y(pitchPID.update(-desiredAtt.y(), currentPitch));

void CF_pid_handler::runAngVelPID(double currentWX, double currentWY, double currentWZ, double* a_PWM) {
	desiredRPY.set(wxPID.update(desiredAngVel.x(), currentWX),
		-wyPID.update(desiredAngVel.y(), currentWY),
		-wzPID.update(desiredAngVel.z(), currentWZ));
	rpyt2MotorPWM(a_PWM);
}

void CF_pid_handler::rpyt2MotorPWM(double* a_PWM) {
	//##########################
	//# Function that transform the output
	//# r, p, y, thrust of the PIDs,
	//# into the values of the PWM
	//# applied to each motor
	//##########################

	double R = r / 2.0
	double P = p / 2.0
	double Y = y

	a_PWM[0] = limit(desiredThrust - R + P + Y));
	a_PWM[1] = limit(desiredThrust - R - P - Y));
	a_PWM[2] = limit(desiredThrust + R - P + Y));
	a_PWM[3] = limit(desiredThrust + R + P - Y));
}