class PID{

	public:
		PID(double _kp, double _ki, double  _kd, double  _max_i_value, double  _dt); 
		double update(double desired, double measurement);

	private: 
		double constrain(double val, double min_vale, double max_val);

	private: 
		double kp;
		double ki;
		double kd;
		double max_i_value;
		double dt;
		double prev_error = 0.0f;
		double integ = 0.0f;
};

PID::PID(double _kp, double _ki, double  _kd, double  _max_i_value, double  _dt) {
	kp = _kp;
	ki = _ki;
	kd = _kd;
	max_i_value = _max_i_value;
	dt = _dt;
}

double PID::constrain(double val, double min_val, double max_val) {
	if (val < min_val) return min_val;
	if (val > max_val) return max_val;
	return val;
}

double PID::update(double desired, double measurement) {
	
	//The error between the desired state and the measured one
	double error = desired - measurement;
	
	//########################
	//# PID EQUATIONS
	//########################

	//#P:
	double output = 0;
	output += kp * error;

	//#D :
	output += kd * (error - prev_error) / dt;

	//#I
	integ += error * dt;
	if (max_i_value != 0) integ = constrain(integ, -max_i_value, max_i_value);

	output += ki * integ;

	prev_error = error;

	return output;
}


