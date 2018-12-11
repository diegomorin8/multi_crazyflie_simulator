#include "Utils/CConstants.h"
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
#include "cf_physical_parameters.h"
#include "cf_state.h"
#include "cf_pid_parameters.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/String.h"
#include "crazyflie_driver/Position.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

class CF_model {
	public:
		CF_model(ros::NodeHandle* nodehandle);
	
	private:

		// put private member data here;  "private" data will only be available to member functions of this class;
		ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

		// some objects to support subscriber, service, and publisher
		ros::Subscriber cmd_vel; //these will be set up within the class constructor, hiding these ugly details
		ros::Subscriber cmd_pos; //these will be set up within the class constructor, hiding these ugly details
		ros::Subscriber init_pose; //these will be set up within the class constructor, hiding these ugly details
		ros::Publisher  out_pos;
		ros::Publisher  init_pose_ack;

		geometry_msgs::PoseStamped msg; 

		// Frequency
		ros::Rate simulation_freq;

		std::string topic; 

		bool isInit = false; 

		// System state : position, linear velocities,
		// attitude and angular velocities
		CF_state cf_state; 

		//# Import the crazyflie physical paramters
		//#     - These parameters are obtained from different sources.
		//#     - For these parameters and the dynamical equations refer
		//#       to : DESIGN OF A TRAJECTORY TRACKING CONTROLLER FOR A
		//#            NANOQUADCOPTER
		//#            Luis, C., &Le Ny, J. (August, 2016)
		CF_parameters cf_physical_parameters;

		// For sync purposes
		int out_pos_counter = 0; 
		int out_pos_counter_max;

		std::string = "POS"; 

		cMatrix3d eulerMatrix; 
		cMatrix3d invRotationMatrix; 

		enum cEulerOrder
		{
			C_EULER_ORDER_XYZ,
			C_EULER_ORDER_XYX,
			C_EULER_ORDER_XZY,
			C_EULER_ORDER_XZX,
			C_EULER_ORDER_YZX,
			C_EULER_ORDER_YZY,
			C_EULER_ORDER_YXZ,
			C_EULER_ORDER_YXY,
			C_EULER_ORDER_ZXY,
			C_EULER_ORDER_ZXZ,
			C_EULER_ORDER_ZYX,
			C_EULER_ORDER_ZYZ
		};
	
	public:
		void eulerMatrix(double roll, double pitch, double yaw, cMatrix3d& a_matrix);
	
	private:
		void newInitPosition(geometry_msgs::PointStamped msg_in);
		void applySimulationStep();


};

CF_model::CF_model(ros::NodeHandle* nodehandle):nh_(*nodehandle){

}
void CF_model::eulerMatrix(double roll, double pitch, double yaw, cMatrix3d& a_matrix) {
	double cos_roll = cos(roll);
	double sin_roll = sin(roll);

	double cos_pitch = cos(pitch) + 1e-12;
	double tan_pitch = tan(pitch);

	a_matrix.set(	1, sin_roll * tan_pitch, cos_roll * tan_pitch,
					0, cos_roll, -sin_roll,
					0, sin_roll / cos_pitch, cos_roll / cos_pitch);
}

void CF_model::newInitPosition(geometry_msgs::PointStamped msg_in) {
	
	if (topic == msg_in.header.frame_id) {
		cf_state.position.set(position_msg.point.x, position_msg.point.y, position_msg.point.z);
		
		//TODO: send to PID
		//self.desired_pos = np.array([position_msg.point.x, position_msg.point.y, position_msg.point.z])
		
		isInit = true;
		std_msgs::String msgID;
		msgID = position_msg.header.frame_id;
		pub_ack.publish(msgID);
	}
}

//###########################
//# Single step simulation
//###########################
void CF_model::applySimulationStep() {

	//###########################
	//# Main simulation loop
	//#   - The CF works at a rate of 1000Hz,
	//#     in the same way, we are simulating
	//#     at the same frequency
	//###########################

	//# New simulated state
	CF_state new_state = CF_state(); 

	//# Matrices
	invRotationMatrix.setExtrinsicEulerRotationRad(cf_state.attitude[0], self.cf_state.attitude[1], self.cf_state.attitude[2], cEulerOrder.C_EULER_ORDER_ZYX);
	eulerMatrix(cf_state.attitude[0], self.cf_state.attitude[1], self.cf_state.attitude[2], eulerMatrix);

	cf_state.getMotorRotationSpeed();
	cf_state.addMotorsRotationsSpeed();
	cf_state.getForces();
	cf_state.getMomentums();


	new_state.lin_vel = np.dot(np.linalg.inv(rotation_matrix), self.cf_state.forces) / self.cf_physical_params.DRONE_MASS - np.array([0, 0, self.cf_physical_params.G])#    - np.cross(self.cf_state.ang_vel, self.cf_state.lin_vel)

}



	

	

	

	new_state.lin_vel = np.dot(np.linalg.inv(rotation_matrix), self.cf_state.forces) / self.cf_physical_params.DRONE_MASS - np.array([0, 0, self.cf_physical_params.G])#   - np.cross(self.cf_state.ang_vel, self.cf_state.lin_vel)

	new_state.position = self.cf_state.lin_vel

	preoperation = self.cf_state.momentums - np.cross(self.cf_state.ang_vel,
		np.dot(self.cf_physical_params.INERTIA_MATRIX,
			self.cf_state.ang_vel))
	new_state.ang_vel = np.dot(self.cf_physical_params.INV_INERTIA_MATRIX, preoperation)

	new_state.attitude = np.dot(euler_matrix, self.cf_state.ang_vel)

	for i in range(0, 3) :
		self.cf_state.position[i] = self.cf_state.position[i] + (new_state.position[i] * self.cf_physical_params.DT_CF)
		self.cf_state.attitude[i] = self.cf_state.attitude[i] + (new_state.attitude[i] * self.cf_physical_params.DT_CF)
		self.cf_state.lin_vel[i] = self.cf_state.lin_vel[i] + (new_state.lin_vel[i] * self.cf_physical_params.DT_CF)
		self.cf_state.ang_vel[i] = self.cf_state.ang_vel[i] + (new_state.ang_vel[i] * self.cf_physical_params.DT_CF)

		self.cf_state.ang_vel_deg = self.cf_state.ang_vel*180.0 / np.pi
		self.cf_state.attitude_deg = self.cf_state.attitude*180.0 / np.pi

		# Ground constraints
		if self.cf_state.position[2] <= 0:
self.cf_state.position[2] = 0
if self.cf_state.lin_vel[2] <= 0 :
	self.cf_state.lin_vel[:] = 0.
	self.cf_state.attitude[:-1] = 0.
	self.cf_state.ang_vel[:] = 0.

class CF_model() :

	def __init__(self) :

	rospy.init_node("dynamic_model", anonymous = True)
	self.topic = rospy.get_param("~topic")

	rospy.Subscriber(self.topic + "/cmd_vel", Twist, self.new_attitude_setpoint)
	rospy.Subscriber(self.topic + "/cmd_pos", Position, self.new_position_setpoint)
	rospy.Subscriber("/init_pose", PointStamped, self.new_init_position)
	self.pub_pos = rospy.Publisher(self.topic + "/out_pos", PoseStamped, queue_size = 1000)
	self.pub_ack = rospy.Publisher("/init_pose_ack", String, queue_size = 1000)
	self.msg = PoseStamped()


	# Main CF variables initialization(if needed)
	self.simulation_freq = rospy.Rate(int(1 / self.cf_physical_params.DT_CF))

############################
	# Communication control
############################
	self.out_pos_counter = 0
	self.out_pos_counter_max = int(self.cf_physical_params.DT_POS_PIDS / self.cf_physical_params.DT_CF) - 1

	self.mode = "POS";

		



			

	def run_pos_pid(self) :
	self.desired_lin_vel = np.array([self.x_pid.update(self.desired_pos[0], self.cf_state.position[0]),
		self.y_pid.update(self.desired_pos[1], self.cf_state.position[1]),
		self.z_pid.update(self.desired_pos[2], self.cf_state.position[2])])

	def run_lin_vel_pid(self) :
	raw_pitch = self.vy_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1])
	raw_roll = self.vx_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0])

	# Current YAW
	raw_yaw = -self.cf_state.attitude[2]

	# Transformation to the drone body frame
	self.desired_att[1] = -(raw_roll * cos(raw_yaw)) - (raw_pitch * sin(raw_yaw))
	self.desired_att[0] = -(raw_pitch * cos(raw_yaw)) + (raw_roll * sin(raw_yaw))

	self.desired_att[0] = max(min(self.cf_pid_gains.MAX_ATT, self.desired_att[0]), -self.cf_pid_gains.MAX_ATT)
	self.desired_att[1] = max(min(self.cf_pid_gains.MAX_ATT, self.desired_att[1]), -self.cf_pid_gains.MAX_ATT)

	raw_thrust = self.vz_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])

	self.desired_thrust = max((raw_thrust * 1000 + self.cf_physical_params.BASE_THRUST), self.cf_physical_params.PWM_MIN)

	def run_att_pid(self) :
	self.desired_ang_vel = np.array([self.roll_pid.update(self.desired_att[0], self.cf_state.attitude_deg[0]),
		self.pitch_pid.update(-self.desired_att[1], self.cf_state.attitude_deg[1]),
		self.yaw_pid.update(self.desired_att[2], self.cf_state.attitude_deg[2])])

	def run_ang_vel_pid(self) :
	self.desired_rpy = np.array([self.wx_pid.update(self.desired_ang_vel[0], self.cf_state.ang_vel_deg[0]),
		-self.wy_pid.update(self.desired_ang_vel[1], self.cf_state.ang_vel_deg[1]),
		-self.wz_pid.update(self.desired_ang_vel[2], self.cf_state.ang_vel_deg[2])])
	self.rpyt_2_motor_pwm()

	def log_state(self) :

	rospy.loginfo("\n Nuevo estado: \n Position: " + str(self.cf_state.position)
		+ "\n Lin. Velocity: " + str(self.cf_state.lin_vel)
		+ "\n Attitude: " + str(self.cf_state.attitude * 180 / np.pi)
		+ "\n Ang. velocities: " + str(self.cf_state.ang_vel * 180 / np.pi)
		+ "\n Delta time: " + str(sum(self.delta_time) / len(self.delta_time))
		+ "\n Processing time " + str(sum(self.time_processing) / len(self.time_processing)))
	self.time_processing = []
	self.delta_time = []

	def rpyt_2_motor_pwm(self) :

	# Inputs
	r = self.desired_rpy[0]
	p = self.desired_rpy[1]
	y = self.desired_rpy[2]
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

	self.cf_state.motor_pwm[0] = int(self.limit(thrust - R + P + Y))
	self.cf_state.motor_pwm[1] = int(self.limit(thrust - R - P - Y))
	self.cf_state.motor_pwm[2] = int(self.limit(thrust + R - P + Y))
	self.cf_state.motor_pwm[3] = int(self.limit(thrust + R + P - Y))

	def publishPose(self) :
	self.msg.header.frame_id = "/base_link"
	self.msg.pose.position.x = self.cf_state.position[0]
	self.msg.pose.position.y = self.cf_state.position[1]
	self.msg.pose.position.z = self.cf_state.position[2]
	quaternion = tf.transformations.quaternion_from_euler(self.cf_state.attitude[0], self.cf_state.attitude[1], self.cf_state.attitude[2])
	self.msg.pose.orientation.w = quaternion[3]
	self.msg.pose.orientation.x = quaternion[0]
	self.msg.pose.orientation.y = quaternion[1]
	self.msg.pose.orientation.z = quaternion[2]
	self.pub_pos.publish(self.msg)
	br = tf.TransformBroadcaster()
	br.sendTransform(self.cf_state.position,
		quaternion,
		rospy.Time.now(),
		self.topic + "/base_link",
		"/base_link")

	def run(self) :
	tic_init = time.time()
	while (not rospy.is_shutdown()) :
		if (self.isInit) :
			self.delta_time.append(time.time() - tic_init)
			tic_init = time.time()
			self.apply_simulation_step()

			if (self.att_pid_counter == self.att_pid_counter_max) :
				self.att_pid_counter = 0
				self.run_att_pid()
				self.run_ang_vel_pid()

			else:
self.att_pid_counter = self.att_pid_counter + 1

if (self.out_pos_counter == self.out_pos_counter_max) :
	self.out_pos_counter = 0
	self.run_pos_pid()
	self.run_lin_vel_pid()
	self.publishPose()
	#self.log_state()
else:
self.out_pos_counter = self.out_pos_counter + 1

self.time_processing.append(time.time() - tic_init)
# Wait for the cycle left time
self.simulation_freq.sleep()


if __name__ == '__main__':
model = CF_model()

model.run()
rospy.spin()


### PARA METER VALORES POR TECLADO EN cmd_hover:
### rostopic pub /cmd_hover crazyflie_driver/Hover '{vx: 0.0, vy: 0.0, yawrate: 0.0 , zDistance: 0.0}'
