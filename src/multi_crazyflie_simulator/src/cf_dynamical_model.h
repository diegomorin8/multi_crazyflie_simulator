#include "Utils/CConstants.h"
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
#include "cf_physical_parameters.h"
#include "cf_state.h"
#include "cf_pid_handler.h"

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
		ros::Subscriber cmd_vel; 
		ros::Subscriber cmd_pos; 
		ros::Subscriber init_pose; 
		ros::Publisher  init_pose_ack;
		ros::Publisher  out_pos;

		geometry_msgs::PoseStamped msgPose; 

		// Frequency
		ros::Rate simulation_freq(1000);

		std::string topic; 

		bool isInit = false;
		int attPidCounter = 0;
		int posPidCounter = 0; 

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

		CF_pid_handler pidHandler; 
		std::string mode = "POS"; 

		cMatrix3d eulerMat; 
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
		void run();
	private:
		void eulerMatrix(double roll, double pitch, double yaw, cMatrix3d& a_matrix);
		void newInitPosition(geometry_msgs::PointStamped msg_in);
		void newVelCommand(geometry_msgs::Twist msg_in);
		void newPosCommand(crazyflie_driver::Position msg_in);
		void applySimulationStep();
		void publishPose(); 
		int min(double val1, double val2);
};

CF_model::CF_model(ros::NodeHandle* nodehandle, std::string topic_):nh_(*nodehandle){
	topic = topic_;

	cmd_vel = nh_.subscribe(self.topic + "/cmd_vel", 1000, newVelCommand);
	cmd_pos = nh_.subscribe(self.topic + "/cmd_pos", 1000, newPosCommand);
	init_pose = nh_.subscribe("/init_pose", 1000, newInitPosition);
	pub_ack = nh_.advertise<geometry_msgs::PoseStamped>(self.topic + "/out_pos", 1000);
	pub_pos = nh_.advertise<std_msgs::String>("/init_pose_ack", , 1000);
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
		msgID.data = position_msg.header.frame_id;
		pub_ack.publish(msgID);
	}
}

void CF_model::newVelCommand(geometry_msgs::Twist msg_in) {
	mode = "ATT";
	pidHandler.setVelCmd(msg_in.linear.y, -msg_in.linear.x, msg_in.angular.z, min(msg_in.linear.z, 60000));
}

void CF_model::newPosCommand(crazyflie_driver::Position msg_in) {
	mode = "POS";
	pidHandler.setPosCmd(msg_in.x, msg_in.y, msg_in.z, msg_in.yaw));
}

int CF_model::min(double val1, double val2) {
	if (val1 > val2) {
		return (int)val2;
	}
	else if (val1 <= val2) {
		return (int)val1;
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
	eulerMatrix(cf_state.attitude[0], self.cf_state.attitude[1], self.cf_state.attitude[2], eulerMat);

	cf_state.getMotorRotationSpeed();
	cf_state.addMotorsRotationsSpeed();
	cf_state.getForces();
	cf_state.getMomentums();

	//# Linear velocity
	rotation_matrix.mulr(cf_state.forces, new_state.lin_vel);
	new_state.lin_vel.div(cf_physical_params.DRONE_MASS);
	new_state.lin_vel.sub(new cVector3d(0, 0, cf_physical_params.G));

	//# Position
	new_state.position.copyfrom(new_state.lin_vel);

	//# Ang. velocity
	cVector3d preopA;
	cVector3d preopB;
	cf_physical_params.INERTIA_MATRIX.mulr(cf_state.ang_vel, preopA);
	cf_state.ang_vel.crossr(preopA, preopB);
	preopB.negate(); 
	preopB.add(cf_state.momentums);
	cf_physical_params.INV_INERTIA_MATRIX.mulr(preopB, new_state.ang_vel);

	//# Attitude
	euler_mat.mulr(cf_state.ang_vel, new_state.attitude);

	//# Integrate;
	new_state.position.mul(cf_physical_params.DT_CF);
	new_state.lin_vel.mul(cf_physical_params.DT_CF);
	new_state.attitude.mul(cf_physical_params.DT_CF);
	new_state.ang_vel.mul(cf_physical_params.DT_CF);
	cf_state.position.add(new_state.position);
	cf_state.lin_vel.add(new_state.lin_vel);
	cf_state.attitude.add(new_state.attitude);
	cf_state.ang_vel.add(new_state.ang_vel);

	//# Obtain degrees
	cf_state.attitude.mulr(C_RAD2DEG, cf_state.attitude_deg);
	cf_state.ang_vel.mulr(C_RAD2DEG, cf_state.ang_vel_deg);

	//# Ground constraints
	if (cf_state.position.z() <= 0) {
		cf_state.position.z(0.0);
		if (cf_state.lin_vel.z() <= 0){
			cf_state.lin_vel.zero(); 
			cf_state.attitude.x(0.0);
			cf_state.attitude.y(0.0);
			cf_state.ang_vel.zero(); 
		}
	}

}

//################
//# PUBLISH POSE #
//################

void CF_state::publishPose(){
	msgPose.header.frame_id = "/base_link";
	msgPose.pose.position.x = cf_state.position.x();
	msgPose.pose.position.y = cf_state.position.y();
	msgPose.pose.position.z = cf_state.position.z();
	tf::Quaternion quaternion;
	tf::Vector3 position; 
	tf::Transform transform;
	position.setValue(cf_state.position.x(), cf_state.position.y(), cf_state.position.z());
	quaternion.setRPY(cf_state.attitude.x(), cf_state.attitude.y(), cf_state.attitude.z());
	transform.setRotation(quaternion);
	
	msgPose.pose.orientation.w = quaternion[3];
	msgPose.pose.orientation.x = quaternion[0];
	msgPose.pose.orientation.y = quaternion[1];
	msgPose.pose.orientation.z = quaternion[2];
	pub_pos.publish(msgPose);
	static tf::TransformBroadcaster br;

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), self.topic + "/base_link", "/base_link"));
}

void CF_state::run() {
	while (ros::ok()) {
		if (isInit) {
			applySimulationStep();
			if (attPidCounter == pidHandler.attPidCounterMax()) {
				runAttPid(); 
				runAngVelPid(); 
				attPidCounter = 0;
			}
			else {
				attPidCounter++;
			}
			if (posPidCounter == pidHandler.posPidCounterMax()) {
				if(mode == "POS"){
					runPosPid()
					runLinVelPid()
				}
				publishPose()
				posPidCounter = 0; 
			}
		}
	}
}
