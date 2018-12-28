#define DEBUG_CMD true
#define DEBUG_DATA false
#define DEBUG_TEST false

#include "Utils/CConstants.h"
#include "Utils/CMatrix3d.h"
#include "Utils/CVector3d.h"
#include "cf_physical_parameters.h"
#include "cf_state.h"
#include "cf_pid_handler.h"

#include <boost/thread/thread.hpp>

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
		CF_model(ros::NodeHandle* nodehandle, std::string topic_);
	
	public: 
		ros::Subscriber cmd_vel; 
		ros::Subscriber cmd_pos; 
		ros::Subscriber init_pose;
 
	private:

		// put private member data here;  "private" data will only be available to member functions of this class;
		ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

		// some objects to support subscriber, service, and publisher
		ros::Publisher  pub_ack;
		ros::Publisher  pub_pos;

		geometry_msgs::PoseStamped msgPose; 

		// Frequency
		ros::Rate simulation_freq = ros::Rate(1000);

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
		CF_parameters cf_physical_params;

		CF_pid_handler pidHandler; 
		std::string mode = "POS"; 

		cMatrix3d eulerMat; 
		cMatrix3d invRotationMatrix; 

		
	
	public:
		void runDynamics();
		void newInitPosition(const geometry_msgs::PointStamped& msg_in);
		void newVelCommand(const geometry_msgs::Twist& msg_in);
		void newPosCommand(const crazyflie_driver::Position& msg_in);
	
	private:
		void run();
		void eulerMatrix(double roll, double pitch, double yaw, cMatrix3d& a_matrix);
		void applySimulationStep();
		void publishPose(); 
		int min(double val1, double val2);
		void runPosPids(); 
		void runAttPids(); 
};

CF_model::CF_model(ros::NodeHandle* nodehandle, std::string topic_):nh_(*nodehandle){	
	topic = topic_;

	ROS_INFO("Dynamical model init for: %s", topic.c_str());

	pidHandler = CF_pid_handler();
	cmd_vel = nh_.subscribe(topic + "/cmd_vel", 1000, &CF_model::newVelCommand, this);
	cmd_pos = nh_.subscribe(topic + "/cmd_pos", 1000, &CF_model::newPosCommand, this);
	init_pose = nh_.subscribe("/init_pose", 1000, &CF_model::newInitPosition, this);
	pub_pos = nh_.advertise<geometry_msgs::PoseStamped>(topic + "/out_pos", 1000);
	pub_ack = nh_.advertise<std_msgs::String>("/init_pose_ack" , 1000);
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

void CF_model::newInitPosition(const geometry_msgs::PointStamped& msg_in) {

#if DEBUG_CMD
	ROS_INFO("[CF_MODEL]: New init position for %s - x: %f - y: %f - z: %f",topic.c_str(), msg_in.point.x, msg_in.point.y, msg_in.point.z);
#endif

	if (topic == msg_in.header.frame_id) {
		cf_state.position.set(msg_in.point.x, msg_in.point.y, msg_in.point.z);
		
		pidHandler.setPosCmd(msg_in.point.x, msg_in.point.y, msg_in.point.z, 0.0);
		
		isInit = true;
		std_msgs::String msgID;
		msgID.data = msg_in.header.frame_id;
		pub_ack.publish(msgID);
	}
}

void CF_model::newVelCommand(const geometry_msgs::Twist& msg_in) {

#if DEBUG_CMD
	ROS_INFO("[CF_MODEL]: New vel command for %s - ROLL: %f  - PITCH: %f - W_YAW: %f - THRUST: %f", topic.c_str(), msg_in.linear.y, -msg_in.linear.x, msg_in.angular.z, msg_in.linear.z); 
#endif

	mode = "ATT";
	pidHandler.setVelCmd(msg_in.linear.y, -msg_in.linear.x, msg_in.angular.z, min(msg_in.linear.z, 60000));
}

void CF_model::newPosCommand(const crazyflie_driver::Position& msg_in) {

#if DEBUG_CMD
	ROS_INFO("[CF_MODEL]: New pos command for %s - x: %f  - y: %f - z: %f - Yaw: %f", topic.c_str(), msg_in.x, msg_in.y, msg_in.z, msg_in.yaw); 
#endif

	mode = "POS";
	pidHandler.setPosCmd(msg_in.x, msg_in.y, msg_in.z, msg_in.yaw);
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

    //cf_state.motor_pwm = new int[4]{36050, 36050, 36000, 36000};
    //cf_state.position.set(0.0, 0.0, 1.0);
#if DEBUG_TEST
    cf_state.motor_pwm = new int[4]{36432, 12345, 63400, 23406};
    cf_state.position.set(0.2, 1, 1.21);
    cf_state.lin_vel.set(2.1, 0.2, -3.01);
    cf_state.attitude.set(0.12, -0.08, 0.23);
    cf_state.ang_vel.set(-0.5, 1.2, -0.7);
#endif
	
	//# Matrices
	invRotationMatrix.setExtrinsicEulerRotationRad(cf_state.attitude.x(), cf_state.attitude.y(), cf_state.attitude.z(), C_EULER_ORDER_XYZ);

#if DEBUG_DATA
	ROS_INFO("Rotation matrix : [[ %f, %f, %f ],[ %f, %f, %f ],[ %f, %f, %f ]]", invRotationMatrix.get(0,0), invRotationMatrix.get(0,1),invRotationMatrix.get(0,2),invRotationMatrix.get(1,0),invRotationMatrix.get(1,1),invRotationMatrix.get(1,2),invRotationMatrix.get(2,0),invRotationMatrix.get(2,1),invRotationMatrix.get(2,2));
#endif
	
	eulerMatrix(cf_state.attitude.x(), cf_state.attitude.y(), cf_state.attitude.z(), eulerMat);
#if DEBUG_DATA
	ROS_INFO("Euler matrix : [[ %f, %f, %f ],[ %f, %f, %f ],[ %f, %f, %f ]]", eulerMat.get(0,0), eulerMat.get(0,1),eulerMat.get(0,2),eulerMat.get(1,0),eulerMat.get(1,1),eulerMat.get(1,2),eulerMat.get(2,0),eulerMat.get(2,1),eulerMat.get(2,2));
#endif
	cf_state.getMotorRotationSpeed();
	cf_state.addMotorRotationSpeed();
	cf_state.getForces();
	cf_state.getMomentums();

#if DEBUG_DATA
	ROS_INFO("Forces : [ %f, %f, %f ]", cf_state.forces.x(), cf_state.forces.y(), cf_state.forces.z());
    ROS_INFO("Momentum : [ %f, %f, %f ]", cf_state.momentums.x(), cf_state.momentums.y(), cf_state.momentums.z());
#endif

	//# Linear velocity
	invRotationMatrix.mulr(cf_state.forces, new_state.lin_vel);

	new_state.lin_vel.div(cf_physical_params.DRONE_MASS);
	new_state.lin_vel.sub(cVector3d(0, 0, cf_physical_params.G));

#if DEBUG_DATA
	ROS_INFO("New lin_vel : [ %f, %f, %f ]", new_state.lin_vel.x(), new_state.lin_vel.y(), new_state.lin_vel.z());
#endif

	//# Position
	new_state.position.copyfrom(cf_state.lin_vel);
#if DEBUG_DATA
	ROS_INFO("New position : [ %f, %f, %f ]", new_state.position.x(), new_state.position.y(), new_state.position.z());
#endif

	//# Ang. velocity
	cVector3d preopA;
	cVector3d preopB;
	cf_physical_params.INERTIA_MATRIX.mulr(cf_state.ang_vel, preopA);
	cf_state.ang_vel.crossr(preopA, preopB);
	preopB.negate(); 
	preopB.add(cf_state.momentums);
	cf_physical_params.INV_INERTIA_MATRIX.mulr(preopB, new_state.ang_vel);

	//# Attitude
	eulerMat.mulr(cf_state.ang_vel, new_state.attitude);
#if DEBUG_DATA
	ROS_INFO("New attitude : [ %f, %f, %f ]", new_state.attitude.x(), new_state.attitude.y(), new_state.attitude.z());
#endif

#if DEBUG_DATA
	ROS_INFO("New ang_vel : [ %f, %f, %f ]", new_state.ang_vel.x(), new_state.ang_vel.y(), new_state.ang_vel.z());
#endif

	//# Integrate;
	new_state.position.mul(cf_physical_params.DT_CF);
	new_state.lin_vel.mul(cf_physical_params.DT_CF);
	new_state.attitude.mul(cf_physical_params.DT_CF);
	new_state.ang_vel.mul(cf_physical_params.DT_CF);
	cf_state.position.add(new_state.position);
	cf_state.lin_vel.add(new_state.lin_vel);
	cf_state.attitude.add(new_state.attitude);
	cf_state.ang_vel.add(new_state.ang_vel);

#if DEBUG_DATA
	ROS_INFO("Final position : [ %f, %f, %f ]", cf_state.position.x(), cf_state.position.y(), cf_state.position.z());
	ROS_INFO("Final lin_vel : [ %f, %f, %f ]", cf_state.lin_vel.x(), cf_state.lin_vel.y(), cf_state.lin_vel.z());
	ROS_INFO("Final attitude : [ %f, %f, %f ]", cf_state.attitude.x(), cf_state.attitude.y(), cf_state.attitude.z());
	ROS_INFO("Final ang_vel : [ %f, %f, %f ]", cf_state.ang_vel.x(), cf_state.ang_vel.y(), cf_state.ang_vel.z());
#endif

	//# Obtain degrees
	cf_state.attitude.mulr(C_RAD2DEG, cf_state.attitude_deg);
	cf_state.ang_vel.mulr(C_RAD2DEG, cf_state.ang_vel_deg);

#if DEBUG_DATA
	ROS_INFO("Final attitude_deg : [ %f, %f, %f ]", cf_state.attitude_deg.x(), cf_state.attitude_deg.y(), cf_state.attitude_deg.z());
	ROS_INFO("FInal ang_vel_deg : [ %f, %f, %f ]", cf_state.ang_vel_deg.x(), cf_state.ang_vel_deg.y(), cf_state.ang_vel_deg.z());
#endif
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

void CF_model::publishPose(){
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

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), topic + "/base_link", "/base_link"));
}

void CF_model::runAttPids() {
	pidHandler.runAttPID(cf_state.attitude_deg.x(), cf_state.attitude_deg.y(),  cf_state.attitude_deg.z());
	pidHandler.runAngVelPID(cf_state.ang_vel_deg.x(), cf_state.ang_vel_deg.y(), cf_state.ang_vel_deg.z(), cf_state.motor_pwm);
}

void CF_model::runPosPids() {
	pidHandler.runPosPID(cf_state.position.x(), cf_state.position.y(), cf_state.position.z());
	pidHandler.runLinVelPID(cf_state.lin_vel.x(), cf_state.lin_vel.y(), cf_state.lin_vel.z(), cf_state.attitude.z());
}

void CF_model::runDynamics(){
    boost::thread t1(&CF_model::run, this);
}

void CF_model::run() {
	while (ros::ok()) {
		if (isInit) {
			applySimulationStep();
			if (attPidCounter == pidHandler.getPidAttCounterMax()) {
				runAttPids(); 
				attPidCounter = 0;
			}
			else {
				attPidCounter++;
			}
			if (posPidCounter == pidHandler.getPidPosCounterMax()) {
				if(mode == "POS"){
					runPosPids();
				}
				publishPose();
				posPidCounter = 0; 
			}
			else{
				posPidCounter++;			
			}
		}
		simulation_freq.sleep();
	}
}
