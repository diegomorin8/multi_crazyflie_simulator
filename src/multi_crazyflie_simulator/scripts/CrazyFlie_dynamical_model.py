#!/usr/bin/env python

import rospy
import numpy as np
import tf
from math import cos, sin, tan

from cf_physical_parameters import CF_parameters
from cf_pid_params import CF_pid_params
from pid import PID
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import String

class CF_state():

    def __init__(self):
        self.position = np.zeros(3)
        self.lin_vel = np.zeros(3)
        self.attitude = np.zeros(3)
        self.attitude_deg = np.zeros(3)
        self.ang_vel_deg = np.zeros(3)
        self.ang_vel = np.zeros(3)   
        self.motor_pwm = np.zeros(CF_parameters().NUM_MOTORS)
        self.motor_rotation_speed = np.zeros(CF_parameters().NUM_MOTORS)
        self.sum_motor_rotations = 0.0
        self.forces = np.zeros(3)
        self.momentums = np.zeros(3)

    def getMotorRotationSpeed(self):
        for i in range(0,4):
            self.motor_rotation_speed[i] = 0.2685*self.motor_pwm[i] + 4070.3

    def addMotorsRotationsSpeed(self):
        self.sum_motor_rotations = sum(self.motor_rotation_speed**2)

    def getForces(self):
        self.forces = np.array([0, 0, CF_parameters().CT*self.sum_motor_rotations])

    def getMomentums(self):
        self.momentums[0] = (CF_parameters().L * CF_parameters().CT/np.sqrt(2))*(-self.motor_rotation_speed[0]**2 - self.motor_rotation_speed[1]**2 + self.motor_rotation_speed[2]**2 + self.motor_rotation_speed[3]**2)
        self.momentums[1] = (CF_parameters().L * CF_parameters().CT/np.sqrt(2))*(-self.motor_rotation_speed[0]**2 + self.motor_rotation_speed[1]**2 + self.motor_rotation_speed[2]**2 - self.motor_rotation_speed[3]**2)
        self.momentums[2] = CF_parameters().CD * (-self.motor_rotation_speed[0]**2 + self.motor_rotation_speed[1]**2 - self.motor_rotation_speed[2]**2 + self.motor_rotation_speed[3]**2)


class CF_model():

    def __init__(self):

        rospy.init_node("dynamic_model", anonymous = True)
        self.topic = rospy.get_param("~topic")

        rospy.Subscriber(self.topic + "/cmd_vel", Twist, self.new_attitude_setpoint)
        rospy.Subscriber(self.topic + "/cmd_pos", Position, self.new_position_setpoint)
        rospy.Subscriber("/init_pose", PointStamped, self.new_init_position)
        self.pub_pos = rospy.Publisher(self.topic +"/out_pos", PoseStamped, queue_size = 1000)
        self.pub_ack = rospy.Publisher("/init_pose_ack", String, queue_size = 1000)
        self.msg = PoseStamped()

        self.isInit = False

        # System state: position, linear velocities,
        # attitude and angular velocities
        self.cf_state = CF_state()

        # Import the crazyflie physical paramters
        #     - These parameters are obtained from different sources.
        #     - For these parameters and the dynamical equations refer
        #       to : DESIGN OF A TRAJECTORY TRACKING CONTROLLER FOR A
        #            NANOQUADCOPTER
        #            Luis, C., & Le Ny, J. (August, 2016)
        self.cf_physical_params = CF_parameters()

        # Import the PID gains (from the firmware)
        self.cf_pid_gains = CF_pid_params()

        # Main CF variables initialization (if needed)
        self.simulation_freq = rospy.Rate(int(1/self.cf_physical_params.DT_CF))

        ######################
        # Initialize PID
        ######################

        # Out from the PIDs, values of
        # r, p, y, thrust
        self.desired_rpy = np.zeros(3)

        # Comes from the external position controller
        self.desired_thrust = 0.0


        ######################
        # Angular velocities
        ######################

        self.wx_pid = PID(self.cf_pid_gains.KP_WX,
                          self.cf_pid_gains.KI_WX,
                          self.cf_pid_gains.KD_WX,
                          self.cf_pid_gains.INT_MAX_WX,
                          self.cf_pid_gains.WX_DT)

        self.wy_pid = PID(self.cf_pid_gains.KP_WY,
                          self.cf_pid_gains.KI_WY,
                          self.cf_pid_gains.KD_WY,
                          self.cf_pid_gains.INT_MAX_WY,
                          self.cf_pid_gains.WY_DT)

        self.wz_pid = PID(self.cf_pid_gains.KP_WZ,
                          self.cf_pid_gains.KI_WZ,
                          self.cf_pid_gains.KD_WZ,
                          self.cf_pid_gains.INT_MAX_WZ,
                          self.cf_pid_gains.WZ_DT)

        self.desired_ang_vel = np.zeros(3)

        ######################                                                  
        # Attitudes
        ######################

        self.roll_pid = PID(self.cf_pid_gains.KP_ROLL,
                          self.cf_pid_gains.KI_ROLL,
                          self.cf_pid_gains.KD_ROLL,
                          self.cf_pid_gains.INT_MAX_ROLL,
                          self.cf_pid_gains.ROLL_DT)

        self.pitch_pid = PID(self.cf_pid_gains.KP_PITCH,
                             self.cf_pid_gains.KI_PITCH,
                             self.cf_pid_gains.KD_PITCH,
                             self.cf_pid_gains.INT_MAX_PITCH,
                             self.cf_pid_gains.PITCH_DT)

        self.yaw_pid = PID(self.cf_pid_gains.KP_YAW,
                             self.cf_pid_gains.KI_YAW,
                             self.cf_pid_gains.KD_YAW,
                             self.cf_pid_gains.INT_MAX_YAW,
                             self.cf_pid_gains.YAW_DT)

        self.att_pid_counter = 0
        self.att_pid_counter_max = int( self.cf_physical_params.DT_ATT_PIDS / self.cf_physical_params.DT_CF) - 1

        self.desired_att = np.zeros(3)

        ######################
        # Angular velocities
        ######################

        self.vx_pid = PID(self.cf_pid_gains.KP_VX,
                          self.cf_pid_gains.KI_VX,
                          self.cf_pid_gains.KD_VX,
                          self.cf_pid_gains.INT_MAX_VX,
                          self.cf_pid_gains.VX_DT)

        self.vy_pid = PID(self.cf_pid_gains.KP_VY,
                          self.cf_pid_gains.KI_VY,
                          self.cf_pid_gains.KD_VY,
                          self.cf_pid_gains.INT_MAX_VY,
                          self.cf_pid_gains.VY_DT)

        self.vz_pid = PID(self.cf_pid_gains.KP_VZ,
                          self.cf_pid_gains.KI_VZ,
                          self.cf_pid_gains.KD_VZ,
                          self.cf_pid_gains.INT_MAX_VZ,
                          self.cf_pid_gains.VZ_DT)

        self.desired_lin_vel = np.zeros(3)

        ######################
        # Angular velocities
        ######################

        self.x_pid = PID(self.cf_pid_gains.KP_X,
                          self.cf_pid_gains.KI_X,
                          self.cf_pid_gains.KD_X,
                          self.cf_pid_gains.INT_MAX_X,
                          self.cf_pid_gains.X_DT)

        self.y_pid = PID(self.cf_pid_gains.KP_Y,
                          self.cf_pid_gains.KI_Y,
                          self.cf_pid_gains.KD_Y,
                          self.cf_pid_gains.INT_MAX_Y,
                          self.cf_pid_gains.Y_DT)

        self.z_pid = PID(self.cf_pid_gains.KP_Z,
                          self.cf_pid_gains.KI_Z,
                          self.cf_pid_gains.KD_Z,
                          self.cf_pid_gains.INT_MAX_Z,
                          self.cf_pid_gains.Z_DT)

        self.desired_pos = np.zeros(3)

        ############################
        # Communication control
        ############################
        self.out_pos_counter = 0
        self.out_pos_counter_max = int(self.cf_physical_params.DT_POS_PIDS/ self.cf_physical_params.DT_CF) - 1

        self.mode = "POS"

        self.time_processing = []
        self.delta_time = []
     
    ###########################
    # Math operations functions
    ###########################

    # Rotation matrix around X
    def rot_x(self, alpha):
        cos_a = cos(alpha)
        sin_a = sin(alpha)
        M = np.array([[1, 0, 0], [0, cos_a, sin_a], [0, -sin_a, cos_a]])

        return M

    # Rotation matrix around Y
    def rot_y(self, beta):
        cos_b = cos(beta)
        sin_b = sin(beta)
        M = np.array([[cos_b, 0, -sin_b], [0, 1, 0], [sin_b, 0, cos_b]])

        return M

    # Rotation matrix around Z
    def rot_z(self, gamma):
        cos_g = cos(gamma)
        sin_g = sin(gamma)
        M = np.array([[cos_g, sin_g, 0], [-sin_g, cos_g, 0], [0, 0, 1]])

        return M

    # Rotation matrix - from the body frame to the inertial frame
    def rot_m(self, roll, pitch, yaw):
        rotx = self.rot_x(roll)
        roty = self.rot_y(pitch)
        rotz = self.rot_z(yaw)
        rot = np.dot(np.dot(rotx, roty), rotz)

        return rot

    def rotation_matrix(self, roll, pitch, yaw):
        return np.array([[cos(pitch)*cos(yaw),
                             cos(pitch)*sin(yaw),
                             -sin(pitch)],
                            [sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
                             sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
                             sin(roll)*cos(pitch)],
                            [cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
                             cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
                             cos(roll)*cos(pitch)]])

    def euler_matrix(self, roll, pitch, yaw):
        cos_roll = cos(roll)
        sin_roll = sin(roll)

        cos_pitch = cos(pitch) + 1e-12
        tan_pitch = tan(pitch)
        return np.array([[1, sin_roll * tan_pitch, cos_roll * tan_pitch], [0, cos_roll, -sin_roll],
                      [0, sin_roll / cos_pitch, cos_roll / cos_pitch]])

    def limit(self, value):
        return max(min(self.cf_physical_params.PWM_MAX, value), self.cf_physical_params.PWM_MIN)

    ###########################
    # Callback function
    ###########################
    def new_attitude_setpoint(self, twist_msg):

        ##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ### DUDA EN EL CODIGO DEL SERVER DE LO QUE ES m_X_trim
        ######################################################
        self.mode = "ATT"
        self.desired_att[0] = twist_msg.linear.y
        self.desired_att[1] = -twist_msg.linear.x
        self.desired_ang_vel[2] = twist_msg.angular.z
        self.desired_thrust = min(twist_msg.linear.z, 60000)

    ###########################
    # Callback function
    ###########################
    def new_position_setpoint(self, position_msg):
        ##############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ### DUDA EN EL CODIGO DEL SERVER DE LO QUE ES m_X_trim
        ######################################################
        self.mode="POS"
        self.desired_pos[0] = position_msg.x
        self.desired_pos[1] = position_msg.y
        self.desired_pos[2] = position_msg.z
        self.desired_att[2] = position_msg.yaw

    def new_init_position(self, position_msg):
        if(self.topic == position_msg.header.frame_id):
            self.cf_state.position = np.array([position_msg.point.x, position_msg.point.y, position_msg.point.z])
            self.desired_pos = np.array([position_msg.point.x, position_msg.point.y, position_msg.point.z])
            self.isInit = True
            msgID = String()
            msgID = position_msg.header.frame_id
            self.pub_ack.publish(msgID)


    ###########################
    # Single step simulation
    ###########################
    def apply_simulation_step(self):

        ###########################
        # Main simulation loop
        #   - The CF works at a rate of 1000Hz,
        #     in the same way, we are simulating
        #     at the same frequency
        ###########################

        # New simulated state
        new_state = CF_state()

        rotation_matrix = self.rot_m(self.cf_state.attitude[0],self.cf_state.attitude[1], self.cf_state.attitude[2])
        euler_matrix = self.euler_matrix(self.cf_state.attitude[0],self.cf_state.attitude[1], self.cf_state.attitude[2])

        self.cf_state.getMotorRotationSpeed()
        self.cf_state.addMotorsRotationsSpeed()
        self.cf_state.getForces()
        self.cf_state.getMomentums()

        new_state.lin_vel = np.dot(np.linalg.inv(rotation_matrix),self.cf_state.forces) / self.cf_physical_params.DRONE_MASS - np.array([0, 0, self.cf_physical_params.G])# - np.cross(self.cf_state.ang_vel, self.cf_state.lin_vel)

        new_state.position = self.cf_state.lin_vel

        preoperation = self.cf_state.momentums - np.cross(self.cf_state.ang_vel,
                                            np.dot(self.cf_physical_params.INERTIA_MATRIX,
                                            self.cf_state.ang_vel))
        new_state.ang_vel = np.dot(self.cf_physical_params.INV_INERTIA_MATRIX, preoperation)

        new_state.attitude = np.dot(euler_matrix, self.cf_state.ang_vel)

        for i in range(0, 3):
            self.cf_state.position[i] = self.cf_state.position[i] + (new_state.position[i] * self.cf_physical_params.DT_CF)
            self.cf_state.attitude[i] = self.cf_state.attitude[i] + (new_state.attitude[i] * self.cf_physical_params.DT_CF)
            self.cf_state.lin_vel[i] = self.cf_state.lin_vel[i] + (new_state.lin_vel[i] * self.cf_physical_params.DT_CF)
            self.cf_state.ang_vel[i] = self.cf_state.ang_vel[i] + (new_state.ang_vel[i] * self.cf_physical_params.DT_CF)

        self.cf_state.ang_vel_deg = self.cf_state.ang_vel*180.0/np.pi
        self.cf_state.attitude_deg = self.cf_state.attitude*180.0/np.pi

        # Ground constraints
        if self.cf_state.position[2] <= 0:
            self.cf_state.position[2] = 0
            if self.cf_state.lin_vel[2] <= 0:
                self.cf_state.lin_vel[:] = 0.
                self.cf_state.attitude[:-1] = 0.
                self.cf_state.ang_vel[:] = 0.

    def run_pos_pid(self):
        self.desired_lin_vel = np.array([self.x_pid.update(self.desired_pos[0], self.cf_state.position[0]),
                                         self.y_pid.update(self.desired_pos[1], self.cf_state.position[1]),
                                         self.z_pid.update(self.desired_pos[2], self.cf_state.position[2])])

    def run_lin_vel_pid(self):
        raw_pitch = self.vy_pid.update(self.desired_lin_vel[1], self.cf_state.lin_vel[1])
        raw_roll = self.vx_pid.update(self.desired_lin_vel[0], self.cf_state.lin_vel[0])

        # Current YAW
        raw_yaw = -self.cf_state.attitude[2]

        # Transformation to the drone body frame
        self.desired_att[1]  = -(raw_roll * cos(raw_yaw)) - (raw_pitch * sin(raw_yaw))
        self.desired_att[0]  = -(raw_pitch * cos(raw_yaw)) + (raw_roll * sin(raw_yaw))

        self.desired_att[0] = max(min(self.cf_pid_gains.MAX_ATT, self.desired_att[0]), -self.cf_pid_gains.MAX_ATT)
        self.desired_att[1] = max(min(self.cf_pid_gains.MAX_ATT, self.desired_att[1]), -self.cf_pid_gains.MAX_ATT)

        raw_thrust = self.vz_pid.update(self.desired_lin_vel[2], self.cf_state.lin_vel[2])

        self.desired_thrust = max((raw_thrust * 1000 + self.cf_physical_params.BASE_THRUST), self.cf_physical_params.PWM_MIN)

    def run_att_pid(self):
        self.desired_ang_vel = np.array([self.roll_pid.update(self.desired_att[0], self.cf_state.attitude_deg[0]),
                                         self.pitch_pid.update(-self.desired_att[1], self.cf_state.attitude_deg[1]),
                                         self.yaw_pid.update(self.desired_att[2], self.cf_state.attitude_deg[2])])

    def run_ang_vel_pid(self):
        self.desired_rpy = np.array([self.wx_pid.update(self.desired_ang_vel[0], self.cf_state.ang_vel_deg[0]),
                                     -self.wy_pid.update(self.desired_ang_vel[1], self.cf_state.ang_vel_deg[1]),
                                     -self.wz_pid.update(self.desired_ang_vel[2], self.cf_state.ang_vel_deg[2])])
        self.rpyt_2_motor_pwm()

    def log_state(self):

        rospy.loginfo("\n Nuevo estado: \n Position: " + str(self.cf_state.position)
                      + "\n Lin. Velocity: " + str(self.cf_state.lin_vel)
                      + "\n Attitude: " + str(self.cf_state.attitude*180/np.pi)
                      + "\n Ang. velocities: " + str(self.cf_state.ang_vel*180/np.pi)
                      + "\n Delta time: " + str(sum(self.delta_time)/len(self.delta_time))
                      + "\n Processing time " + str(sum(self.time_processing)/len(self.time_processing)))
        self.time_processing = []
        self.delta_time = []

    def rpyt_2_motor_pwm(self):

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

    def publishPose(self):
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

    def run(self):
        tic_init = time.time()
        while(not rospy.is_shutdown()):
            if(self.isInit):
                self.delta_time.append(time.time() - tic_init)
                tic_init = time.time()
                self.apply_simulation_step()

                if(self.att_pid_counter == self.att_pid_counter_max):
                    self.att_pid_counter = 0
                    self.run_att_pid()
                    self.run_ang_vel_pid()

                else:
                    self.att_pid_counter = self.att_pid_counter + 1

                if(self.out_pos_counter == self.out_pos_counter_max):
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
