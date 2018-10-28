#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from math import cos, sin
import random

class initPose():

    def __init__(self):

        # Init ros node
        rospy.init_node("pose_init", anonymous=True)

        ########################
        ### ROSLAUNCH PARAMS ###
        ########################

        self.num_quadcopters = rospy.get_param("~num_quads")
        self.mode = rospy.get_param("~mode")
        self.spread_radius = rospy.get_param("~radius")

        # Position publisher
        self.pub_pos = rospy.Publisher("/init_pose", PointStamped, queue_size = 1000)
        rospy.Subscriber("/init_pose_ack", String, self.ack_received)
        self.msg = PointStamped()

        self.ack = {}
        self.poses = {}

    def ack_received(self, id):
        self.ack[id.data] = True

    def allAckReceived(self):
        return (len(self.ack) == self.num_quadcopters)

    def getXYinCircle(self, r, theta):
        x = r*cos(theta)
        y = r*sin(theta)
        return [x,y]

    def send(self):
        for key in self.poses.iterkeys():
            self.msg.header.frame_id = key
            self.msg.point.x = self.poses[key][0]
            self.msg.point.y = self.poses[key][1]
            self.msg.point.z = 0.0
            self.pub_pos.publish(self.msg)

    def getPoses(self):
        if self.num_quadcopters > 1:
            if self.mode == "CIRCLE":
                deltaTheta = np.pi*2/self.num_quadcopters
                for i in range(self.num_quadcopters):
                    self.poses["/crazyflie_" + str(i)] = self.getXYinCircle(self.spread_radius, i*deltaTheta)

            if self.mode == "RANDOM":
                for i in range(self.num_quadcopters):
                    deltaTheta = random.random()*np.pi*2
                    radius = random.random()*self.spread_radius
                    self.poses["/crazyflie_" + str(i)] = self.getXYinCircle(radius, deltaTheta)
        else:
            self.poses["/crazyflie_0"] = [0.0, 0.0]
if __name__ == '__main__':
    initPose_ = initPose()
    initPose_.getPoses()

    while(not initPose_.allAckReceived()):
        initPose_.send()
        rospy.sleep(0.1)
