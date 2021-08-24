#!/usr/bin/env python

from __future__ import print_function
import threading
import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String
import sys, select, termios, tty
import numpy as np
import parameter_control as params


class Tracking:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber("/distance", String, self.callback1)
        rospy.Subscriber("/angle", Angle, self.callback2)
        # rospy.Subscriber("/raw_vel", vel, self.callback)
        self.last_vel_time_ = rospy.get_time()
        self.max_vel_z = params.max_vel_z
        self.k_distan_phi = params.k_distan_phi
        self.setpoint_x = -1
        self.setpoint_phi = -1
        self.thresh_angle = params.thresh_angle
        self.distan_phi = 0
        self.error_phi = 0
        self.prevError = 0
        self.vel_dt_ = 0

    def callback(self, data):

        vel_z = self.rotate_robot()
        vel_x = 0
        # print('setpoint-rotated: ', self.setpoint_phi)
        # try:
        #     self.controller(vel_x, vel_z)
        # except rospy.ROSInterruptException:
        #     pass

    def callback1(self, data):
        if self.setpoint_x == -1 and float(data.data) != -1:
            self.setpoint_x = float(data.data)

    def callback2(self, data):
        if self.setpoint_phi == -1 and data.angle_base != -1:
            self.setpoint_phi = data.angle_base
        if data.angle_base != -1:
            self.error_phi = data.angle_base
            vel_z = self.rotate_robot()
        try:
            self.controller(0, vel_z)
        except rospy.ROSInterruptException:
            pass

    def rotate_robot(self):
        current_time = rospy.get_time()
        cP = self.error_phi
        vel_dt_ = current_time - self.last_vel_time_
        delta_error = cP - self.prevError
        self.last_vel_time_ = current_time
        self.prevError = cP
        cD = delta_error / vel_dt_
        kp = 0.06
        kd = 0.01
        vel_z = kp * cP + kd * cD
        print(cP, cD, vel_z)
        if vel_z > self.max_vel_z:
            vel_z = self.max_vel_z
        if vel_z < -self.max_vel_z:
            vel_z = -self.max_vel_z
        if np.abs(self.error_phi) <= self.thresh_angle:
            vel_z = 0
            self.setpoint_phi = -1
            self.distan_phi = 0
        return vel_z

    def controller(self, vel_x, vel_z):
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vel_z

        self.pub.publish(twist)


def main():
    rospy.init_node('aruco_tracking', anonymous=True)
    Tracking()

    rospy.spin()


if __name__ == '__main__':
    main()
