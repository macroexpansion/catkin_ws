#!/usr/bin/env python

from __future__ import print_function
import threading
import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String, Float32
import numpy as np
import parameter_control as params
from scipy.ndimage import median_filter as mf


def convert_to_parallel(distance_base, phi_data):
    pi = params.pi
    angle_aruco = phi_data.angle_aruco
    angle_base = phi_data.angle_base
    fix_rot = params.fix_rot
    if angle_aruco <= 180:
        angle_rot_to_prll = 90 - angle_aruco
        angle_to_paralle = -angle_base + angle_rot_to_prll
        angle_to_paralle = angle_to_paralle * pi / 180
        distance_to_prll = distance_base * np.cos(angle_to_paralle)
        fix_phi = -fix_rot
    if angle_aruco >= 180:
        angle_aruco = 360 - angle_aruco
        angle_rot_to_prll = -(90 - angle_aruco)
        angle_to_paralle = angle_base - angle_rot_to_prll
        angle_to_paralle = angle_to_paralle * pi / 180
        distance_to_prll = distance_base * np.cos(angle_to_paralle)
        fix_phi = fix_rot
    print('aruco {:.3f}\nangle to pll {:.3f}\nangle rot: {:.3f}\ndistance to pll: {:.3f}\ndistance base: {:.3f}'.format(
        angle_aruco,
        angle_to_paralle * 180 / pi,
        angle_rot_to_prll,
        distance_to_prll,
        distance_base))
    return distance_to_prll, angle_rot_to_prll, fix_phi


class controller:
    def __init__(self):
        self.last_vel_time_ = rospy.get_time()
        self.distan_x = 0
        self.distan_phi = 0
        self.in_run_x = 0
        self.in_run_z = 1
        self.setpoint_x = 0
        self.setpoint_phi = 0
        self.error_angle_cam = 0
        self.state = 0
        self.dentaphiquay = 0
        self.list_aruco = []
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/distance", Float32, self.callback1)
        rospy.Subscriber("/angle", Angle, self.callback2)
        rospy.Subscriber("/raw_vel", vel, self.callback)

    def reset(self):
        self.setpoint_x = 0
        self.setpoint_phi = 0
        self.distan_phi = 0
        self.distan_x = 0

    def callback1(self, data):
        if self.setpoint_x == 0 and data.data != -1:
            self.setpoint_x = data.data

    def callback2(self, data):
        if 0 < data.angle_aruco < 360:
            self.list_aruco.append(data.angle_aruco)
        if len(self.list_aruco) >= 10 and self.setpoint_phi == 0:
            aruco_array = self.list_aruco[-10:]
            aruco_array = np.asarray(aruco_array)
            self.setpoint_phi = data
            self.setpoint_phi.angle_aruco = np.max(mf(aruco_array, 40))
        self.error_angle_cam = data.angle_base

    def callback(self, data):
        # self.pI(0.5, 0)
        if (self.setpoint_x != 0 and self.setpoint_phi != 0) or self.state == 1:
            if self.state == 0 and isinstance(self.setpoint_phi, Angle):
                self.setpoint_x, self.setpoint_phi, self.dentaphiquay = convert_to_parallel(self.setpoint_x,
                                                                                            self.setpoint_phi)
            if self.state == 1:
                self.setpoint_x = 0.005
                self.setpoint_phi = self.dentaphiquay

            if self.state >= 2 and isinstance(self.setpoint_phi, Angle):
                self.setpoint_phi = self.setpoint_phi.angle_base

            vel_z = 0
            vel_x = 0
            current_time = rospy.get_time()
            linear_velocity_x_ = data.linear_x
            angular_velocity_z_ = data.angular_z
            vel_dt_ = current_time - self.last_vel_time_
            self.last_vel_time_ = current_time
            delta_heading = angular_velocity_z_ * vel_dt_ * 180 / 3.14
            delta_x = linear_velocity_x_ * vel_dt_
            self.distan_phi += delta_heading
            self.distan_x += delta_x
            if self.in_run_z:
                vel_z = self.rotatate_robot()
            if self.in_run_x:
                vel_x = self.linear_move()

            try:
                self.pI(vel_x, vel_z)
            except rospy.ROSInterruptException:
                pass

    def rotatate_robot(self):
        thresh_angle = params.thresh_angle
        error_ang = self.setpoint_phi - self.distan_phi * params.k_distan_phi
        if error_ang > thresh_angle:
            vel_z = params.max_vel_z
        elif error_ang < -thresh_angle:
            vel_z = -params.max_vel_z
        if self.state >= 2:
            error_ang = self.error_angle_cam
            vel_z = error_ang * params.kp
        if np.abs(error_ang) <= thresh_angle:
            print("state: {:.3f} \nerror_angle_cam: {:.3f}".format(self.state, self.error_angle_cam))
            vel_z = 0
            rospy.sleep(1)
            self.in_run_z = 0
            self.in_run_x = 1
            self.last_vel_time_ = rospy.get_time()
        return vel_z

    def linear_move(self):
        stop = 0
        error = self.setpoint_x - self.distan_x + params.offset_x
        if error > 0.01:
            vel_x = params.max_vel_x

        if error <= 0.01 or stop:
            print('reset')
            vel_x = 0
            self.reset()
            self.in_run_x = 0
            self.in_run_z = 1
            self.state += 1
            rospy.sleep(1)
            self.last_vel_time_ = rospy.get_time()
        return vel_x

    def pI(self, vel_x, vel_z):
        # while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vel_z
        self.pub.publish(twist)


def main():
    rospy.init_node('controller', anonymous=True)
    controller()
    rospy.spin()


if __name__ == '__main__':
    main()
