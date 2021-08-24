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


def callback1(data):
    global setpoint_x
    if setpoint_x == 0 and float(data.data) != -1:
        setpoint_x = float(data.data)


def callback2(data):
    global setpoint_phi
    if setpoint_phi == 0 and data.angle_base != -1:
        setpoint_phi = data
    global error_angle_cam
    error_angle_cam = data.angle_base


def rotatate_robot(design_phi, delta_heading, k_distan_phi, max_vel_z):
    global distan_phi, in_run_z, in_run_x, state_trans
    state_trans = 0
    thresh_angle = params.thresh_angle
    distan_phi += delta_heading
    error_ang = design_phi - distan_phi * k_distan_phi
    if error_ang > thresh_angle:
        vel_z = max_vel_z
    elif error_ang < -thresh_angle:
        vel_z = -max_vel_z

    if np.abs(error_ang) < thresh_angle:
        state_trans = 1
        rospy.sleep(1)
        vel_z = 0
        in_run_z = 0
        in_run_x = 1
    return vel_z


def linear_move(design_distance, delta_x, offset_x, max_vel_x):
    global distan_x, in_run_x, setpoint_x, setpoint_phi, state, distan_phi, in_run_z, state_trans
    state_trans = 0
    distan_x += delta_x
    error = design_distance - distan_x + offset_x
    if error > 0.01:
        vel_x = max_vel_x
    if error <= 0.01:
        print('reset')
        state_trans = 1
        setpoint_x = 0
        setpoint_phi = 0
        distan_phi = 0
        distan_x = 0
        vel_x = 0
        in_run_x = 0
        in_run_z = 1
        state += 1
        rospy.sleep(1)
    return vel_x


def convert_to_parallel(distance_base, phi_data):
    pi = params.pi
    angle_aruco = phi_data.angle_aruco
    angle_base = phi_data.angle_base
    angle_to_paralle = -angle_base + 90 - angle_aruco
    angle_to_paralle = angle_to_paralle * pi / 180
    distance_to_prll = distance_base * np.cos(angle_to_paralle)
    angle_rot_to_prll = 90 - angle_aruco
    return distance_to_prll, angle_rot_to_prll


def callback(data):
    offset_x = params.offset_x
    max_vel_x = params.max_vel_x
    max_vel_z = params.max_vel_z
    k_distan_phi = params.k_distan_phi
    global last_vel_time_
    global in_run_x, setpoint_x, setpoint_phi, in_run_z
    global distan_y, distan_x, distan_phi, state

    if (setpoint_x != 0 and setpoint_phi != 0) or state == 1:
        if state == 0 and isinstance(setpoint_phi, Angle):
            setpoint_x, setpoint_phi = convert_to_parallel(setpoint_x, setpoint_phi)
        if state == 1:
            setpoint_x = 0.01
            setpoint_phi = -120
        if state >= 2 and isinstance(setpoint_phi, Angle):
            distan_phi = 0
            setpoint_phi = setpoint_phi.angle_base * params.k_angle_base
        # print("setpoint", state, setpoint_x, setpoint_phi)
        vel_z = 0
        vel_x = 0
        current_time = rospy.get_time()
        linear_velocity_x_ = data.linear_x
        linear_velocity_y_ = data.linear_y
        angular_velocity_z_ = data.angular_z
        vel_dt_ = current_time - last_vel_time_
        vel_dt_ = vel_dt_ - int(vel_dt_)
        last_vel_time_ = current_time
        delta_heading = angular_velocity_z_ * vel_dt_ * 180 / 3.14
        delta_x = linear_velocity_x_ * vel_dt_
        delta_y = linear_velocity_y_ * vel_dt_
        if in_run_z:
            vel_z = rotatate_robot(setpoint_phi, delta_heading, k_distan_phi, max_vel_z)
        if in_run_x:
            vel_x = linear_move(setpoint_x, delta_x, offset_x, max_vel_x)
        if state == 2:
            # print(delta_heading, angular_velocity_z_, vel_dt_)
            print("setpoint", state, setpoint_x, setpoint_phi)
            print('vel_z: ', distan_phi, vel_z)
            print('vel_x: ', distan_x, vel_x)

        try:
            pI(vel_x, vel_z)
        except rospy.ROSInterruptException:
            pass


def pI(vel_x, vel_z):
    rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    twist.linear.x = vel_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = vel_z
    pub.publish(twist)
    # rate.sleep()


def main():
    global last_vel_time_, setpoint_x, setpoint_phi

    last_vel_time_ = rospy.get_time()

    # if setpoint_x == 0 and setpoint_phi == 0:
    rospy.Subscriber("/distance", String, callback1)
    rospy.Subscriber("/angle", Angle, callback2)
    # else:
    rospy.Subscriber("/raw_vel", vel, callback)
    rospy.spin()


if __name__ == '__main__':
    global last_vel_time_
    global distan_x, distan_phi
    global in_run_x, in_run_z
    global setpoint_x, setpoint_phi
    global error_angle_cam, state, state_trans
    state_trans = 0
    state = 0
    error_angle_cam = 0
    aruco_detected = 0
    setpoint_x = 0
    setpoint_phi = 0
    in_run_x = 0
    in_run_z = 1
    distan_x = 0
    distan_y = 0
    distan_phi = 0
    twist = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    print('init node')
    main()
