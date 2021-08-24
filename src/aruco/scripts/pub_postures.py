#!/usr/bin/env python


from __future__ import print_function
import threading
import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String, Float32
import sys, select, termios, tty


import numpy as np
import parameter_control as params


def controler(q_r, p_e):
    print('pe : ', p_e)
    K_x, K_y, K_theta = 0.1, 15, 5
    x_e, y_e, theta_e = p_e
    v_r, w_r = q_r
    v = v_r * np.cos(theta_e) + K_x * x_e
    w = w_r + v_r * (K_y * y_e + K_theta * np.sin(theta_e))
    return [v, w]


def callback(data):
    global last_vel_time_, p_c, x, y
    r = params.r
    b = params.b
    p_r = [0.98, 0.33, -0.2]
    q_r = [0.1, 0]
    current_time = rospy.get_time()
    linear_velocity_x_ = data.linear_x
    linear_velocity_y_ = data.linear_y
    angular_velocity_z_ = data.angular_z
    vel_dt_ = current_time - last_vel_time_
    last_vel_time_ = current_time
    theta = p_c[-1]
    # J = r / 2 * np.array([[np.cos(theta), np.cos(theta)], [np.sin(theta), np.cos(theta)], [1 / b, -1 / b]])
    # B = 1 / r * np.array(([1, b], [1, -b]))
    # J = np.dot(J, B)
    S = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
    p_c_dot = np.dot(S, np.array([linear_velocity_x_, angular_velocity_z_]))
    p_c += p_c_dot * vel_dt_
    theta = p_c[-1]
    p_e = p_r - p_c
    T = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    p_e = np.dot(T, p_e)
    v, w = controler(q_r, p_e)
    print(v, w)
    # x.append(p_c[0])
    # y.append(p_c[1])
    # if len(x) == 200:
    #     print('ok')
    #     x = np.asarray(x)
    #     y = np.asarray(y)
    #     with open('/home/cros/x.npy', 'wb') as f:
    #         np.save(f, x)
    #     with open('/home/cros/y.npy', 'wb') as f:
    #         np.save(f, y)

    try:
        pI(v, w)
    except rospy.ROSInterruptException:
        pass


def pI(vel_x, vel_z):
    twist.linear.x = vel_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = vel_z
    pub.publish(twist)


if __name__ == '__main__':
    global x, y
    x = []
    y = []
    p_c = [0.65, 0.445, 0]
    rospy.init_node('postures', anonymous=True)
    last_vel_time_ = rospy.get_time()
    twist = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/raw_vel", vel, callback)
    rospy.spin()
