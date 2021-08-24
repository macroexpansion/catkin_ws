#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String, Float32
import numpy as np
from scipy.ndimage import median_filter as mf
import numpy as np


def callback(data):
    global last_time, x, y, theta
    linear_velocity_x_ = data.linear_x
    angular_velocity_z_ = data.angular_z
    cr_time = rospy.get_time()
    dt = cr_time - last_time
    last_time = cr_time
    theta_k = theta + angular_velocity_z_ * dt
    S = np.array([[np.cos(theta_k), 0], [np.sin(theta_k), 0], [0, 1]])
    p_c_dot = np.dot(S, np.array([linear_velocity_x_, angular_velocity_z_]))
    x, y, theta = np.array([x, y, theta]) + p_c_dot * dt
    distance = np.sqrt(x ** 2 + y ** 2)
    setpoint_theta.publish(np.degrees(theta))
    setpoint_distance.publish(distance)


if __name__ == '__main__':
    x = -0.562-0.65
    y = -0
    theta = np.radians(0)
    rospy.init_node('static', anonymous=True)
    setpoint_theta = rospy.Publisher("/setpoint_theta", Float32, queue_size=1)
    setpoint_distance = rospy.Publisher("/setpoint_distance", Float32, queue_size=1)
    last_time = rospy.get_time()
    rospy.Subscriber("/raw_vel", vel, callback)
    # pub = rospy.Publisher('/median_aruco', Float32, queue_size=10)
    rospy.spin()
