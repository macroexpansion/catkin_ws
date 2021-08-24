#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String, Float32
import numpy as np
from scipy.ndimage import median_filter as mf
import json


class Collect:
    def __init__(self):
        rospy.Subscriber("/setpoint_theta", Float32, self.callback_sptheta)
        rospy.Subscriber("/setpoint_distance", Float32, self.callback_spdistance)
        rospy.Subscriber("/distance", Float32, self.callback_distance)
        rospy.Subscriber("/angleFiltered", Float32, self.callback_kalman)
        rospy.Subscriber("/angle", Float32, self.callback_angle)

        self.sptheta = 0
        self.spdistance = 0
        self.distance = 0
        self.kalman = 0
        self.angle = 0

        self.save = []

    def callback_sptheta(self, data):
        self.sptheta = data.data

    def callback_spdistance(self, data):
        self.spdistance = data.data
        self.save.append(
            {"angleFiltered": self.kalman, "distance": self.distance, "angle_base": self.angle, "sptheta": self.sptheta,
             'spdis': self.spdistance})
        print(len(self.save))
        if len(self.save) == 5000:
            print('------xxx------------------saved---------------------xxxx------')
            with open('/home/cros/catkin_ws/src/aruco/data_fiting/input.json', 'w') as f:
                json.dump(self.save, f, indent=1)

    def callback_distance(self, data):
        self.distance = data.data

    def callback_kalman(self, data):
        self.kalman = data.data

    def callback_angle(self, data):
        self.angle = data.data


if __name__ == '__main__':
    rospy.init_node('static', anonymous=True)


    Collect()
    # pub = rospy.Publisher('/median_aruco', Float32, queue_size=10)
    rospy.spin()
