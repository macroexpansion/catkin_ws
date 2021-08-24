#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from aruco.msg import vel, Angle
from std_msgs.msg import String, Float32
import numpy as np
from scipy.ndimage import median_filter as mf


def callback(data):
    global list_aruco
    if 0 < data.angle_aruco < 360:
        list_aruco.append(data.angle_aruco)
    if len(list_aruco) >= 20:
        aruco_array = list_aruco[-10:]
        aruco_array = np.asarray(aruco_array)
        # print(aruco_array)
        output = np.max(mf(aruco_array, 40))
        print(output)
        # pub.publish(output)


if __name__ == '__main__':
    global list_aruco
    list_aruco = []
    rospy.init_node('preprocess', anonymous=True)
    rospy.Subscriber("/angle", Angle, callback)
    # pub = rospy.Publisher('/median_aruco', Float32, queue_size=10)
    rospy.spin()
