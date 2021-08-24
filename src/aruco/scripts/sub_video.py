#!/usr/bin/env python
import sys

# print(sys.version)

import rospy
import cv2
from std_msgs.msg import String
from aruco.msg import Angle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import message_filters
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, atan
from Calculate_angle import create_viture_point, convert_angle
import parameter_control as params
import time


class image_convert_pub:

    def __init__(self):
        self.image_pub = rospy.Publisher("/detected_markers", Image, queue_size=1)
        self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
        self.distance = rospy.Publisher("/distance", String, queue_size=1)
        self.angle = rospy.Publisher("/angle", Angle, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.image_depth = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.image_depth], 20)
        self.ts.registerCallback(self.callback)

    def callback(self, data_image, data_depth):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data_image, 'bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(data_depth)
            # cv_depth = cv_depth[60:400, 80:700]
            # cv_depth = cv2.resize(cv_depth, (1280, 720))

        except CvBridgeError as e:
            print(e)

        markers_img, ids_list, dis, ang = self.detect_aruco(cv_image, cv_depth)

        if ids_list is None:
            self.id_pub.publish(ids_list)
        else:
            # print(ids_list)
            ids_str = ''.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)
            self.distance.publish(str(dis))
            self.angle.publish(ang)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def detect_aruco(self, img, depth):
        global lastime
        angle_pub = Angle()
        pi = params.pi
        tuning_angle_cam = params.tuning_angle_cam
        len_robot = params.len_robot
        _matrix_coefficients = params.mc
        _distortion_coefficients = params.dc
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        distance_base = -1
        angle_base = -1
        angle_cam = -1
        angle_aruco = -1
        dis_to_vp = -1
        for i in range(len(corners)):
            center = (corners[i][0][0] + corners[i][0][1] + corners[i][0][2] + corners[i][0][3]) // 4
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, _matrix_coefficients,
                                                                       _distortion_coefficients)
            (rvec - tvec).any()
            aruco.drawAxis(img, _matrix_coefficients, _distortion_coefficients, rvec, tvec, 0.01)
            distance = 0.001 * depth[int(center[1]), int(center[0])]
            # print(distance)
            if ids[i] == 888:
                # print(rvec[0][0] * 180 / pi)
                # print('vector: ',np.abs(rvec[0][0][1] * 180 / pi))
                # cv2.circle(img, tuple(center), 3, (255, 255, 255), -1)
                # cv2.line(img, tuple(center), (640, 720), (255, 0, 255), 2)
                # cv2.line(img, (640, 720), (640, 420), (255, 255, 0), 2)
                angle2d = np.arctan((720 - center[1]) / (center[0] - 640 + 0.001)) * 180 / pi
                Ki = np.linalg.inv(_matrix_coefficients)
                r2 = Ki.dot([center[0], center[1], 1.0])
                r1 = Ki.dot([640, 360, 1.0])
                cos_angle = r1.dot(r2) / (np.linalg.norm(r1) * np.linalg.norm(r2))
                angle_cam = np.arccos(cos_angle) * 180 / pi
                if angle2d > 0:
                    angle_cam = -angle_cam + tuning_angle_cam
                else:
                    angle_cam = angle_cam - tuning_angle_cam
                angle_base, distance_base = convert_angle(angle_cam, leng_robot=len_robot, cam_offset=0.04,
                                                          depth=distance)
                now = time.time()
                print('timing: ', now - lastime, angle_base)
                lastime = now
                #
                # cv2.putText(img, "angle cam: " + str(angle_cam), (50, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                # cv2.putText(img, "angle base: " + str(angle_base), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
                # cv2.putText(img, "distance: " + str(distance), (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0),
                #             2)
                # angle_aruco = 180 + rvec[0][0][1] * 180 / pi

                # dis_to_vp = create_viture_point(angle_base, angle_aruco, distance_base)
        # output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.

        angle_pub.angle_cam = float(angle_cam)
        angle_pub.angle_base = float(angle_base)
        angle_pub.angle_aruco = float(angle_aruco)
        return img, ids, distance_base, angle_pub


def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the aruco-ID in front of camera")
    ic = image_convert_pub()
    rospy.spin()


if __name__ == '__main__':
    global lastime
    lastime = time.time()
    main()
