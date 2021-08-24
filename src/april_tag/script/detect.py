#!/usr/bin/env python

import apriltag
import rospy
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import parameter_control as params
from scipy.ndimage import median_filter as mf
import apriltag
import time
import tf
from tf.transformations import *
import math
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt


def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])


class kalman_filter:
    def __init__(self):
        self.A = 1.0
        self.B = 0
        self.Q = 1e-3  # given Q = cov(w)

        self.H = 1  # given H obser model
        self.R = 3e-1  # given R  = cov(v)
        self.P = 0
        self.U_hat = 0  # gia tri khoi tao uoc luong

    def update(self, U):
        # U la nhieu do duoc
        K = self.P * self.H / (self.H * self.P * self.H + self.R)
        self.U_hat = self.U_hat + K * (U - self.H * self.U_hat)
        self.P = (1 - K * self.H) * self.P + self.Q


class listener:
    def __init__(self):
        print("init")
        self._matrix_coefficients = params.mc
        self._distortion_coefficients = params.dc
        self.pi = params.pi
        self.lastime = time.time()
        self.detectImage = rospy.Publisher("/detectMarker", Image, queue_size=1)
        self.pub = rospy.Publisher("/tag_pose", Point, queue_size=1)
        self.detector = apriltag.Detector()
        self.br = CvBridge()
        rospy.Subscriber("/camera_front/color/image_raw", Image, self.get_color)
        rospy.Subscriber("/camera_front/aligned_depth_to_color/image_raw", Image, self.get_depth)
        self.cv_depth = np.zeros((480, 848)) - 1
        self.kalman_filter = kalman_filter()
        self.t = tf.TransformListener()
        self.matrix_depth_to_cam, self.matrix_base_to_depth, self.matrix_tag_to_vtag = self.calulate_matrix()

    def get_color(self, data):
        cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        # plt.imshow(cv_image)
        # plt.show()
        # print(cv_image.shape)
        pose = self.detect_tag(cv_image)
        print(pose)
        self.pub.publish(pose)
        now = time.time()
        print('timing: {:.3f}\n'.format(now - self.lastime))
        self.lastime = now

    def get_depth(self, data):
        self.cv_depth = CvBridge().imgmsg_to_cv2(data)
        # h, w = cv_depth.shape[:2]
        # cv_depth = cv_depth[71:391, 111:711]
        # self.cv_depth = cv2.resize(cv_depth, (w, h))

    def calulate_matrix(self):
        rospy.sleep(0.5)
        pos_depth_cam, ori_depth_cam = self.t.lookupTransform("camera_front_aligned_depth_to_color_frame",
                                                              "camera_front_color_optical_frame", rospy.Time())
        pos_base, ori_base = self.t.lookupTransform("base_link", "camera_front_aligned_depth_to_color_frame",
                                                    rospy.Time())
        matrix_depth_to_cam = self.t.fromTranslationRotation(pos_depth_cam, ori_depth_cam)
        matrix_base_to_depth = self.t.fromTranslationRotation(pos_base, ori_base)
        matrix_tag_to_vtag = euler_matrix(params.pi / 2, 0, params.pi / 2, 'rxyz')
        return matrix_depth_to_cam, matrix_base_to_depth, matrix_tag_to_vtag

    def detect_tag(self, img):
        point = Point()

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        result = self.detector.detect(img)
        for res in result:
            matrix_cam_to_tag, _, _ = self.detector.detection_pose(res, params.cam_param, tag_size=0.157)

            matrix_depth_to_tag = np.dot(self.matrix_depth_to_cam, matrix_cam_to_tag)
            matrix_base_to_tag = np.dot(self.matrix_base_to_depth, matrix_depth_to_tag)
            matrix_base_to_vtag = np.dot(matrix_base_to_tag, self.matrix_tag_to_vtag)
            matrix_vtag_to_base = inverse_matrix(matrix_base_to_vtag)
            
            theta = rotationMatrixToEulerAngles(matrix_base_to_vtag)
            x1, y1, z1, _ = -matrix_base_to_vtag[:, -1]
            distance = np.sqrt(x1 ** 2 + y1 ** 2)
            # print(x1, y1, theta[-1])
            # print('distance: ', distance)
            x, y, z, _ = -matrix_vtag_to_base[:, -1]
            y = np.sign(y)*np.sqrt(distance ** 2 - x ** 2)

            point.x = x
            point.y = y
            point.z = -theta[-1]

        return point


def main():
    rospy.init_node('detect_markers', anonymous=True)
    listener()
    rospy.spin()


if __name__ == '__main__':
    print('chieu dai xe: ', params.len_robot)
    main()
