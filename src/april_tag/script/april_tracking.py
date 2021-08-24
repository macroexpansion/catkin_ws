#!/usr/bin/env python

from __future__ import print_function
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import parameter_control as params
import apriltag
import tf
from tf.transformations import *
from geometry_msgs.msg import Point, Twist

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


class Tracking:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("//camera_front/color/image_raw", Image, self.tracking)
        self.detector = apriltag.Detector()
        self.br = CvBridge()
        self.t = tf.TransformListener()
        self.matrix_base_to_cam, self.matrix_tag_to_vtag = self.calculate_matrix()
        self.last_vel_time_ = rospy.get_time()

    def tracking(self, data):
        cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        pose, heading = self.detect_tag(cv_image)
        k_p = 0.5
        if np.abs(heading) > 0.02 and heading != 0:
            vel_z = heading * k_p
            vel_z = np.clip(vel_z, -params.max_vel_z, params.max_vel_z)

            try:
                self.controller(0, vel_z)
            except rospy.ROSInterruptException:
                pass
        now = rospy.get_time()
        print('timing: {:.3f}\n'.format(now - self.last_vel_time_))
        self.last_vel_time_ = now

    def controller(self, vel_x, vel_z):
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vel_z

        self.pub.publish(twist)

    def calculate_matrix(self):
        rospy.sleep(0.5)
        pos_base, ori_base = self.t.lookupTransform("base_link", "camera_front_color_optical_frame",
                                                    rospy.Time())
        matrix_base_to_cam = self.t.fromTranslationRotation(pos_base, ori_base)
        matrix_tag_to_vtag = euler_matrix(params.pi / 2 + 0.04, 0, params.pi / 2 + 0.02, 'rxyz')
        return matrix_base_to_cam, matrix_tag_to_vtag

    def detect_tag(self, img):
        point = Point()

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        result = self.detector.detect(img)
        for res in result:
            matrix_cam_to_tag, _, _ = self.detector.detection_pose(res, params.cam_param, tag_size=0.12)

            matrix_base_to_tag = np.dot(self.matrix_base_to_cam, matrix_cam_to_tag)
            matrix_base_to_vtag = np.dot(matrix_base_to_tag, self.matrix_tag_to_vtag)
            matrix_vtag_to_base = inverse_matrix(matrix_base_to_vtag)

            theta = rotationMatrixToEulerAngles(matrix_base_to_vtag)
            print('theta base to vtag: ', theta)
            x1, y1, z1, _ = -matrix_base_to_vtag[:, -1]
            distance = np.sqrt(x1 ** 2 + y1 ** 2)
            x, y, z, _ = -matrix_vtag_to_base[:, -1]
            heading = np.arctan(y1 / x1)
            print('heading', heading)
            point.x = x
            point.y = y
            point.z = z

        return point, heading


def main():
    rospy.init_node('aruco_tracking', anonymous=True)
    Tracking()

    rospy.spin()


if __name__ == '__main__':
    main()
