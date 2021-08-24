#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import parameter_control as params
import apriltag
import time
import tf
from tf.transformations import *
import math
from geometry_msgs.msg import Point, Twist

from aruco.msg import vel


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


class Listener:
    def __init__(self):
        print("init")
        self.last_time = time.time()
        self.detector = apriltag.Detector()
        rospy.Subscriber("/fisheye2/image_raw", Image, self.get_color)
        self.pub_speed = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.t = tf.TransformListener()
        self.last_vel_time_ = rospy.get_time()
        self.set_point_rot = None
        w, h = 848, 800
        self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(params.mtx_fe2, params.dist_fe22, (2 * w, 2 * h),
                                                                      1,
                                                                      (2 * w, 2 * h))

        self.matrix_base_to_cam = euler_matrix(-1.58391257, 0, 0, 'rxyz')
        self.matrix_base_to_cam[:, -1] = [0.02, 0.26, 0.08, 1]
        self.matrix_tag_to_vtag = euler_matrix(1.58391257, 0, 0, 'rxyz')

        self.matrix_base_to_cam_front = euler_matrix(-1.58391257, 0, -1.58391257)
        self.matrix_base_to_cam_front[:, -1] = [0.63456555, 0.03233302, 0.12281542, 1]
        self.matrix_tag_to_vtag_front = euler_matrix(params.pi / 2, 0, params.pi / 2, 'rxyz')

        self.go_straight = 1
        self.finetune_x = 0
        self.rotate = 0
        self.check_visible = 0
        self.tuning_heading = 0
        self.forwarding = 0
        self.set_forward = 0

        self.offsetX = None
        self.times_reach = 0
        self.goal = None
        self.x = 0
        self.y = 0
        self.theta = 0

    def get_color(self, data):

        if self.go_straight == 1:
            self.GoToXaxis(data)
        if self.finetune_x == 1:
            self.FinetuneX(data)
        if self.rotate == 1:
            self.Rotation()
        if self.check_visible == 1:
            self.check()
        if self.tuning_heading == 1:
            self.tuning_head()
        if self.forwarding == 1:
            self.forward()

        now = time.time()
        print('timing: {:.3f}\n'.format(now - self.last_time))
        self.last_time = now

    def FinetuneX(self, data):
        if self.offsetX is None:
            cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            un_dis = cv2.undistort(cv_image, params.mtx_fe2, params.dist_fe22, None, self.new_camera_mtx)
            pose, _ = self.detect_tag(un_dis, params.cam_param_fe2, self.matrix_base_to_cam, self.matrix_tag_to_vtag)
            self.offsetX = pose.x
        print('finetune x: ', self.offsetX)
        odom = rospy.wait_for_message("/raw_vel", vel)
        linear_x = odom.linear_x
        current_time = rospy.get_time()
        vel_dt_ = current_time - self.last_vel_time_
        self.last_vel_time_ = current_time
        delta_linear = linear_x * vel_dt_
        self.offsetX -= delta_linear
        if np.abs(self.offsetX) >= 0.01:
            try:
                self.pI(np.sign(self.offsetX)*0.1, 0)
            except rospy.ROSInterruptException:
                pass
        else:
            self.finetune_x = 0
            self.rotate = 1

    def check(self):
        data = rospy.wait_for_message("/camera_front/color/image_raw", Image)
        cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        pose2, heading = self.detect_tag(cv_image, params.cam_param, self.matrix_base_to_cam_front,
                                         self.matrix_tag_to_vtag_front)
        if pose2.x == 0:
            print('going back for visible...')
            try:
                self.pI(-0.1, 0)
            except rospy.ROSInterruptException:
                pass
        else:
            rospy.sleep(0.5)

            self.check_visible = 0
            self.tuning_heading = 1

    def Rotation(self):
        data = rospy.wait_for_message("/raw_vel", vel)
        angular_velocity_z_ = data.angular_z
        current_time = rospy.get_time()
        vel_dt_ = current_time - self.last_vel_time_
        self.last_vel_time_ = current_time
        delta_heading = angular_velocity_z_ * vel_dt_
        self.set_point_rot -= delta_heading
        kp = 2
        if self.set_point_rot > params.thresh_angle:
            print('rotating: ', self.set_point_rot)
            vel_z = np.abs(self.set_point_rot * kp)
            vel_z = np.clip(vel_z, -params.max_vel_z, params.max_vel_z)
            try:
                self.pI(0, vel_z)
            except rospy.ROSInterruptException:
                pass
        else:
            print('finish rotation')
            rospy.sleep(0.5)
            self.rotate = 0
            self.check_visible = 1

    def tuning_head(self):

        data = rospy.wait_for_message("/camera_front/color/image_raw", Image)
        cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        pose2, heading = self.detect_tag(cv_image, params.cam_param, self.matrix_base_to_cam_front,
                                         self.matrix_tag_to_vtag_front)
        heading = pose2.z
        if np.abs(heading) > params.thresh_angle:
            print('in tuning head: ', heading)
            vel.z = heading * 1.2
            vel_z = np.clip(vel.z, -params.max_vel_z, params.max_vel_z)
            try:
                self.pI(0, vel_z)
            except rospy.ROSInterruptException:
                pass
        else:
            print('finish tuning head: ', np.sqrt(pose2.x ** 2 + pose2.y ** 2))
            rospy.sleep(0.5)
            self.tuning_heading = 0
            self.forwarding = 1
            self.set_forward = np.sqrt(pose2.x ** 2 + pose2.y ** 2) - 0.5
            self.last_vel_time_ = rospy.get_time()

    def forward(self):
        data = rospy.wait_for_message("/raw_vel", vel)
        linear_x = data.linear_x
        current_time = rospy.get_time()
        vel_dt_ = current_time - self.last_vel_time_
        self.last_vel_time_ = current_time
        delta_linear = linear_x * vel_dt_
        self.set_forward -= delta_linear

        if np.abs(self.set_forward) > params.thresh_distance:
            print('in forward', self.set_forward)
            vel_x = self.set_forward * 0.3
            vel_x = np.clip(vel_x, -params.max_vel_x, params.max_vel_x)
            try:
                self.pI(vel_x, 0)
            except rospy.ROSInterruptException:
                pass
        else:
            print('finished!')

    def moveToPose(self, Kp_rho=9, Kp_alpha=15, Kp_beta=-3):
        data = rospy.wait_for_message("/raw_vel", vel)
        linear_x = data.linear_x
        angular_z = data.angular_z
        current_time = rospy.get_time()
        vel_dt_ = current_time - self.last_vel_time_
        self.last_vel_time_ = current_time
        theta_goal = 0
        x_goal, y_goal = self.goal.x, self.goal.y
        self.theta += angular_z * vel_dt_
        self.x += linear_x * vel_dt_ * np.cos(self.theta)
        self.y += linear_x * vel_dt_ * np.sin(self.theta)
        print(x_goal, y_goal, self.x, self.y)
        x_diff = x_goal - self.x
        y_diff = y_goal - self.y
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - self.theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - self.theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta
        v = np.clip(v, -params.max_vel_x, params.max_vel_x)
        w = np.clip(w, -params.max_vel_z, params.max_vel_z)
        if rho > 0.001:
            try:
                self.pI(v, w)
            except rospy.ROSInterruptException:
                pass

    def GoToXaxis(self, data):
        cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        un_dis = cv2.undistort(cv_image, params.mtx_fe2, params.dist_fe22, None, self.new_camera_mtx)
        pose, _ = self.detect_tag(un_dis, params.cam_param_fe2, self.matrix_base_to_cam, self.matrix_tag_to_vtag)
        # pose.x += 0.06
        kp = 0.2
        if np.abs(pose.x) > 0.15:
            print('start go to x axis', pose.x)
            vel_x = pose.x * kp
            vel_x = np.clip(vel_x, -params.max_vel_x, params.max_vel_x)
            vel_z = pose.z / 2
            vel_z = np.clip(vel_z, -params.max_vel_z, params.max_vel_z)
            try:
                self.pI(vel_x, vel_z)
            except rospy.ROSInterruptException:
                pass
        elif pose.x != 0:
            print('reach to x axis', pose.x)
            self.times_reach += 1

        if self.times_reach >= 5:
            print('finish go to x axis', pose.x)
            rospy.sleep(0.5)
            pose.z = -pose.z + params.pi / 2
            self.set_point_rot = params.pi / 2
            self.go_straight = 0
            self.finetune_x = 1
            self.last_vel_time_ = rospy.get_time()

    def pI(self, vel_x, vel_z):
        # while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = vel_z
        self.pub_speed.publish(twist)

    def detect_tag(self, img, cam_params, matrix_base_to_cam, matrix_tag_to_vtag):
        point = Point()
        result = self.detector.detect(img)
        heading = 0
        for res in result:
            matrix_cam_to_tag, _, _ = self.detector.detection_pose(res, cam_params, tag_size=0.157)
            matrix_base_to_tag = np.dot(matrix_base_to_cam, matrix_cam_to_tag)
            matrix_base_to_vtag = np.dot(matrix_base_to_tag, matrix_tag_to_vtag)
            matrix_vtag_to_base = inverse_matrix(matrix_base_to_vtag)
            theta = rotationMatrixToEulerAngles(matrix_base_to_vtag)
            x, y, z, _ = -matrix_vtag_to_base[:, -1]
            x_hat, y_hat, z_hat, _ = matrix_base_to_vtag[:, -1]
            point.x = x
            point.y = y
            point.z = theta[-1]
            heading = np.arctan(y_hat / x_hat)
            # debug
            matrix_cam_to_vtag = np.dot(matrix_cam_to_tag, matrix_tag_to_vtag)
            matrix_vtag_cam = inverse_matrix(matrix_cam_to_vtag)
            # print(matrix_vtag_cam[:, -1])

        return point, heading


def main():
    rospy.init_node('detect_markers', anonymous=True)
    Listener()
    rospy.spin()


if __name__ == '__main__':
    print('chieu dai xe: ', params.len_robot)
    main()
