#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
from aruco.msg import Angle
import numpy as np
import tf
from tf.transformations import *
import math
from tf import transformations as tm

def convert_to_parallel(distance_base, phi_data):
    pi = 3.14159
    angle_aruco = phi_data.angle_aruco
    angle_base = phi_data.angle_base
    if angle_aruco < 0 and angle_base < 0:
        angle_rot = pi/2  - angle_aruco
        angle_pll = angle_rot + angle_base
        x = distance_base*np.sin(angle_pll)
        y =  - distance_base*np.cos(angle_pll)

    print(x,y)

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

class AprilDetector:
    def __init__(self):
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback)
        self.pub = rospy.Publisher('/tag_pose',Point, queue_size=10)
        self.t = tf.TransformListener()
        self.pi = 3.14159
        self.q_rot = quaternion_from_euler(self.pi/2, 0, -self.pi/2) 
        self.q_rot[3] = -self.q_rot[3]

    def createVirturePose(self,mark):
        orient = mark.pose.pose.pose.orientation
        q_orig = np.array([orient.x,orient.y, orient.z, orient.w])
        q_new = quaternion_multiply(q_orig, self.q_rot)
        mark.pose.pose.pose.orientation.x = q_new[0]
        mark.pose.pose.pose.orientation.y = q_new[1]
        mark.pose.pose.pose.orientation.z = q_new[2]
        mark.pose.pose.pose.orientation.w = q_new[3]
        return mark
    def callback(self, data):
        point = Point()
        pose = PoseStamped()
        detecs = data.detections
        for mark in detecs:   
            angle = Angle()
            mark = self.createVirturePose(mark)
            pose.header = mark.pose.header
            pose.pose = mark.pose.pose.pose
            pose_base = self.t.transformPose("/base_link", pose)
            pos, ori = pose_base.pose.position, pose_base.pose.orientation

            
            rot = self.t.fromTranslationRotation((pos.x,pos.y,pos.z),(ori.x,ori.y,ori.z,ori.w))
            # print(rot)
            rot_inv = tm.inverse_matrix(rot)
            euler = rotationMatrixToEulerAngles(rot[:3,:3])
            # print(-rot_inv[:,-1])
            point.x = -rot_inv[0,-1]
            point.y = -rot_inv[1,-1]
            point.z = -euler[2]    
            # x = pose_base.pose.position.x
            # y = pose_base.pose.position.y
            # distance = np.linalg.norm(np.array([x,y]))
            # angle_base = np.arctan(y/x)
            # angle.angle_base = angle_base
            # angle.angle_aruco = euler[2]
            # angle.angle_cam = 0.0
            self.pub.publish(point)
            print(point)
            print('\n')


if __name__=="__main__":
    

    rospy.init_node('aprilControl')
    AprilDetector()
    rospy.spin()