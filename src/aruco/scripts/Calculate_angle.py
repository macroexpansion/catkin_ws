import numpy as np
from math import sin
import parameter_control as params

pi = params.pi
"""calculate angle from center of robot to object base on angle from camera to object"""


def convert_center_cam(angle_cam, cam_offset, depth):
    angle_cam = angle_cam * params.pi / 180
    gocC = pi / 2 + angle_cam
    gocA = np.arcsin(cam_offset / depth * np.sin(gocC))
    gocB = pi - gocA - gocC
    return (pi / 2 - gocB) * 180 / pi


def convert_center_cam_to_base(angle, depth):
    angle_abs = np.abs(angle * params.pi / 180)
    gocA = pi - angle_abs
    a = slove_SAS_triangle(params.len_robot / 2, depth, gocA)
    gocBase = np.arcsin(depth / a * np.sin(gocA))
    return np.sign(angle) * gocBase * 180 / pi, a


def convert_angle(angle_cam, leng_robot, cam_offset, depth):
    angle_cam = angle_cam * params.pi / 180
    if angle_cam < 0:
        angle_cam = - angle_cam
        angle_cam_t = params.pi / 2 - angle_cam
        # depth = solve_SSA_triangle(cam_offset, depth, angle_cam_t)
        ac = leng_robot / 2 + cam_offset * np.tan(angle_cam_t)
        if depth > cam_offset / np.cos(angle_cam_t):
            ab = depth - cam_offset / np.cos(angle_cam_t)
            phi = params.pi - angle_cam
            sign = -1
        else:
            ab = - depth + cam_offset / np.cos(angle_cam_t)
            phi = angle_cam
            sign = 1
        bc = np.sqrt(ac ** 2 + ab ** 2 - 2 * ac * ab * np.cos(phi))
        phi_c = np.arccos((ac ** 2 + bc ** 2 - ab ** 2) / (2 * ac * bc))
        phi_c = sign * phi_c * 180 / params.pi
    else:
        angle_cam_t = params.pi / 2 - angle_cam
        # depth = solve_SSA_triangle(cam_offset, depth, angle_cam_t)
        ac = leng_robot / 2 - cam_offset * np.tan(angle_cam_t)
        ab = depth + cam_offset / np.cos(angle_cam_t)
        phi = params.pi - angle_cam
        bc = np.sqrt(ac ** 2 + ab ** 2 - 2 * ac * ab * np.cos(phi))
        phi_c = np.arccos((ac ** 2 + bc ** 2 - ab ** 2) / (2 * ac * bc))
        phi_c = phi_c * 180 / params.pi
    return phi_c, bc


def create_viture_point(angle_base, angle_tf_base_aruco, distance_to_aruco):
    angle_base = np.abs(angle_base / 180 * params.pi)
    angle_tf_base_aruco = np.abs(angle_tf_base_aruco / 180 * params.pi)
    distance_to_vp = distance_to_aruco / angle_tf_base_aruco * sin(angle_tf_base_aruco - angle_base)
    return distance_to_vp


def slove_SAS_triangle(a, b, phi):
    return np.sqrt(
        a ** 2 + b ** 2 - 2 * a * b * np.cos(phi))


def solve_SSA_triangle(a, b, gocB):
    gocA = np.arcsin(a / b * sin(gocB))
    gocC = pi - gocA - gocB
    c = b / sin(gocB) * sin(gocC)
    return c


def angle_pose_from_2_point(angle_to_left, angle_to_right, dis_to_left, dis_to_right):
    """Gia su co tam giac voi 3 dinh la 2 diem da cho va tam robot
    tinh goc giua truc robot voi trung truc noi giua 2 diem kia"""
    if angle_to_left == -1 or angle_to_right == -1:
        return -1, -1
    else:
        if angle_to_right * angle_to_left >= 0:
            sign = -1
        else:
            sign = +1
        if dis_to_right > dis_to_left:
            sign_rot = -1
        else:
            sign_rot = 1
        angle_to_left = np.abs(angle_to_left * params.pi / 180)
        angle_to_right = np.abs(angle_to_right * params.pi / 180)
        gamma = np.abs(angle_to_right + sign * angle_to_left)
        distance_2_point = slove_SAS_triangle(dis_to_left, dis_to_right, gamma)
        # k = 0.243 / distance_2_point
        #  dis_to_left = k * dis_to_left
        # dis_to_right = k * dis_to_right
        # distance_2_point = slove_SAS_triangle(dis_to_left, dis_to_right, gamma)
        print('dis2p: ', distance_2_point)
        # print('gamma: ', gamma * 180/pi)
        """based on https://www.mathsisfun.com/algebra/trig-solving-sas-triangles.html"""
        if dis_to_right < dis_to_left:  # goc trai la goc nhon
            sin_goc_trai = dis_to_right / distance_2_point * sin(gamma)
            goc_trai = np.arcsin(sin_goc_trai)
            beta = pi - gamma - goc_trai
        else:
            sin_beta = dis_to_left / distance_2_point * sin(gamma)
            beta = np.arcsin(sin_beta)
        # print('dl, dr', dis_to_left, dis_to_right)
        # print('beta', beta * 180 / pi)
        dis_to_cen = slove_SAS_triangle(dis_to_right, distance_2_point / 2, beta)
        pose = np.abs(beta + angle_to_right - pi / 2)
        # print('dis_to_cen', dis_to_cen)
        return sign_rot * pose * 180 / pi / params.k_pll, dis_to_cen
