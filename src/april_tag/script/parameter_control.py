import numpy as np

""" 
Neu xe di qua vi tri mong muon giam offset_x, nguoc lai, tang len
Neu xe quay qua goc mong muon, tang k_distan_phi, nguoc lai giam 
Thay doi chieu dai robot len_robot
muon di nhanh hon tang max_vel_x
muon quay nhanh hon tang max_vel_z
Thay doi goc quay co dinh o state 1: fix_rot


"""
k_pll = 1.26  # he so ti le quang duong tien ra quang truc
kp = 0.02  # kp trong bo PID dieu khien goc
pi = 3.14159
k_p_phi = 0.01
offset_x = -0.00  # am se dung lai som
k_distan_phi = 1  # he so bu goc quay encoder de bang goc quay tu cam
thresh_angle = 0.03  # gioi han de dung goc quay
thresh_distance = 0.03  # gioi han de dung di thang
max_vel_z = 0.3  # toc do quay toi da
max_vel_x = 0.2  # toc do di thang toi da
len_robot = 1.3
fix_rot = 95  # goc quay co dinh o state 2
# mc = np.array([907.136, 0., 649.546, 0.,
#                906.922, 365.379, 0., 0., 1.]).reshape((3, 3))  # 1280x720
# dc = np.array([0., 0., 0., 0., 0.])

mc = np.array([604.758, 0., 430.364, 0.,
               604.615, 243.586, 0., 0., 1.]).reshape((3, 3))  # 848x480
dc = np.array([0., 0., 0., 0., 0.])

regression_param = [0.52603084, 2.9391792410073254]  # tham so lay sau khi training goc that - goc tu aruco

# dong hoc robot
r = 0.032  # ban kinh banh xe
b = 0.075  # 1 nua chieu rong xe

cam_param = [604.758, 604.615, 430.364, 243.586]

cam_param_fe2 = [2.9051629018791448e+02, 2.9122677351528995e+02, 4.2659384671446452e+02, 3.9942900274732108e+02]
# cam_param_fe2 = [79.47380829, 77.36890411, 116, 107]

mtx_fe2 = np.array([2.9051629018791448e+02, 0., 4.2659384671446452e+02, 0.,
                    2.9122677351528995e+02, 3.9942900274732108e+02, 0., 0., 1.]).reshape((3, 3))
dist_fe22 = np.array([-2.4349566329316419e-01, 5.0881089457320207e-02,
                      1.7767295723885013e-03, -8.2529896886837520e-04,
                      -4.3726864545650043e-03])
# mtx_fe2 = np.array(
#     [286.5538024902344, 0.0, 427.535400390625, 0.0, 286.6983947753906, 392.22589111328125, 0.0, 0.0, 1.0])
# dist_fe22 = np.array([-0.0066444771364331245, 0.04461219161748886, -0.041549328714609146, 0.007337781134992838, 0.0])

dist_fe2 = np.array([0, 0, 0, 0, 0])
