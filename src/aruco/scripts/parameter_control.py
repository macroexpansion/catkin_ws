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
thresh_angle = 0.8  # gioi han de dung goc quay
max_vel_z = 0.3  # toc do quay toi da
max_vel_x = 0.1  # toc do di thang toi da
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
