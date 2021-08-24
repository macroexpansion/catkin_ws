import json
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
import joblib
from scipy.ndimage import median_filter as mf



f1 = open("input.json")
inputs = json.load(f1)
a_b = []
d_b = []
gt_theta = []
gt_dis = []
af = []
for i in inputs:
    a_b.append(i["angle_base"])
    d_b.append(i["distance"])
    gt_theta.append(i["sptheta"])
    gt_dis.append(i["spdis"])
    af.append(i["angleFiltered"])

d_b = np.asarray(d_b)
gt_dis = np.asarray(gt_dis)
a_b = np.asarray(a_b)

gt_dis = gt_dis[np.where(d_b > 0.7)[0]]
d_b = d_b[np.where(d_b > 0.7)[0]]
a_b = a_b[np.where(d_b > 0.7)[0]]

gt_dis = gt_dis[np.where(d_b < 4)[0]].reshape((-1, 1))
d_b = d_b[np.where(d_b < 4)[0]].reshape((-1, 1))
a_b = a_b[np.where(d_b < 4)[0]].reshape((-1, 1))

a_b = mf(a_b, size=10)
d_b = mf(d_b, size=10)

X = np.concatenate((d_b, a_b), 1)
reg = LinearRegression().fit(X, gt_dis)
d_pred = reg.predict(X)

regr = RandomForestRegressor(max_depth=10, random_state=0)
# regr = joblib.load('rf5.joblib')
regr.fit(X, gt_dis)

joblib.dump(regr, "rf10.joblib", protocol=2)

ranFor_pred = regr.predict(X)

L = np.linalg.norm(ranFor_pred.reshape((-1, 1)) - gt_dis)
print(L)

# plt.plot(ranFor_pred)
# # plt.plot(d_pred)
# plt.plot(gt_dis)
# # plt.plot(a_b)
# plt.show()
