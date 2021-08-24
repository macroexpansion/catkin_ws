import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

y = np.array([13, 25, 16, 3.6, 16, 4, 12.8, 3, 5, 13])  # ground truth
x = np.array([14, 43, 24, 0.5, 25, 3, 19.4, 3, 4, 20]).reshape((-1, 1))  # cam measurement

reg = LinearRegression().fit(x, y)
a = reg.coef_[0]
b = reg.intercept_
pred = x * a + b

plt.plot(x, y, '.r')
plt.plot(x, pred)
plt.show()
