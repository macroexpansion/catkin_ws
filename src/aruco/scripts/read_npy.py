import numpy as np
import matplotlib.pyplot as plt

x = np.load('x.npy')
y = np.load('y.npy')

plt.plot(x)
plt.plot([0, 200], [0.6, 0.6])

plt.figure()
plt.plot(y)
plt.plot([0, 200], [0.4, 0.4])
plt.show()
