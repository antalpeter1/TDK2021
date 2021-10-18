import numpy as np
import matplotlib.pyplot as plt


t = np.linspace(0, 1, 1000)
y = [x[0].derivative().basis(t_)[0, 10] for t_ in t]

plt.figure()
plt.plot(y)
plt.show()