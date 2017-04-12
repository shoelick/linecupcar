#!/usr/bin/env python3

from math import *
import matplotlib.pyplot as plt
import numpy as np

import code

kernel = [-1, 0, 1]

data = []
deriv2 = []
derivc = []
derivcn = []

for i in range(0, 360):
    data.append(sin(radians(i)))

print(data)

derivc = np.convolve(data, kernel)

for i in range(0, 358):
    deriv2.append((data[i] - data[i + 1])/2 * 10)


#code.interact(local=locals())

#plt.plot(derivc)
plt.plot([i * (1 / max(derivc)) for i in derivc])
plt.plot(data)
plt.show()

