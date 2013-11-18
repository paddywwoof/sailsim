import numpy as np
import time
import random

a = np.array([[random.random(), random.random(), random.random()] for i in range(100)])
t1 = time.time()
b = np.array(a)
d0 = np.subtract.outer(a[:,0], b[:,0])
d1 = np.subtract.outer(a[:,1], b[:,1])
d3 = np.hypot(d0, d1)
print time.time() - t1
