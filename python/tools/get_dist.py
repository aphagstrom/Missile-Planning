import numpy as np

def get_dist(x1, x2):
    sum = 0
    for i in range(np.shape(x1)[0]):
        sum += (x1[i, 0] - x2[i, 0])**2
    return np.sqrt(sum)    