"""
wrap chi_1, so that it is within +-pi of chi_2
"""
import numpy as np

def wrap(chi_1, chi_2):
    while chi_1 - chi_2 > np.pi:
        chi_1 = chi_1 - 2.0 * np.pi
    while chi_1 - chi_2 < -np.pi:
        chi_1 = chi_1 + 2.0 * np.pi
    return chi_1

def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

def bound_angle(angle):
    while angle > 2*np.pi:
        angle -= 2*np.pi
    while angle < 0:
        angle += 2*np.pi  
        
    return angle

def inHalfSpace(pos, halfspace_r, halfspace_n):
        if (pos - halfspace_r).T @ halfspace_n >= 0:
            return True
        else:
            return False
