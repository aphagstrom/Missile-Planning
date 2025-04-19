"""
p_control
    - Shumway, 2023
    - Last Update:
        11/29/2023 - LDS
"""
import sys
import numpy as np
sys.path.append('..')


class PControl:
    # PD control with rate information
    # u = kp*(yref-y) - kd*ydot
    def __init__(self, kp=0.0, limit=1.0):
        self.kp = kp
        self.limit = limit

    def update(self, y_ref, y):
        u = self.kp * (y_ref - y)  
        # saturate PID control at limit
        u_sat = self._saturate(u)
        return u_sat

    def _saturate(self, u):
        # saturate u at +- self.limit
        if u >= self.limit:
            u_sat = self.limit
        elif u <= -self.limit:
            u_sat = -self.limit
        else:
            u_sat = u
        return u_sat