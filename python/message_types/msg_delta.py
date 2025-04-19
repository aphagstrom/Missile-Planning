"""
msgDelta
    - messages type for inputs
    - Update history:  
        7/12/2023 - RWB
"""
import numpy as np


class MsgDelta:
    def __init__(self,
                 x_rudder=0.,
                 y_rudder=0.,
                 z_rudder=0.,
                 ):
        self.x_rudder = x_rudder  # commanded x-rudder angle
        self.y_rudder = y_rudder  # commanded y-rudder angle
        self.z_rudder = z_rudder  # commanded z-rudder angle
            