"""
msg_autopilot
    - messages type for input to the autopilot
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/5/2019 - RWB
"""


class MsgAutopilot:
    def __init__(self):
        self.course_command = 0.0  # commanded course angle in rad
        self.altitude_command = 0.0  # commanded altitude in m
        self.phi_command = 0.0  # command for roll angle
        self.theta_command = 0.0 # command for pitch angle
