"""
mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
        2/24/2020 - RWB
        7/13/2023 - RWB
"""
import numpy as np
from math import sin, cos

from message_types.msg_autopilot import MsgAutopilot
from tools.angles import wrap
from planners.dubins_parameters import DubinsParameters


class PathFollower:
    def __init__(self, model):
        self.chi_inf = np.radians(50)  # approach angle for large distance from straight-line path
        self.k_path = model.params.k_line  # path gain for straight-line path following
        self.k_orbit = model.params.k_orbit #10.0  # path gain for orbit following
        self.gravity = model.params.gravity
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot
        self.dubins_parameters = DubinsParameters(model)

    def update(self, path, state, target_state):
        self.autopilot_commands.phi_command = 0.0
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        elif path.type == 'terminal':
            self._hit_target(path, state, target_state)
            print("switching to pronav")
        return self.autopilot_commands
            
        # 
        #         print("norm is:",np.linalg.norm(state.pos[:2] - target_state.pos[:2]))
     

    def _hit_target(self, path, state, target_state):
        # lateral autopilot
        self._follow_straight_line(path, state)
        attitude = state.get_euler()
        # self.autopilot_commands.course_command = wrap(np.arctan2(state.LOS[1, 0], state.LOS[0, 0]), attitude[2])
        LOS = target_state.pos - state.pos
        self.autopilot_commands.theta_command = -np.arctan2(LOS[2, 0], np.sqrt(LOS[1,0]**2 + LOS[0, 0]**2))
        self.autopilot_commands.altitude_command = 0.0

    def _follow_straight_line(self, path, state):
        attitude = state.get_euler()
        print("ATTITUDE IS:",attitude)
        chi_q = np.arctan2(path.line_direction.item(1),
                           path.line_direction.item(0))
        chi_q = wrap(chi_q, attitude[2])
        print("CHIq is",chi_q)
        ep = np.array([[state.pos[0, 0]], [state.pos[1, 0]], [state.pos[2, 0]]]) - path.line_origin
        # path_error = -sin(chi_q) * (state.pos[0, 0] - path.line_origin.item(0)) \
        #              + cos(chi_q) * (state.pos[1, 0] - path.line_origin.item(1))
        path_error = -sin(chi_q) * ep.item(0) + cos(chi_q) * ep.item(1)
        # course command
        self.autopilot_commands.course_command \
            = chi_q - self.chi_inf * (2 / np.pi) * np.arctan(self.k_path * path_error)
        # altitude command
        n = np.cross(np.array([[0, 0, 1]]), path.line_direction.T).T
        n = n / np.linalg.norm(n)
        s = ep - (ep.T @ n) * n
        self.autopilot_commands.altitude_command \
            = -path.line_origin.item(2) \
              - np.sqrt((s.item(0))**2 + (s.item(1))**2) * path.line_direction.item(2) \
              / np.sqrt(path.line_direction.item(0)**2 + path.line_direction.item(1)**2)

    def _follow_orbit(self, path, state):
        attitude = state.get_euler()
        if path.orbit_direction == 'CW':
            direction = 1.0
        else:
            direction = -1.0
        # distance from orbit center
        d = np.sqrt((state.pos[0, 0] - path.orbit_center.item(0))**2
                    + (state.pos[1, 0] - path.orbit_center.item(1))**2)
        # compute wrapped version of angular position on orbit
        varphi = np.arctan2(state.pos[1, 0] - path.orbit_center.item(1),
                            state.pos[0, 0] - path.orbit_center.item(0))
        varphi = wrap(varphi, attitude[2])
        # compute normalized orbit error
        orbit_error = (d - path.orbit_radius) / path.orbit_radius
        # course command
        self.autopilot_commands.course_command \
            = varphi + direction * (np.pi/2.0 + np.arctan(self.k_orbit * orbit_error))
        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)



