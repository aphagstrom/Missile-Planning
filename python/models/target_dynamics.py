"""
targetDynamics 
    - Update history:  
        7/12/2023 - RWB
"""
import numpy as np
from message_types.msg_state import MsgState
import parameters.target_parameters as TARGET

class TargetDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        self._state = np.array([
            [TARGET.pos0[0,0]],  # (0)
            [TARGET.pos0[1,0]],  # (1)
            [TARGET.pos0[2,0]],  # (2)
            [TARGET.vel0[0,0]],  # (3)
            [TARGET.vel0[1,0]],  # (4)
            [TARGET.vel0[2,0]],  # (5)
        ])
        self.state = MsgState(pos=TARGET.pos0, 
                              vel=TARGET.vel0)

    ###################################
    # public functions
    def update(self):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self.__derivatives(self._state[0:6])
        k2 = self.__derivatives(self._state[0:6] + time_step/2.*k1)
        k3 = self.__derivatives(self._state[0:6] + time_step/2.*k2)
        k4 = self.__derivatives(self._state[0:6] + time_step*k3)
        self._state[0:6] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # update the message class for the true state
        self.__update_true_state()

    ###################################
    # private functions
    def __derivatives(self, state):
        north_dot = state[3,0]
        east_dot = state[4,0]
        down_dot = state[5,0]
        north_ddot = 0.
        east_ddot = 0.
        down_ddot = 0.
        # collect the derivative of the states
        x_dot = np.array([[north_dot], 
                          [east_dot], 
                          [down_dot],
                          [north_ddot], 
                          [east_ddot], 
                          [down_ddot]])
        return x_dot

    def __update_true_state(self):
        self.state.pos = self._state[0:3]
        self.state.vel = self._state[3:6]
