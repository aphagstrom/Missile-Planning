import numpy as np
from tools.rotations import rotation_to_euler, euler_to_rotation, quaternion_to_rotation
from abc import ABC, abstractmethod
# Class designed to allow easy access to model-specific state values (i.e. pos, vel, etc.) for each agent
class MsgState(ABC):
    """
    Checks that the init_state array is of the right length for the given model, and sets self.pos
    Arguments:
        - init_state: list of initial values for state
        - req_len: required length for init_state (this value is initiatedin child __init__ function)
    """
    @abstractmethod
    def __init__(self, init_state, req_len):  
        if len(init_state) != req_len:
            raise IndexError(str(self) + ": init_state has " + str(len(init_state)) + " elements (expected " + str(req_len) + ")")
        self.pos = np.array([init_state[:3]]).T  # position in inertial frame    

    """
    Prints each attribute of the state. Here, it prints position, since that is an attribute for every model type, but
    children print() functions should add other attributes according to their model type.
    """
    @abstractmethod
    def print(self):
        print("position =", self.pos)
        

class MsgState_6DOF(MsgState):
    def __init__(self, init_state):
        req_len = 16
        super().__init__(init_state, req_len)
        
        self.vel = np.array([init_state[3:6]]).T # body frame
        self.R = quaternion_to_rotation(init_state[6:10]) # body orientation (from quat to rotation)
        self.omega = np.array([init_state[10:13]]).T # angular rates (body frame)
        self.delta = np.array([init_state[13:]]).T # rudder angles
    
    def get_euler(self):
        return rotation_to_euler(self.R)

    def set_euler(self, Theta):
        return euler_to_rotation(Theta[0], Theta[1], Theta[2]) 
    
    def print(self):
        super().print()
        # print("velocity =", self.vel)
        # print("rotation =", self.R)
        # print("angular velocity =", self.omega)
        # print("rudder angles =", self.delta)
        

class MsgState_Stationary(MsgState):
    def __init__(self, init_state):
        req_len = 3
        super().__init__(init_state, req_len)
        
    def print(self):
        super().print()
        