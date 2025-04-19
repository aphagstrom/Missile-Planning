import numpy as np
import copy
class Agent():
    """
    Instantiates Agent object. Child __init__ function must call this in addition to adding their own attributes
    Arguments:
        - id: integer to identify agent from others of its own type (i.e. ally vehicle, target, etc.)
    """

    def __init__(self, id, exists = True):
        self.id = id
        # Bool to track whether object exists or not in simulation
        self.exists = exists
  
    
    """
    Executes an update of one time step to update the agent's path and autopilot (if necessary for model type) and 
    calls the model's update function to propogate dynamics.
    """
    def update_dynamics(self):
        self.state = self.dynamics.update(self.state)
    
    

    def set_dynamics(self, model, init_state):
        """
        - model: Model object that contains the dynamics for the agent
        - init_state: list containing the initial state values for the agent
        """
        self.dynamics = model 
        self.state = self.dynamics.init_state(init_state) # MsgState object with attributes (i.e. pos, vel. etc.) that correspond to the model

    def update_all(self):
        
        path = self.path_manager.update(self)
        print("PATH AIRSPEED IS",path.airspeed)
        print("PATH LINE DIR IS",path.line_direction)
        print("PATH LINE ORIGIN IS",path.line_origin)
        print("PATH ORBIT CENTER IS",path.orbit_center)
        print("PATH ORBIT DIR IS",path.orbit_direction)
        autopilot_commands = self.path_follower.update(path, self.state, self.target.state)
        # print("COURSE COMMAND IS:",autopilot_commands.course_command)
        # print("ALTITUDE COMMAND IS:",autopilot_commands.altitude_command)
        # print("PHI COMMAND IS:",autopilot_commands.phi_command)
        # print("THETA COMMAND IS:",autopilot_commands.theta_command)
        
        # ------- set references ----------------
        self.dynamics.autopilot.update(autopilot_commands, self.state)
        
        # -------update physical system-------------
        self.update_dynamics()
        self.check_ground_collision()
        # self.update_guidance()

    """
    Updates the target assigned to the agent. 
    # """
    def set_target(self, target_object):
        self.target= target_object
        self.target.pos = copy.deepcopy(target_object.state.pos)
    def set_guidance(self, guidance):
        self.guidance = guidance
        self.update_guidance()
    def get_airspeed(self):
        if hasattr(self.dynamics, 'get_airspeed'):
            return self.dynamics.get_airspeed()
        else:
            raise AttributeError("The dynamics model does not have a method to get airspeed.")
    def update_guidance(self):
        self.desired = self.guidance.update(self.state, self.target.state)
    
    def set_weapon_effectiveness(self, effectiveness):
        self.effectiveness = effectiveness

    def set_effective_zone(self, zone):
        self.effective_zone = zone

    def set_communications(self, range):
        self.comms_range = range

    def set_attrition(self, attrition):
        self.attrition = attrition

    def set_priority(self, priority):
        self.priority = priority

    def check_ground_collision(self):
        if self.state.pos[2, 0] > 0.01:
            self.exists = False
    def calculate_yaw(self):
        if self.state is not None:
            euler_angles = self.state.get_euler()  
            yaw = euler_angles[2] 
            return yaw
        else:
            return None  