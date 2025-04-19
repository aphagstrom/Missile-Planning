from controllers.pid_control import PIDControl
from controllers.p_control import PControl
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
from abc import ABC, abstractmethod
import parameters.simulation_parameters as SIM

class Autopilot(ABC):
    def __init__(self, params):
        self.params = params
        # self.commanded_state = MsgState() # TODO: fix
        self.Ts = SIM.ts_simulation
    
    """
    Calculate commzand inputs based on current state of the vehicle and the commanded state
    Arguments:
        - cmd: the commanded state of the vehicle
        - state: the current state of the vehicle
    Returns:
        - command input (i.e. rudder angles, thrust command, etc.)
    """
    @abstractmethod   
    def update(self, cmd, state):
        pass
    
    """
    Saturates the command between given limits.
    Arguments:
        - low_limit: lower bound to saturate command
        - up_limit: upper bound to saturate command
        - input: the command to be saturated
    Returns:
        - output: the saturated command
    """
    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output

class AP_6DOF(Autopilot):
    def __init__(self, params):
        super().__init__(params)
        
        ap = self.params
        
        self.roll_controller = PIDControl(ap.roll_kp, ap.roll_ki, ap.roll_kd, self.Ts, ap.sigma, ap.delta_max, ap.roll_aw, ap.roll_aw_limit)
        self.yaw_controller = PControl(ap.yaw_kp, ap.delta_max)
        self.pitch_controller = PIDControl(ap.pitch_kp, ap.pitch_ki, ap.pitch_kd, self.Ts, ap.sigma, ap.delta_max, ap.pitch_aw, ap.pitch_aw_limit)
        self.altitude_controller = PIDControl(ap.alt_kp, ap.alt_ki, ap.alt_kd, self.Ts, ap.sigma, ap.pitch_limit, ap.alt_aw, ap.alt_aw_limit)

        self.delta_c = MsgDelta(0, 0, 0)
        
    def update(self, cmd, state):
        ap = self.params
        
        attitude = state.get_euler()
        # # lateral autopilot
        delta_x = self.saturate(self.roll_controller.update_with_rate(cmd.phi_command, attitude[0], state.omega[0, 0]), -ap.delta_max, ap.delta_max)
        delta_z = self.saturate(-self.yaw_controller.update(cmd.course_command, attitude[2]), -ap.delta_max, ap.delta_max)
            
        # longitudinal autopilot
        if cmd.altitude_command > 0:
            cmd.theta_command = self.altitude_controller.update(cmd.altitude_command, -state.pos[2, 0]) 
    
        delta_y = self.saturate(-self.pitch_controller.update_with_rate(cmd.theta_command, attitude[1], state.omega[1, 0]), -ap.delta_max, ap.delta_max)

        # construct control outputs and commanded states
        self.delta_c = MsgDelta(delta_x, delta_y, delta_z)
    
        