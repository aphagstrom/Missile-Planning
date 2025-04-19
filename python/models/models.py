from tools.rotations import quaternion_to_rotation
import numpy as np
from parameters import hsv_6DOF_parameters, stationary_target_parameters, simulation_parameters
import controllers.autopilot as ap
from abc import abstractmethod, ABC
import message_types.msg_state as msg_state

class Model(ABC):
    
    """
    Instantiate Model object with params file and time-step
    Arguments:
      - params: params file associated with the specific model (this is instantiated in child __init__ function)
    Child function will also instantiate an autopilot (if necessary) to attach as an attribute to self
    """
    @abstractmethod
    def __init__(self, params):
        self.Ts = simulation_parameters.ts_simulation
        self.params = params
    
    """
    Instantiate MsgState object that corresponds to ModelX and store raw vector form of state
    Arguments:
      - init_state: list of initial state values
      - MsgState_ModelX: the MsgState object that corresponds to ModelX
    Returns:
      - MsgState object with attributes corresponding to the model (i.e. pos, vel, etc.)
    """
    @abstractmethod
    def init_state(self, init_state, MsgState_ModelX):
        state = MsgState_ModelX(init_state) # checks that state length is correct and creates MsgState object
        self._state = np.array(init_state) # store state in raw vector form
        return state
    
    """
    Calculates state propogation inputs (i.e. forces, moments, rudder angles, etc.) and calls RK4() and update_true_state() to propogate dynamics
    Arguments:
      - state: MsgState object corresponding to agent
    Returns:
      - new_state: updated MsgState object with propogated dynamics
    """
    @abstractmethod
    def update(self, state):
        pass
    
    """
    Calculate the time derivative of the state vector for given inputs
    Arguments:
      - _state: the agent's state (in raw vector form)
      - inputs: inputs to execute state propogation (i.e. forces, moments, rudder deflection angles, etc.)
    Returns:
      - xdot: the time derivative of the state vector
    """
    @abstractmethod
    def derivatives(self, _state, inputs):
        pass
    
    """
    Updates MsgState object with propogated state values 
    Arguments:
      - state: MsgState object that needs to be updated
    Returns:
      - new_state: updated MsgState object with propogates state values
    """
    @abstractmethod
    def update_true_state(self, state):
        pass
    
    """
    Performs 4th-order Runge-Kutta to propogate state dynamics (updates self._state)
    Arguments:
      - inputs: inputs to execute state propogation (i.e. forces, moments, rudder deflection angles, etc.)
    """
    def RK4(self, inputs):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        k1 = self.derivatives(self._state, *inputs)
        k2 = self.derivatives(self._state + self.Ts/2.*k1, *inputs)
        k3 = self.derivatives(self._state + self.Ts/2.*k2, *inputs)
        k4 = self.derivatives(self._state + self.Ts*k3, *inputs)
        self._state += self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)
 
class HSV_6DOF(Model):
    def __init__(self):
        params = hsv_6DOF_parameters
        super().__init__(params)
        
        self.autopilot = ap.AP_6DOF(params)
        
    def init_state(self, init_state):
        state = super().init_state(init_state, msg_state.MsgState_6DOF)
        self.__update_velocity_data(state)
        return state
    
    def update(self, state):
        forces_moments = self.__forces_moments(state)
        
        self.RK4([forces_moments, self.autopilot.delta_c])

        # normalize the quaternion
        e0, e1, e2, e3 = self._state[6:10]
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6:10] = np.array([e0, e1, e2, e3])/normE

        # update the message class for the true state
        new_state = self.update_true_state(state)
        
        # update the airspeed, angle of attack, and side slip angles using new state
        self.__update_velocity_data(new_state)
        
        return new_state
    def get_airspeed(self):
        return self._Va
        
    def derivatives(self, _state, forces_moments, delta_c):
        DOF6 = self.params
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        u, v, w = _state[3:6]
        e0, e1, e2, e3 = _state[6:10]
        p, q, r = _state[10:13]
        delta_x, delta_y, delta_z = _state[13:]
        #   extract forces/moments
        fx, fy, fz, Mx, My, Mz = forces_moments

        # position kinematics
        R = quaternion_to_rotation([e0, e1, e2, e3]) # body to world frame
        pos_dot = R @ np.array([[u], [v], [w]])
        north_dot, east_dot, down_dot = pos_dot.flatten()

        # position dynamics
        u_dot = r*v - q*w + fx/DOF6.mass
        v_dot = p*w - r*u + fy/DOF6.mass
        w_dot = q*u - p*v + fz/DOF6.mass

        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)

        # rotatonal dynamics
        p_dot = ((DOF6.Jy-DOF6.Jz)/DOF6.Jx)*q*r + Mx/DOF6.Jx
        q_dot = ((DOF6.Jz-DOF6.Jx)/DOF6.Jy)*p*r + My/DOF6.Jy
        r_dot = ((DOF6.Jx-DOF6.Jy)/DOF6.Jz)*q*p + Mz/DOF6.Jz

        # rudder dynamics
        delta_x_dot = (delta_c.x_rudder - delta_x) / DOF6.tau_x
        delta_y_dot = (delta_c.y_rudder - delta_y) / DOF6.tau_y
        delta_z_dot = (delta_c.z_rudder - delta_z) / DOF6.tau_z

        # collect the derivative of the states
        x_dot = np.array([north_dot, east_dot, down_dot, 
                           u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, 
                           p_dot, q_dot, r_dot,
                           delta_x_dot, delta_y_dot, delta_z_dot])
        return x_dot

    def __update_velocity_data(self, state):
        ur, vr, wr = state.vel.flatten()
        
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        self._alpha = np.arctan2(wr,ur)
        # compute sideslip angle
        if (self._Va != 0):
            self._beta = np.arcsin(vr/self._Va)
        else:
            self._beta = 0.0
        
    def __forces_moments(self, state):
        p = self.params
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        # compute gravitaional forces (in body frame)
        f_g = state.R.T @ np.array([[0.], [0.], [p.mass * p.gravity]])
        fx, fy, fz = f_g.flatten()
        delta_x, delta_y, delta_z = state.delta.flatten()
    
        # forces and moments on missile
        Q = 0.5 * p.rho * self._Va**2
        fy += p.c_y_beta * Q * p.S * self._beta 
        fz += p.c_z_alpha * Q * p.S * self._alpha 
        
        Mx = Q * p.S * p.L * (\
                p.m_x_alpha * self._alpha \
              + p.m_x_beta * self._beta \
              + p.m_x_delta_x * delta_x)
        My = Q * p.S * p.L * (\
                p.m_y_alpha * self._alpha \
              + p.m_y_delta_y * delta_y) 
        Mz = Q * p.S * p.L * (\
                p.m_z_beta * self._beta \
              + p.m_z_delta_z * delta_z) 
        
        return np.array([fx, fy, fz, Mx, My, Mz])

    def update_true_state(self, state):
        state.pos = np.array([self._state[0:3]]).T
        state.vel = np.array([self._state[3:6]]).T
        state.R = quaternion_to_rotation(self._state[6:10])
        state.omega = np.array([self._state[10:13]]).T
        state.delta = np.array([self._state[13:16]]).T
        
        return state
            
class StationaryTarget(Model):
    # Model dynamics for targets (kinematic - 3DOF)
    def __init__(self):
        params = hsv_6DOF_parameters
        super().__init__(params)
        
    def init_state(self, init_state):
        return super().init_state(init_state, msg_state.MsgState_Stationary)
    
    def update(self, state):
        return state
    
    def derivatives(self):
        return np.array([0, 0, 0])
    
    def update_true_state(self, state):
        return state






# class Target1(Model):
#     def __init__(self):
#         params = hsv_6DOF_parameters
#         super().__init__(params)
        
#         self.autopilot = ap.AP_6DOF(params)
        
#     def init_state(self, init_state):
#         state = super().init_state(init_state, msg_state.MsgState_6DOF)
#         self.__update_velocity_data(state)
#         return state
    
#     def update(self, state):
#         forces_moments = self.__forces_moments(state)
        
#         self.RK4([forces_moments, self.autopilot.delta_c])

#         # normalize the quaternion
#         e0, e1, e2, e3 = self._state[6:10]
#         normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
#         self._state[6:10] = np.array([e0, e1, e2, e3])/normE

#         # update the message class for the true state
#         new_state = self.update_true_state(state)
        
#         # update the airspeed, angle of attack, and side slip angles using new state
#         self.__update_velocity_data(new_state)
        
#         return new_state
#     def get_airspeed(self):
#         return self._Va
        
#     def derivatives(self, _state, forces_moments, delta_c):
#         DOF6 = self.params
#         """
#         for the dynamics xdot = f(x, u), returns f(x, u)
#         """
#         # extract the states
#         u, v, w = np.array([1000,1000,0])
#         e0, e1, e2, e3 = _state[6:10]
       

#         # position kinematics
#         R = quaternion_to_rotation([e0, e1, e2, e3]) # body to world frame
#         pos_dot = R @ np.array([[u], [v], [w]])
#         north_dot, east_dot, down_dot = pos_dot.flatten()

#         # position dynamics
#         u_dot = 0
#         v_dot = 0
#         w_dot = 0

#         # rotational kinematics
#         e0_dot =0
#         e1_dot = 0
#         e2_dot = 0
#         e3_dot = 0

#         # rotatonal dynamics
#         p_dot = 0
#         q_dot = 0
#         r_dot = 0

#         # rudder dynamics
#         delta_x_dot = 0
#         delta_y_dot = 0
#         delta_z_dot = 0

#         # collect the derivative of the states
#         x_dot = np.array([north_dot, east_dot, down_dot, 
#                            u_dot, v_dot, w_dot,
#                            e0_dot, e1_dot, e2_dot, e3_dot, 
#                            p_dot, q_dot, r_dot,
#                            delta_x_dot, delta_y_dot, delta_z_dot])
#         return x_dot

#     def __update_velocity_data(self, state):
#         ur, vr, wr = state.vel.flatten()
        
#         # compute airspeed
#         self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
#         # compute angle of attack
#         self._alpha = np.arctan2(wr,ur)
#         # compute sideslip angle
#         if (self._Va != 0):
#             self._beta = np.arcsin(vr/self._Va)
#         else:
#             self._beta = 0.0
        
#     def __forces_moments(self, state):
#         p = self.params
#         """
#         return the forces on the UAV based on the state, wind, and control surfaces
#         :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
#         """
#         # compute gravitaional forces (in body frame)
#         f_g = state.R.T @ np.array([[0.], [0.], [p.mass * p.gravity]])
#         fx, fy, fz = f_g.flatten()
#         delta_x, delta_y, delta_z = state.delta.flatten()
    
#         # forces and moments on missile
#         Q = 0.5 * p.rho * self._Va**2
#         fy += p.c_y_beta * Q * p.S * self._beta 
#         fz += p.c_z_alpha * Q * p.S * self._alpha 
        
#         Mx = Q * p.S * p.L * (\
#                 p.m_x_alpha * self._alpha \
#               + p.m_x_beta * self._beta \
#               + p.m_x_delta_x * delta_x)
#         My = Q * p.S * p.L * (\
#                 p.m_y_alpha * self._alpha \
#               + p.m_y_delta_y * delta_y) 
#         Mz = Q * p.S * p.L * (\
#                 p.m_z_beta * self._beta \
#               + p.m_z_delta_z * delta_z) 
        
#         return np.array([0,0,0,0,0,0])

#     def update_true_state(self, state):
#         state.pos = np.array([self._state[0:3]]).T
#         state.vel = np.array([self._state[3:6]]).T
#         state.R = quaternion_to_rotation(self._state[6:10])
#         state.omega = np.array([self._state[10:13]]).T
#         state.delta = np.array([self._state[13:16]]).T
        
        return state
# class Target1(Model):
#     # Model dynamics for targets (kinematic - 3DOF)
#     def __init__(self):
#         params = stationary_target_parameters # put params file here
#         super().__init__(params)
        
#     def init_state(self, init_state):
#         return super().init_state(init_state, msg_state.MsgState_6DOF)
    
#     def update(self,_state):
#         xdot = self.derivatives()
#         # Update the state using the derivatives
#         self._state += self.Ts * xdot
    
#     def derivatives(self):
#         return np.array([2000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    
#     def update_true_state(self, state):
#         state.pos = np.array([self._state[0:3]]).T
#         state.vel = np.array([self._state[3:6]]).T
#         state.R = quaternion_to_rotation(self.state[6:10])
#         state.omega = np.array([self._state[10:13]]).T
#         state.delta = np.array([self._state[13:16]]).T
#         return state

