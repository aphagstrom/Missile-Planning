"""
hsvDynamics 
    - this file implements the dynamic equations of motion for 
      for hypersonic vehicle as outlined in the paper
      Zhibing Li, Xiaoyue Zhang, Huanrui Zhang, Feng Zhang, 
      "Three-dimensional cooperative integrated guidance and 
      control with fixed-impact time and azimuth constraints,"
      submitted to Aerospace Science and Techology, 2023.
    - Update history:  
        5/19/2023 - RWB
"""
import numpy as np
import parameters.hsv_parameters as HSV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
import parameters.sensor_parameters as SENSOR
import random
from tools.rotations import euler_to_quaternion

class HsvDynamics:
    def __init__(self, Ts, missile_id, target = None):
        self._ts_simulation = Ts
        self.target  = target
        self.missile_id = missile_id
        
        if HSV.prev_init_conditions:
            hsv_data = np.loadtxt(''.join(['hypersonic/python/charts/text/data_', str(self.missile_id), '.txt']), usecols = 1, max_rows = 4)
            psi0 = hsv_data[3]
            north0 = hsv_data[0]
            east0 = hsv_data[1]
        elif HSV.rand_init_conditions:
            dist0 = random.uniform(HSV.dist_from_target_lim[0], HSV.dist_from_target_lim[1])
            crs0 = np.deg2rad(random.uniform(HSV.dist_from_target_lim[0], HSV.dist_from_target_lim[1]))
            psi0 = np.deg2rad(random.uniform(HSV.psi0_limit[0], HSV.psi0_limit[1]))  # initial yaw angle
            north0 = dist0*np.cos(crs0)
            east0 = dist0*np.sin(crs0)
        else:
            if len(HSV.psi0) - 1 < self.missile_id:
                print("Not enough psi0 values for number of missiles in hsv_parameters.py")
                exit()
            if len(HSV.north0) - 1 < self.missile_id:
                print("Not enough north0 values for number of missiles in hsv_parameters.py")
                exit()
            if len(HSV.east0) - 1 < self.missile_id:
                print("Not enough east0 values for number of missiles in hsv_parameters.py")    
                exit()
                
            psi0 = HSV.psi0[missile_id]
            north0 = HSV.north0[missile_id]
            east0 = HSV.east0[missile_id]
            
        #   Quaternion State
        e = euler_to_quaternion(HSV.phi0, HSV.theta0, psi0)
        
        self._state = np.array([[north0],  # (0)
                               [east0],   # (1)
                               [HSV.down0],   # (2)
                               [HSV.u0],      # (3)
                               [HSV.v0],      # (4)
                               [HSV.w0],      # (5)
                               [e.item(0)],   # (6)
                               [e.item(1)],   # (7)
                               [e.item(2)],   # (8)
                               [e.item(3)],   # (9)
                               [HSV.p0],     # (10)
                               [HSV.q0],     # (11)
                               [HSV.r0],     # (12)
                               [0.], # delta_x (13)
                               [0.], # delta_y (14)
                               [0.], # delta_z (15)
                               ])
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = HSV.Va0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.state = MsgState()
        # update velocity data and forces and moments
        self.__update_velocity_data()
        self.__forces_moments()
        self.__update_true_state()


    ###################################
    # public functions
    def update(self, delta_c):
        # get forces and moments acting on rigid bod
        forces_moments = self.__forces_moments()

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self.__derivatives(self._state[0:16], forces_moments, delta_c)
        k2 = self.__derivatives(self._state[0:16] + time_step/2.*k1, forces_moments, delta_c)
        k3 = self.__derivatives(self._state[0:16] + time_step/2.*k2, forces_moments, delta_c)
        k4 = self.__derivatives(self._state[0:16] + time_step*k3, forces_moments, delta_c)
        self._state[0:16] += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self.__update_velocity_data()
        # update the message class for the true state
        self.__update_true_state()

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def __derivatives(self, _state, forces_moments, delta_c):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = _state.item(0)
        # east = _state.item(1)
        # down = _state.item(2)
        u = _state.item(3) # body frame velocity along i
        v = _state.item(4) # body frame velocity along j
        w = _state.item(5) # body frame velocity along k
        e0 = _state.item(6) # q_w (real part)
        e1 = _state.item(7) # q_x
        e2 = _state.item(8) # q_y
        e3 = _state.item(9) # q_z
        p = _state.item(10) # roll rate along i in body frame
        q = _state.item(11) # pitch rate along j in body frame
        r = _state.item(12) # yaw rate along k in body frame
        delta_x = _state.item(13) 
        delta_y = _state.item(14) 
        delta_z = _state.item(15)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        Mx = forces_moments.item(3)
        My = forces_moments.item(4)
        Mz = forces_moments.item(5)

        # position kinematics
        R = quaternion_to_rotation(_state[6:10]) # body to world frame
        pos_dot = R @ _state[3:6]
        north_dot = pos_dot.item(0)
        east_dot = pos_dot.item(1)
        down_dot = pos_dot.item(2)

        # position dynamics
        u_dot = r*v - q*w + fx/HSV.mass
        v_dot = p*w - r*u + fy/HSV.mass
        w_dot = q*u - p*v + fz/HSV.mass

        # rotational kinematics
        e0_dot = 0.5 * (-p*e1 - q*e2 - r*e3)
        e1_dot = 0.5 * (p*e0 + r*e2 - q*e3)
        e2_dot = 0.5 * (q*e0 - r*e1 + p*e3)
        e3_dot = 0.5 * (r*e0 + q*e1 -p*e2)

        # rotatonal dynamics
        p_dot = ((HSV.Jy-HSV.Jz)/HSV.Jx)*q*r + Mx/HSV.Jx
        q_dot = ((HSV.Jz-HSV.Jx)/HSV.Jy)*p*r + My/HSV.Jy
        r_dot = ((HSV.Jx-HSV.Jy)/HSV.Jz)*q*p + Mz/HSV.Jz

        # rudder dynamics
        delta_x_dot = (delta_c.x_rudder - delta_x) / HSV.tau_x
        delta_y_dot = (delta_c.y_rudder - delta_y) / HSV.tau_y
        delta_z_dot = (delta_c.z_rudder - delta_z) / HSV.tau_z

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, 
                           u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, 
                           p_dot, q_dot, r_dot,
                           delta_x_dot, delta_y_dot, delta_z_dot]]).T
        return x_dot

    def __update_velocity_data(self):
        v_air = self._state[3:6]
        ur = v_air.item(0)
        vr = v_air.item(1)
        wr = v_air.item(2)
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        self._alpha = np.arctan2(wr,ur)
        # compute sideslip angle
        if (self._Va != 0):
            self._beta = np.arcsin(vr/self._Va)
        else:
            self._beta = 0.0

    def __forces_moments(self):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        # compute gravitaional forces (in body frame)
        R = quaternion_to_rotation(self._state[6:10]) # rotation from body to world frame
        f_g = R.T @ np.array([[0.], [0.], [HSV.mass * HSV.gravity]])
        fx = f_g.item(0)
        fy = f_g.item(1)
        fz = f_g.item(2)
        delta_x = self._state.item(13)
        delta_y = self._state.item(14)
        delta_z = self._state.item(15)
    
        # forces and moments on missile
        Q = 0.5 * HSV.rho * self._Va**2
        fy += HSV.c_y_beta * Q * HSV.S * self._beta 
        fz += HSV.c_z_alpha * Q * HSV.S * self._alpha 
        
        Mx = Q * HSV.S * HSV.L * (\
                HSV.m_x_alpha * self._alpha \
              + HSV.m_x_beta * self._beta \
              + HSV.m_x_delta_x * delta_x)
        My = Q * HSV.S * HSV.L * (\
                HSV.m_y_alpha * self._alpha \
              + HSV.m_y_delta_y * delta_y) 
        Mz = Q * HSV.S * HSV.L * (\
                HSV.m_z_beta * self._beta \
              + HSV.m_z_delta_z * delta_z) 
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def __update_true_state(self):
        self.state.pos = self._state[0:3]
        self.state.vel = self._state[3:6]
        self.state.R = quaternion_to_rotation(self._state[6:10])
        self.state.omega = self._state[10:13]
        self.state.delta = self._state[13:16]
        if self.target != None:
            self.state.LOS = self.target._state[0:3] - self.state.pos
            self.state.LOS /= np.linalg.norm(self.state.LOS)
