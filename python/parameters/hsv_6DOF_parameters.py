import numpy as np

######################################################################################
                #   Physical Parameters
######################################################################################
mass = 1200.0  # kg
S = 0.43  # m^2
L = 0.69  # m
Jx = 100.0  # kg*m^2
Jy = 5700.0  # kg*m^2
Jz = 5800.0  # kg*m^2
rho = 1.1558  # kg/m^3
c_z_alpha = -57.15 
c_z_beta = 0.081 # not present anywhere else
c_z_delta_y = -5.75 # not present anywhere else
c_y_alpha = 0.091 # not present anywhere else
c_y_beta = -56.32 
c_y_delta_z = 5.6 # not present anywhere else
m_x_alpha = 0.45
m_x_beta = -0.38
m_x_delta_x = 2.13
m_z_beta = 27.30 
m_z_delta_z = -26.6 
m_y_alpha = -28.15 
m_y_delta_y = -27.9 
tau_x = 0.1
tau_y = 0.1
tau_z = 0.1
gravity = 9.81


######################################################################################
                #   Autopilot Parameters
######################################################################################
min_turn_radius = 12000.0

# gain for orbit and line following
k_orbit = 3.0
k_line = 0.001

delta_max = 0.008 # maximum rudder deflection (radians)
sigma = 0.05 # used to calculate dirty derivative of error

roll_kp = 0.004 
roll_ki = 0.001 
roll_kd = 0.002 
roll_aw = 2 # type of anti-windup (1 is based on error and 2 is based on error_dot)
roll_aw_limit = 0.02 # (radians)

yaw_kp = 0.08 

pitch_kp = 0.2 
pitch_ki = 0.2 
pitch_kd = 0.005 
pitch_aw = 1 # type of anti-windup (1 is based on error and 2 is based on error_dot)
pitch_aw_limit = 0.01 # (radians)

pitch_limit = np.pi/2 # (radians)
alt_kp = 0.0006
alt_ki = 0.00001
alt_kd = 0.0006
alt_aw = 1 # type of anti-windup (1 is based on error and 2 is based on error_dot)
alt_aw_limit = 5.0 # (meters)
