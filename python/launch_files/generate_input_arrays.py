import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1])) 

from tools.rotations import euler_to_quaternion
import numpy as np
import random

# number of each agent type
num_ally_vehicles = 3
num_enemy_vehicles = 1
num_targets = 1

# number of each model type for ally vehicles (must add up to num_ally_vehicles)
num_ally_6DOF = 3
# num_ally_modelX = 0

# number of each model type for enemy vehicles (must add up to num_enemy_vehicles)
num_enemy_6DOF = 1
# num_enemy_modelX = 0

# number of each target type (must add up to num_targets)
num_target_stationary = 1
# num_target_typeX = 0

# randomly spawn agents?
allies_rand_init = True
enemies_rand_init = True
targets_rand_init = True

allies_init_seed = [109, 280, 200, -785,-785]
enemies_init_seed = [15, 29]
targets_input_seed = [-3457, 876]

ally_nedp0_limit = np.array([[90000,  100000],
                             [90000,  100000],
                             [-6000,   -6000],
                             [    0, 2*np.pi],
                             [-6000,   -6000]])

allies_nedp0 = np.array([[ 95000,  -96000, -100000,         0],
                         [ 90000,  96000, -100000,     np.pi],
                         [ 85000, 180000, -100000, 3*np.pi/2],
                         [105000,  85000, -100000,   np.pi/2],
                         [135000,  85000, -100000,   np.pi/2]])
    
enemy_nedp0_limit = np.array([[10000,   20000],
                              [-10000,  20000],
                              [-5500,   -6500],
                              [    0, 2*np.pi]])

enemies_nedp0 = np.array([[10000, -12000, -100000,         0],
                          [15000, -17000, -30000,     np.pi],
                          [11000, -15000, -6500, 3*np.pi/2],
                          [19000, -10000, -6200,   np.pi/2]])
    

target_ned0_limit = np.array([[10000,  20000],
                              [-10000,  20000],
                              [     0,      0]])
  
targets_ned0 = np.array([[ 10000,  15000,     0],
                         [-12000, -17000,     0]])

def generate_input_arrays():
    ally_vehicles_input = [None]*num_ally_vehicles
    enemy_vehicles_input = [None]*num_enemy_vehicles
    targets_input = [None]*num_targets
    
    ally_itr = 0
    for i in range(num_ally_6DOF):
        vehicle = init_6DOF_vehicle(allies_init_seed[ally_itr], ally_nedp0_limit, allies_nedp0[ally_itr], '')
        ally_vehicles_input[i] = vehicle
        ally_itr += 1

    enemy_itr = 0
    for i in range(num_enemy_6DOF):
        vehicle = target_init_6DOF_vehicle(enemies_init_seed[enemy_itr], enemy_nedp0_limit, enemies_nedp0[enemy_itr], '')
        enemy_vehicles_input[i] = vehicle
        enemy_itr += 1
        
    target_itr = 0
    for i in range(num_target_stationary):
        target = init_target_stationary(enemies_init_seed[target_itr], enemy_nedp0_limit, enemies_nedp0[target_itr], '')
        targets_input[i] = target
        target_itr += 1


    return ally_vehicles_input, enemy_vehicles_input, targets_input

def init_6DOF_vehicle(seed, nedp0_limit, nedp0, rand_init):
    vehicle = [None]*2
    vehicle[0] = '6DOF'
    
    if rand_init:
        random.seed(seed)
        north0 = random.uniform(nedp0_limit[0, 0], nedp0_limit[0, 1])
        east0 = random.uniform(nedp0_limit[1, 0], nedp0_limit[1, 1])
        down0 = random.uniform(nedp0_limit[2, 0], nedp0_limit[2, 1])
        psi0 = random.uniform(nedp0_limit[3, 0], nedp0_limit[3, 1])
    else:
        north0 = nedp0[0]
        east0 = nedp0[1]
        down0 = nedp0[2]
        psi0 = nedp0[3]
        
    u0 = 2000
    v0 = 0
    w0 = 0 
    theta0 = 0
    phi0 = 0
    p0 = 0
    q0 = 0
    r0 = 0
    delta_x0 = 0
    delta_y0 = 0
    delta_z0 = 0

    e = euler_to_quaternion(phi0, theta0, psi0)
    
    init_conditions = [north0, east0, down0, u0, v0, w0, e.item(0), e.item(1), e.item(2), e.item(3), p0, q0, r0, delta_x0, delta_y0, delta_z0]
    vehicle[1] = init_conditions
    
    return vehicle

def target_init_6DOF_vehicle(seed, nedp0_limit, nedp0, rand_init):
    vehicle = [None]*2
    vehicle[0] = '6DOF'
    
    if rand_init:
        random.seed(seed)
        north0 = random.uniform(nedp0_limit[0, 0], nedp0_limit[0, 1])
        east0 = random.uniform(nedp0_limit[1, 0], nedp0_limit[1, 1])
        down0 = random.uniform(nedp0_limit[2, 0], nedp0_limit[2, 1])
        psi0 = random.uniform(nedp0_limit[3, 0], nedp0_limit[3, 1])
    else:
        north0 = nedp0[0]
        east0 = nedp0[1]
        down0 = nedp0[2]
        psi0 = nedp0[3]
        
    u0 = 1000
    v0 = 0
    w0 = 0 
    theta0 = 0
    phi0 = 0
    p0 = 0
    q0 = 0
    r0 = 0
    delta_x0 = 0
    delta_y0 = 0
    delta_z0 = 0

    e = euler_to_quaternion(phi0, theta0, psi0)
    
    init_conditions = [north0, east0, down0, u0, v0, w0, e.item(0), e.item(1), e.item(2), e.item(3), p0, q0, r0, delta_x0, delta_y0, delta_z0]
    vehicle[1] = init_conditions
    
    return vehicle

def init_target_stationary(seed, ned0_limit, ned0, rand_init):
    target = [None]*2
    target[0] = 'stationary'
    
    if rand_init:
        random.seed(seed)
        north0 = random.uniform(ned0_limit[0, 0], ned0_limit[0, 1])
        east0 = random.uniform(ned0_limit[1, 0], ned0_limit[1, 1])
        down0 = random.uniform(ned0_limit[2, 0], ned0_limit[2, 1])
    else:
        north0 = ned0[0]
        east0 = ned0[1]
        down0 = ned0[2]

    
    init_conditions = [north0, east0, down0]
    target[1] = init_conditions
    
    return target
    
       
if __name__ == '__main__':
    generate_input_arrays()