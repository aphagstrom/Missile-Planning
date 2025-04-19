from queue import PriorityQueue
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from tools.get_dist import get_dist
from tools.angles import bound_angle
from planners.dubins_parameters import DubinsParameters
import matplotlib.pyplot as plt
from itertools import permutations

def assign_waypoints(ally_vehicles, targets, terminal_radius,sigma,position,term_yaw):
    for i in range(len(targets)):
        print("Target ", targets[i].id, ":", sep='')
        hsv = [vehicle for vehicle in ally_vehicles if vehicle.target == targets[i]] # list of vehicles assigned to target
        for j in range(len(hsv)):
            hsv[j].path_manager.waypoints = MsgWaypoints()
            hsv[j].path_manager.waypoints.type = 'dubins'
            hsv[j].path_manager.waypoints.add(ned = hsv[j].state.pos, course = hsv[j].state.get_euler()[2])
            hsv[j].path_manager.waypoints.add(ned = hsv[j].target.state.pos+sigma*np.array([[1000,0,0]]).T+terminal_radius*rotz(position[j])@np.array([[1000,0,0]]).T/(np.linalg.norm(np.array([[1000,0,0]]).T)), course = term_yaw[j])     
    return
def rotz(theta: float):
    '''
    returns rotation matrix for right handed passive rotation about z-axis
    '''
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])