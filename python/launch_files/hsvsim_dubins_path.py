"""
    - This file tests the dynamics
    - Update history:   
        7/20/23 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1])) 

import parameters.simulation_parameters as SIM
import parameters.autopilot_parameters as ap
import parameters.hsv_parameters as HSV
from planners.path_manager import PathManager
from planners.path_follower import PathFollower
from controllers.autopilot import Autopilot
from models.hsv_dynamics import HsvDynamics
from models.target_dynamics import TargetDynamics
from viewers.hsv_viewer import WorldViewer
from viewers.data_viewer import DataViewer
import pyqtgraph as pg
import copy
from charts.plot_data import plot_data
from charts.data_writer import DataWriter
from tools.assign_waypoints import assign_waypoints
from tools.calculate_impact import calculate_impact
from tools.sync_dubins_paths import sync_dubins_paths

# initialize the visualization
plot_app = pg.QtWidgets.QApplication([])
if SIM.show_visualizer:
    world_view = WorldViewer(
        app=plot_app, dt=SIM.ts_simulation,
        plot_period=SIM.ts_plot_refresh)

if SIM.plot_data:
    data_view = [DataViewer(
        app=plot_app,dt=SIM.ts_simulation, 
        plot_period=SIM.ts_plot_refresh,
        data_recording_period=SIM.ts_plot_record_data, 
        time_window_length=100,
        window_title = ''.join(["Missile ", str(i)])) 
        for i in range(HSV.num_missiles)]

# initialize writer for saving data to text files
data_writer = DataWriter()

# initialize autonomy stack
target = TargetDynamics(SIM.ts_simulation)
hsv = [HsvDynamics(SIM.ts_simulation, i, target) for i in range(HSV.num_missiles)]
path_manager = [PathManager() for i in range(HSV.num_missiles)]
path_follower = [PathFollower() for i in range(HSV.num_missiles)]
autopilot = [Autopilot(SIM.ts_simulation) for i in range(HSV.num_missiles)]

# save initial position conditions to file
data_writer.write_initial_conditions(hsv)

# assign missiles to waypoints
waypoints, max_path = assign_waypoints(hsv)

# initialize the simulation time
sim_time = SIM.start_time

sync_dubins_paths(path_manager, waypoints, hsv, max_path)
                       
collisions = [False]*HSV.num_missiles
# main simulation loop
print('Press CTRL-C to exit...\n')
while sim_time < SIM.end_time and (False in collisions):
    for i in range(HSV.num_missiles):
        if not collisions[i]:
            prev_state = copy.copy(hsv[i]._state[0:3])
            
            # -------path manager-------------
            path = path_manager[i].update(waypoints[i], ap.min_turn_radius, hsv[i])
            
            # -------path follower-------------
            autopilot_commands = path_follower[i].update(path, hsv[i].state)
        
            # ------- set references ----------------
            delta = autopilot[i].update(autopilot_commands, hsv[i].state)
            
            # -------update physical system-------------
            hsv[i].update(delta)
            
            # -------plt data---------------------------
            if SIM.plot_data:
                data_view[i].update(hsv[i].state, delta, target.state)
                
            # ---write state data to file---
            data_writer.write_state(sim_time, autopilot_commands, hsv[i]) 
            
            # -------end condition-------------------
            if hsv[i]._state[2] > 0: # assumes target is on ground
                collisions[i] = True
                calculate_impact(sim_time, prev_state, hsv[i], data_writer)
    
    # -------update viewer-------------  
    if SIM.show_visualizer:  
        world_view.update(hsv, target.state)  
    target.update()
    # -------increment time------------------
    sim_time += SIM.ts_simulation

if SIM.plot_end:
    input('Press RETURN to show plots\n')
    plot_data(HSV.num_missiles, True)