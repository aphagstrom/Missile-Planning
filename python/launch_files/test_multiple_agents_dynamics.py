import os, sys
import numpy as np
from pathlib import Path
sys.path.insert(0, os.fspath(Path(__file__).parents[1]))
import parameters.simulation_parameters as SIM
from planners.path_manager import PathManager
from planners.path_follower import PathFollower
from viewers.hsv_viewer import WorldViewer
import pyqtgraph as pg
from tools.assign_waypoints import assign_waypoints
from tools.sync_dubins_paths import sync_dubins_paths
from parameters.hsv_6DOF_parameters import min_turn_radius
from launch_files.generate_input_arrays import generate_input_arrays
from launch_files.init_agents import init_agents
from scipy.optimize import minimize
from message_types.msg_waypoints import MsgWaypoints
from scipy.optimize import minimize
import numpy as np

def rotz(theta: float):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])


class DubinsPlanner:
    def __init__(self, allies_input, enemies_input, targets_input, position, term_yaw):
        self.allies_input = allies_input
        self.enemies_input = enemies_input
        self.targets_input = targets_input
        self.position = position
        self.term_yaw = term_yaw
        self.SIM = SIM  
        self.initialize_agents()
        self.previous_enemy_psi = [None] * len(self.enemy_vehicles)
        


    def initialize_agents(self):
        self.ally_vehicles, self.enemy_vehicles, self.targets = init_agents(self.allies_input, self.enemies_input, self.targets_input)
        print("targets are:",self.targets[0].state.pos)
        print("ally_vehicles are;",self.ally_vehicles)

        # Print the type of ally_vehicles
        print("Type of ally_vehicles:", type(self.ally_vehicles))
        
        # Check if ally_vehicles is a list
        if isinstance(self.ally_vehicles, list):
            print("ally_vehicles is a list with length:", len(self.ally_vehicles))
            
            # Iterate over the list and print the type of each element
            for i, vehicle in enumerate(self.ally_vehicles):
                print(f"Type of ally_vehicles[{i}]:", type(vehicle))
        else:
            print("Error: ally_vehicles is not a list")


    def calculate_cost(self, sigma, ally_vehicle, target, term_yaw, position):
        ally_vehicle.path_manager = PathManager(ally_vehicle.dynamics) 
        ally_vehicle.path_follower = PathFollower(ally_vehicle.dynamics) 
        ally_vehicle.set_target(target)  
        # print("target is:",target.state.pos)
        ally_vehicle.path_manager.waypoints = MsgWaypoints()
        ally_vehicle.path_manager.waypoints.type = 'dubins'
        ally_vehicle.path_manager.waypoints.add(ned=ally_vehicle.state.pos,
                                                course=ally_vehicle.state.get_euler()[2])
        pseudo_target_pos = ally_vehicle.target.state.pos + sigma * np.array([[1000, 0, 0]]).T + 16500 * rotz(position) @ np.array([[1000, 0, 0]]).T / np.linalg.norm(np.array([[1000, 0, 0]]))
        ally_vehicle.path_manager.waypoints.add(ned=pseudo_target_pos,
                                                course=term_yaw)
        ally_vehicle.path_manager.update(ally_vehicle, min_turn_radius)
        dubins_path_length = ally_vehicle.path_manager.dubins_path.length
        dubins_time = dubins_path_length / ally_vehicle.get_airspeed()
        cost_j = np.abs(sigma - dubins_time)
        return cost_j
    def setup(self):
        sim.run_optimization()
        sim.assign_waypoints()
    def run_optimization(self):
        optimal_sigmas = []
        initial_sigma = 50
        for j in range(len(self.ally_vehicles)):
            result = minimize(self.calculate_cost, initial_sigma, args=(self.ally_vehicles[j], self.targets[j % len(self.targets)], self.term_yaw[j], self.position[j]),
                              method='Nelder-Mead', options={'maxiter': 1000})
            optimal_sigma_j = result.x[0]
            optimal_sigmas.append(optimal_sigma_j)
        print("Optimal sigmas are:", optimal_sigmas)
        self.max_sigma = max(optimal_sigmas)
        
    def assign_waypoints(self):
        term_radius = 16500
        assign_waypoints(self.ally_vehicles, self.targets, term_radius, self.max_sigma, self.position, self.term_yaw)
    
    def course_has_changed(self, enemy_vehicle, idx, threshold_degrees=np.deg2rad(.001)):
        enemy_psi = enemy_vehicle.calculate_yaw()  
        if enemy_psi is not None:
            previous_psi = self.previous_enemy_psi[idx]
            if previous_psi is None:
                has_changed = True  
            else:
                has_changed = abs(enemy_psi - previous_psi) > threshold_degrees
            if has_changed:
                self.previous_enemy_psi[idx] = enemy_psi 
            return has_changed
        else:
            return False
    def simulate(self):
        sim_time = self.SIM.start_time
        sync_dubins_paths(self.ally_vehicles, self.targets)
        print("ally_vehivles are",self.ally_vehicles)
        print("targets are",self.targets)
        # print("Dubins path 1 is:", self.ally_vehicles[0].path_manager.dubins_path.length / self.ally_vehicles[0].get_airspeed())
        # print("Dubins path 2 is:", self.ally_vehicles[1].path_manager.dubins_path.length / self.ally_vehicles[1].get_airspeed())
        # print("Dubins path 3 is:", self.ally_vehicles[2].path_manager.dubins_path.length / self.ally_vehicles[2].get_airspeed())
        # print("Dubins path 4 is:", self.ally_vehicles[3].path_manager.dubins_path.length / self.ally_vehicles[3].get_airspeed())
        # print("Dubins path 5 is:", self.ally_vehicles[4].path_manager.dubins_path.length / self.ally_vehicles[4].get_airspeed())
        
        print("Dubins path 3 center s is:", self.ally_vehicles[2].path_manager.dubins_path.center_s)
        print("Dubins path 3 center e is:", self.ally_vehicles[2].path_manager.dubins_path.center_e)

        print("Dubins path 3 dir e is:", self.ally_vehicles[2].path_manager.dubins_path.dir_e)

        print("Dubins path 3 r1 is:", self.ally_vehicles[2].path_manager.dubins_path.r1)

        print("Dubins path 3 Ls is:", self.ally_vehicles[2].path_manager.dubins_path.L_s)
        print("Dubins path 3 Lc1 is:", self.ally_vehicles[2].path_manager.dubins_path.L_c1)
        print("Dubins path 3 Lc2 is:", self.ally_vehicles[2].path_manager.dubins_path.L_c2)
        
        
        
        plot_app = pg.QtWidgets.QApplication([])
        world_view = WorldViewer(
            app=plot_app, dt=self.SIM.ts_simulation,
            plot_period=self.SIM.ts_plot_refresh)
        while sim_time < self.max_sigma:
            # for i in range(len(self.enemy_vehicles)):
            #     if self.enemy_vehicles[i].exists:
            #         self.enemy_vehicles[i].update_dynamics()
            for i in range(len(self.enemy_vehicles)):
                if self.enemy_vehicles[i].exists:
                    self.enemy_vehicles[i].update_dynamics()
                   
                    # if self.course_has_changed(self.enemy_vehicles[i], i):
                    #     print(f"Course has changed for enemy vehicle {i}. Rerunning optimization and assigning waypoints.")
                    #     for j in range(len(self.ally_vehicles)):
                    #         self.ally_vehicles[j].set_target(self.enemy_vehicles[i])  
                    #         print("target pos 0 is:",self.ally_vehicles[j].target.state.pos)
                    #     self.run_optimization()
                    #     self.assign_waypoints()
                    #     self.targets=self.ally_vehicles[0].target
                    #     sync_dubins_paths(self.ally_vehicles, self.targets)
                    # print("Dubins path 1 is:", self.ally_vehicles[0].path_manager.dubins_path.length / self.ally_vehicles[0].get_airspeed())
                    # print("Dubins path 2 is:", self.ally_vehicles[1].path_manager.dubins_path.length / self.ally_vehicles[1].get_airspeed())
                    # print("Dubins path 3 is:", self.ally_vehicles[2].path_manager.dubins_path.length / self.ally_vehicles[2].get_airspeed())
                    # print("Dubins path 4 is:", self.ally_vehicles[3].path_manager.dubins_path.length / self.ally_vehicles[3].get_airspeed())
                    # print("Dubins path 5 is:", self.ally_vehicles[4].path_manager.dubins_path.length / self.ally_vehicles[4].get_airspeed())
            for i in range(len(self.ally_vehicles)):
                if self.ally_vehicles[i].exists:
                    self.ally_vehicles[i].update_all()
                    # print("updated dynamics is:", self.ally_vehicles[i].state.pos)
            if self.SIM.show_visualizer:  
                world_view.update(self.ally_vehicles, self.enemy_vehicles, self.targets, self.term_yaw, self.position, sim_time)  
            sim_time += self.SIM.ts_simulation
        if SIM.plot_end:
            input('Press RETURN to show plots\n')


if __name__ == "__main__":
    allies_input, enemies_input, targets_input = generate_input_arrays()
    # position = [np.deg2rad(60), np.deg2rad(30), np.deg2rad(0), np.deg2rad(-30), np.deg2rad(-60)]
    # term_yaw = [np.deg2rad(240), np.deg2rad(210), np.deg2rad(180), np.deg2rad(130), np.deg2rad(120)]
    position = [np.deg2rad(30), np.deg2rad(0), np.deg2rad(-30)]
    term_yaw = [np.deg2rad(210), np.deg2rad(180), np.deg2rad(150)]
    sim = DubinsPlanner(allies_input, enemies_input, targets_input, position, term_yaw)
    sim.setup()
    print("WAYPOINTS 1 are:",sim.ally_vehicles[0].path_manager.waypoints.ned)
    print("WAYPOINTS 2 are:",sim.ally_vehicles[1].path_manager.waypoints.ned)
    print("WAYPOINTS 3 are:",sim.ally_vehicles[2].path_manager.waypoints.ned)
    sim.simulate()

















