"""
hsv_sim python
    - Shumway, 2023
    - Last Update:
        10/23/2023 - LDS
"""

import os
import numpy as np

class DataWriter:
    def __init__(self):
        self.path = "python/charts/text/"
        self.files = [''.join([self.path, "data_", str(i), ".txt"]) for i in range(HSV.num_missiles)]


    def delete_files_in_directory(self):
        try:
            if not os.path.isdir(self.path):
                os.mkdir(self.path)
            files = os.listdir(self.path)
            for file in files:
                file_path = os.path.join(self.path, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
        except OSError:
            print("Error occurred while deleting text files.")

    def write_initial_conditions(self, hsv):
        self.delete_files_in_directory()
        self.f = [None]*HSV.num_missiles
        
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
        for i in range(HSV.num_missiles):
            self.f[i] = open(self.files[i], 'w+')
        for i in range(HSV.num_missiles):
            self.f[i].writelines(["initial_north: ", str(hsv[i].state.pos[0, 0]), "\n"])
            self.f[i].writelines(["initial_east: ", str(hsv[i].state.pos[1, 0]), "\n"])
            self.f[i].writelines(["initial_down: ", str(hsv[i].state.pos[2, 0]), "\n"])
            self.f[i].writelines(["initial_heading: ", str(hsv[i].state.get_euler()[2]), "\n\n"])
            self.f[i].writelines(["initial_target_north: ", str(hsv[i].target.state.pos[0, 0]), "\n"])
            self.f[i].writelines(["initial_target_east: ", str(hsv[i].target.state.pos[1, 0]), "\n"])
            self.f[i].writelines(["initial_target_north: ", str(hsv[i].target.state.pos[2, 0]), "\n\n"])
            self.f[i].writelines(["initial_terminal_circle_radius: ", str(ap.terminal_radius), "\n\n"])
            self.f[i].close()
     
    def write_state(self, sim_time, autopilot_commands, hsv):
        i = hsv.missile_id
        attitude = hsv.state.get_euler()
        self.f[i] = open(self.files[i], 'a')
    
        self.f[i].writelines(["time: ", str(sim_time), "\n"])
        self.f[i].writelines(["roll: ", str(attitude[0]*180/np.pi), "\n"])
        self.f[i].writelines(["roll_r: ", str(autopilot_commands.phi_command*180/np.pi), "\n"])
        self.f[i].writelines(["pitch: ", str(attitude[1]*180/np.pi), "\n"])
        self.f[i].writelines(["pitch_r: ", str(autopilot_commands.theta_command*180/np.pi), "\n"])
        self.f[i].writelines(["yaw: ", str(attitude[2]*180/np.pi), "\n"])
        self.f[i].writelines(["yaw_r: ", str(autopilot_commands.course_command*180/np.pi), "\n"])
        self.f[i].writelines(["Position:, ", str(hsv._state[0, 0]), ",", str(hsv._state[1, 0]), ",", str(hsv._state[2, 0]), "\n"])
        self.f[i].writelines(["Velocity: ", str(np.linalg.norm(hsv.state.vel)), "\n\n"])
            
        self.f[i].close()
        
    def write_dubins_circles(self, i, dubins_path):
        self.f = [None]*HSV.num_missiles
        self.f[i] = open(self.files[i], 'a')
        radius_s = 0
        radius_e = 0
        if not np.isclose(dubins_path.L_c1, 0):
            radius_s = dubins_path.radius
        if not np.isclose(dubins_path.L_c2, 0):
            radius_e = dubins_path.radius
        self.f[i].writelines(["Circles:, ", str(dubins_path.center_s[1, 0]), ",", \
                                            str(dubins_path.center_s[0, 0]), ",", \
                                            str(radius_s), ",", \
                                            str(dubins_path.center_e[1, 0]), ",", \
                                            str(dubins_path.center_e[0, 0]), ",", \
                                            str(radius_e), '\n\n'])
        self.f[i].close()
        
    def write_impact(self, i, impact_time, impact_position, elev_angle, az_angle, speed):
        self.f[i] = open(self.files[i], 'a')
        self.f[i].writelines(["Impact_t:, ", str(impact_time), ", 0, 0\n"])
        self.f[i].writelines(["Impact_p:, ", str(impact_position[0]), ",", str(impact_position[1]), ",", str(impact_position[2]), "\n"])
        self.f[i].writelines(["Impact_error (m):, ", str(np.sqrt(impact_position[0]**2 + impact_position[1]**2)), ", 0, 0\n"]) 
        self.f[i].writelines(["Impact_elevation_angle (deg):, ", str(elev_angle), ", 0, 0\n"]) 
        self.f[i].writelines(["Impact_azimuth_angle (deg):, ", str(az_angle), ", 0, 0\n"]) 
        self.f[i].writelines(["Impact_speed (m/s):, ", str(speed), ", 0, 0\n\n"]) 
        self.f[i].close()
        
        print("Missile ", i, ":", sep='')
        print("Impact time: ", impact_time, sep='')
        print("Impact position: ", impact_position, sep='')
        print("Error: ", np.sqrt(impact_position[0]**2 + impact_position[1]**2), sep='')
        print("Elevation Angle: ", elev_angle, " deg", sep='')      
        print("Azimuth Angle: ", az_angle, " deg", sep='')   
        print("Missile Speed (m/s): ", speed, "\n", sep='')
        