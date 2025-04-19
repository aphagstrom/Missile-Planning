import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1])) 

import matplotlib.pyplot as plt
import numpy as np
import subprocess
import parameters.simulation_parameters as SIM

def plot_data(num_missiles, plot_impact_data = True, plot_circles = True): 
    for i in range(num_missiles):
        subprocess.call(('./python/charts/parse_data.sh', str(i)))
    
    impact_data = [None]*num_missiles
    time = [None]*num_missiles
    yaw_r = [None]*num_missiles
    yaw = [None]*num_missiles
    position = [None]*num_missiles
    pitch_r = [None]*num_missiles
    pitch = [None]*num_missiles
    roll_r = [None]*num_missiles
    roll = [None]*num_missiles
    velocity = [None]*num_missiles
    circles = [None]*num_missiles
    impact_position = [None]*num_missiles
    impact_time = [None]*num_missiles
    initial_conditions = [None]*num_missiles
    
    
    # get data from text files
    for i in range(num_missiles):
        if plot_impact_data:
            impact_data[i] = np.loadtxt(''.join(['python/charts/text/impact_data_', str(i), '.txt']), usecols=(1, 2, 3), delimiter = ",")
            impact_time[i] = impact_data[i][0, 0]
            impact_position[i] = impact_data[i][1]
            impact_error = impact_data[i][0, 0]
        time[i] = np.loadtxt(''.join(['python/charts/text/time_', str(i), '.txt']), usecols=(1))
        yaw_r[i] = np.loadtxt(''.join(['python/charts/text/yaw_r_', str(i), '.txt']), usecols=(1))
        yaw[i] = np.loadtxt(''.join(['python/charts/text/yaw_', str(i), '.txt']), usecols=(1))
        position[i] = np.loadtxt(''.join(['python/charts/text/position_', str(i), '.txt']), usecols=(1, 2, 3), delimiter = ',')
        pitch_r[i] = np.loadtxt(''.join(['python/charts/text/pitch_r_', str(i), '.txt']), usecols=(1))
        pitch[i] = np.loadtxt(''.join(['python/charts/text/pitch_', str(i), '.txt']), usecols=(1))
        roll_r[i] = np.loadtxt(''.join(['python/charts/text/roll_r_', str(i), '.txt']), usecols=(1))
        roll[i] = np.loadtxt(''.join(['python/charts/text/roll_', str(i), '.txt']), usecols=(1))
        velocity[i] = np.loadtxt(''.join(['python/charts/text/velocity_', str(i), '.txt']), usecols=(1))
        initial_conditions[i] = np.loadtxt(''.join(['python/charts/text/initial_conditions_', str(i), '.txt']), usecols=(1))
        
        if plot_circles:
            circles[i] = np.loadtxt(''.join(['python/charts/text/circles_', str(i), '.txt']), usecols=(1, 2, 3, 4, 5, 6), delimiter = ",")
        
        # set up subplots
        if SIM.plot_individual_missiles:
        
            north_range = abs(np.max(position[i][:, 0]) - np.min(position[i][:, 0]))
            east_range = abs(np.max(position[i][:, 1]) - np.min(position[i][:, 1]))

            f, axs = plt.subplots(2, 4)
            # gs = axs[0, 0].get_gridspec()
            
            # if north_range > east_range:
            #     for ax in axs[0:, 0]:
            #         ax.remove()
            #     axs[0, 0] = f.add_subplot(gs[0:, 0])
                
            #     north_i, north_j = 0, 1
            #     east_i, east_j = 0, 2
            #     down_i, down_j = 0, 3
            #     roll_i, roll_j, = 1, 1
            #     pitch_i, pitch_j = 1, 2
            #     yaw_i, yaw_j = 1, 3
                
            # else:
            #     for ax in axs[0, 0:2]:
            #         ax.remove()
            #     axs[0, 0] = f.add_subplot(gs[0, 0:2])
                
            #     north_i, north_j = 0, 2
            #     east_i, east_j = 0, 3
            #     down_i, down_j = 1, 3
            #     roll_i, roll_j, = 1, 0
            #     pitch_i, pitch_j = 1, 1
            #     yaw_i, yaw_j = 1, 2
                
            f.tight_layout(h_pad = 1.0, w_pad = 0.0)
            
            axs[0, 0].set_aspect('equal')
            axs[0, 0].plot(position[i][:, 1], position[i][:, 0], label = 'Path')
            axs[0, 0].scatter(0, 0, label = 'Target', color = 'b')
            axs[0, 0].scatter(position[i][0, 1], position[i][0, 0], label = 'Start', color = 'g')
            if plot_impact_data:
                axs[0, 0].scatter(impact_position[i][1], impact_position[i][0], label = 'Impact Position', color = 'r')
            axs[0, 0].legend()
            axs[0, 0].set_title(''.join(["Missile ", str(i), " NE Position"]))
            axs[0, 0].set_xlabel("East (m)")
            axs[0, 0].set_ylabel("North (m)")
            
            if plot_circles:
                if circles[i].ndim == 1:
                    plot_circles = [None]*circles[i].ndim*2
                else:
                    plot_circles = [None]*len(circles[i])*2
                for j in range(int(len(plot_circles)/2)):
                    if circles[i].ndim == 1:
                        if circles[i][2] != 0:
                            plot_circles[2*j] = plt.Circle((circles[i][0], circles[i][1]), circles[i][2], fill = False, color = 'c')
                            axs[0, 0].add_patch(plot_circles[2*j])
                        if circles[i][5] != 0:
                            plot_circles[2*j + 1] = plt.Circle((circles[i][3], circles[i][4]), circles[i][5], fill = False, color = 'c')
                            axs[0, 0].add_patch(plot_circles[2*j + 1])
                    else:
                        if circles[i][j, 2] != 0:
                            plot_circles[2*j] = plt.Circle((circles[i][j, 0], circles[i][j, 1]), circles[i][j, 2], fill = False, color = 'c')
                            axs[0, 0].add_patch(plot_circles[2*j])
                        if circles[i][j, 5] != 0:
                            plot_circles[2*j + 1] = plt.Circle((circles[i][j, 3], circles[i][j, 4]), circles[i][j, 5], fill = False, color = 'c')
                            axs[0, 0].add_patch(plot_circles[2*j + 1])
                
            if len(time[i]) > len(position[i]):
                time[i] = time[i][:len(position[i])]
            
            axs[0, 1].plot(time[i], position[i][:, 0])
            axs[0, 1].set_title("N Position")
            axs[0, 1].set_xlabel("Time (s)")
            axs[0, 1].set_ylabel("North (m)")
            if plot_impact_data:
                axs[0, 1].scatter(impact_time[i], impact_position[i][0], label = 'Impact Position', color = 'r')
            
            axs[0, 2].plot(time[i], position[i][:, 1])
            axs[0, 2].set_title("E Position")
            axs[0, 2].set_xlabel("Time (s)")
            axs[0, 2].set_ylabel("East (m)")
            if plot_impact_data:
                axs[0, 2].scatter(impact_time[i], impact_position[i][1], label = 'Impact Position', color = 'r')
            
            axs[0, 3].plot(time[i], position[i][:, 2])
            axs[0, 3].set_title("Down Position")
            axs[0, 3].set_xlabel("Time (s)")
            axs[0, 3].set_ylabel("Down (m)")
            if plot_impact_data:
                axs[0, 3].scatter(impact_time[i], impact_position[i][2], label = 'Impact Position', color = 'r')
            
            axs[1, 0].set_title("Yaw Performance")
            axs[1, 0].plot(time[i], yaw[i][:len(time[i])], label = 'Yaw', color = 'b')
            axs[1, 0].plot(time[i], yaw_r[i][:len(time[i])], label = 'Yaw Reference', color = 'orange')
            if plot_impact_data:
                axs[1, 0].axvline(x = impact_time[i], color = 'r', label = "Impact Time")
            axs[1, 0].legend()
            axs[1, 0].set_xlabel("Time (s)")
            axs[1, 0].set_ylabel("Yaw Angle (degrees)")
            
            axs[1, 1].set_title("Pitch Performance")
            axs[1, 1].plot(time[i], pitch[i][:len(time[i])], label = 'Pitch', color = 'b')
            axs[1, 1].plot(time[i], pitch_r[i][:len(time[i])], label = 'Pitch Reference', color = 'orange')
            if plot_impact_data:
                axs[1, 1].axvline(x = impact_time[i], color = 'r', label = "Impact Time")
            axs[1, 1].legend()
            axs[1, 1].set_xlabel("Time (s)")
            axs[1, 1].set_ylabel("Pitch Angle (degrees)")
            
            axs[1, 2].set_title("Roll Performance")
            axs[1, 2].plot(time[i], roll[i][:len(time[i])], label = 'Roll', color = 'b')
            axs[1, 2].plot(time[i], roll_r[i][:len(time[i])], label = 'Roll Reference', color = 'orange')
            if plot_impact_data:
                axs[1, 2].axvline(x = impact_time[i], color = 'r', label = "Impact Time")
            axs[1, 2].legend()
            axs[1, 2].set_xlabel("Time (s)")
            axs[1, 2].set_ylabel("Roll Angle (degrees)")
            
            axs[1, 3].set_title("Velocity (m/s)")
            axs[1, 3].plot(time[i][:min(len(velocity[i]), len(time[i]))], velocity[i][:min(len(velocity[i]), len(time[i]))], label = "Velocity")
            if plot_impact_data:
                axs[1, 3].axvline(x = impact_time[i], color = 'r', label = "Impact Time")
                axs[1, 3].legend()
            axs[1, 3].set_xlabel("Time (s)")
            axs[1, 3].set_ylabel("Velocity (m/s)")
            
            figManager = plt.get_current_fig_manager()
            figManager.window.showMaximized()
            
            
            plt.show(block = False)
    
    if plot_impact_data:
        print("Average impact time:", np.average(impact_time))
        print("Difference between impact times:", np.max(impact_time) - np.min(impact_time))
    
    # plot 3D trajectory
    ax = plt.figure().add_subplot(projection='3d')
    xRange = np.max([np.max(position[i][:, 1]) for i in range(num_missiles)]) - np.min([np.min(position[i][:, 1]) for i in range(num_missiles)])
    yRange = np.max([np.max(position[i][:, 0]) for i in range(num_missiles)]) - np.min([np.min(position[i][:, 0]) for i in range(num_missiles)])
    zRange = np.max([np.max(position[i][:, 2]) for i in range(num_missiles)]) - np.min([np.min(position[i][:, 2]) for i in range(num_missiles)])
    ax.set_box_aspect([xRange, yRange, zRange])
    for i in range(num_missiles):
        ax.plot(position[i][:, 1], position[i][:, 0], -position[i][:, 2], label = ''.join(['Missile ', str(i)]))
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude (m)")
    
    ax.scatter(0, 0, 0, label = 'Target', color = 'b')
    ax.scatter(position[i][0, 1], position[i][0, 0], -position[i][0, 2], label = 'Start', color = 'g')
    if plot_impact_data:
        ax.scatter(impact_position[i][1], impact_position[i][0], -impact_position[i][2], label = 'Impact Position', color = 'r')
    ax.legend(loc = "upper left")
    ax.set_title("Missile Trajectory")
    
    # plot combined 2D trajectory
    plt.figure()
    for i in range(num_missiles):
        plt.plot(position[i][:, 1], position[i][:, 0], label = ''.join(['Missile ', str(i)]))
        plt.scatter(position[i][0, 1], position[i][0, 0], color = 'g')
        if plot_impact_data:
            plt.scatter(impact_position[i][1], impact_position[i][0], color = 'r')
            
    terminal_circle = plt.Circle((initial_conditions[0][4], initial_conditions[0][5]), initial_conditions[0][7], fill = False, color = 'c')
    plt.gca().add_patch(terminal_circle)        
    
    plt.gca().set_aspect('equal')
    plt.scatter(0, 0, label = 'Target', color = 'b')
    plt.legend()
    plt.title(''.join(["2D Trajectory"]))
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    
    plt.show()
    
if __name__ == '__main__':
    plot_data()