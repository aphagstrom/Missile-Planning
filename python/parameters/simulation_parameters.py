ts_simulation = 0.01  # time step for simulation
start_time = 0.  # start time for simulation
end_time = 10000.  # end time for simulation
#ts_video = 0.1  # write rate for video
ts_control = ts_simulation  # sample rate for the controller

ts_plot_refresh = 0.2  # seconds between each plot update
ts_plot_record_data = 0.1 # seconds between each time data is recorded for plots

plot_data = False # live plotting of each missile's position, velocity, rudder angles, etc.
show_visualizer = True # live world visualizer showing missiles on their trajectory to hit the target
plot_dubins = False # live plotting of each dubins path when it is generated
plot_end  = True # plots at the end of the simulation showing trajectories, position, velocity, autopilot control, etc.
plot_individual_missiles = False # have a window per missile at the end of the simulation showing position, velocity, control surfaces, etc.