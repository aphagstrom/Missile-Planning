import matplotlib.pyplot as plt
import h5py
import numpy as np
import argparse
import os.path
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import http.server
import socketserver
import signal
from tqdm import tqdm
import threading
import sys

class DataPlotter:
    """
    A class that provides available functions to plot data from an HDF5 file.
      
    Attributes:
        data (HDF5 File Object): Contains all of the sim data to parse through.
            
    """
    def __init__(self, HDF5_path):
        """
        Initializes the `data` object given a path to the HDF5 file.
        
        Arguments:
            HDF5_path (str): Path to the HDF5 file.
        """
        self.data = h5py.File(HDF5_path, "r", driver = "core")

    def frame_args(self, duration):
        return {
            "frame": {"duration": duration},
            "mode": "immediate",
            "fromcurrent": True,
            "transition": {"duration": duration, "easing": "linear"},
        }

    def add_buttons_and_sliders(self):
        slider_steps = [
            {"args": [[f.name], self.frame_args(0)],
             "label": str(self.slider_labels_transformed[k]),
             "method": "animate",
             } for k, f in enumerate(self.fig.frames)
        ]

        sliders = [{
            "pad": {"b": 10, "t": 60},
            "len": 0.9,
            "x": 0.1,
            "y": 0,
            "steps": slider_steps
        }]

        self.fig.update_layout(
            updatemenus=[{
                "buttons": [
                    {
                        "args": [None, self.frame_args(0)],
                        "label": "Play",
                        "method": "animate",
                    },
                    {
                        "args": [[None], self.frame_args(0)],
                        "label": "Pause",
                        "method": "animate",
                    }
                ],
                "direction": "left",
                "pad": {"r": 10, "t": 70},
                "type": "buttons",
                "x": 0.1,
                "y": 0,
            }],
            sliders=sliders,
            yaxis2=dict(
                title='Total Rewards',
                overlaying='y',
                side='right'
            )
        )

    def plot_html(self,
                  filepath="animation.html",
                  plot_every='auto',
                  marker_size=3,
                  line_width=2,
                  edge_pad=2500,
                  plot_scale=2,
                  show_plot=True,
                  total_rewards=None):
        """
        Plots the trajectories to an interactive html file.
        
        Arguments:
            filepath (str): The filepath to save the interactive html file.
            plot_every (int): How many frames to skip when plotting. Note that plot_every = 1 will be too large of a file.
            marker_size (int): The size of the object markers.
            line_width (int): The width of the trajectories of the objects.
            edge_pad (int): How much to pad around the edges of the trajectories when viewing the plot.
            plot_scale (int): How much to scale the size of the plot.
            show_plot (bool): Whether or not to run an html server to show the plot after it is made. 
            total_rewards (dict): A dictionary showing the total rewards for each agent for the plot. For example, 
                total_rewards[agent][timestep] should give the total reward for the agent at a specific timestep.
            
        
        """

        symbol_map = {
            "blue_station" : "x",
            "blue_battery_station" : "square",
            "blue_vehicle" : "circle",
            "red_vehicle" : "circle",
            "red_station" : "x",
            "red_battery_station": "square"
        }

        view_dict = {
            "max_x": -np.inf,
            "min_x": np.inf,
            "max_y": -np.inf,
            "min_y": np.inf,
            "max_z": -np.inf,
            "min_z": np.inf,
        }

        self.x_traj, self.y_traj, self.z_traj = [], [], []
        self.x_pos, self.y_pos, self.z_pos = [], [], []
        self.times, self.colors, self.symbols = [], [], []
        self.id2idx = dict()
        self.obj_types = []
        idx_num = 0
        for key in self.data.keys():
            agent_type = key
            for subkey in self.data[key]:
                y_traj = np.array(self.data[key][subkey]['Pos North'])
                x_traj = np.array(self.data[key][subkey]['Pos East'])
                z_traj = np.array(self.data[key][subkey]['Altitude'])

                traj_bounds = {"max_y": y_traj.max(),
                               "min_y": y_traj.min(),
                               "max_x": x_traj.max(),
                               "min_x": x_traj.min(),
                               "max_z": z_traj.max(),
                               "min_z": z_traj.min()}

                # Set the view of the plot based on the trajectories
                for view_key in view_dict:
                    if "max" in view_key:
                        if traj_bounds[view_key] > view_dict[view_key]:
                            view_dict[view_key] = traj_bounds[view_key] + edge_pad
                    if "min" in view_key:
                        if traj_bounds[view_key] < view_dict[view_key]:
                            view_dict[view_key] = traj_bounds[view_key] - edge_pad

                self.y_traj.append(np.array(self.data[key][subkey]['Pos North']))
                self.x_traj.append(np.array(self.data[key][subkey]['Pos East']))
                self.z_traj.append(np.array(self.data[key][subkey]['Altitude']))
                self.times.append(np.array(self.data[key][subkey]['Time']))
                self.colors.append("red" if "red" in key else "blue")
                self.symbols.append(symbol_map[key])
                self.id2idx[subkey] = idx_num
                self.obj_types.append(agent_type)
                idx_num += 1

        # Get the time frame for the entire simulation
        max_time_len, min_time_len, max_idx, min_idx = 0, np.inf, 0, 0
        for i, time_array in enumerate(self.times):
            if time_array[-1] > max_time_len:
                max_idx = i
                max_time_len = time_array[-1]
            if time_array[0] < min_time_len:
                min_idx = i
                min_time_len = time_array[0]

        # This should record all the objects in the simulation
        self.slider_labels = np.union1d(self.times[min_idx], self.times[max_idx])

        len_slider_labels = len(self.slider_labels)

        if plot_every == 'auto':
            for plot_every_num in range(25,501,25):
                # We pick the max number of frames to be 200 so the file won't be too large
                if len_slider_labels/plot_every_num > 250:
                    continue
                else:
                    plot_every = plot_every_num
                    break
            
            print(f"plotting every {plot_every} frames")


        # Create the initial arrays that we will edit over time
        for i, _ in enumerate(self.x_traj):
            if self.times[i][0] == 0.0:
                self.x_pos.append(self.x_traj[i][0])
                self.y_pos.append(self.y_traj[i][0])
                self.z_pos.append(self.z_traj[i][0])
            else:
                self.x_pos.append(None)
                self.y_pos.append(None)
                self.z_pos.append(None)

        indices = [i for i, _ in enumerate(self.x_traj) if self.times[i][0] == 0.0]

        # Create the intitial figure
        self.fig = go.Figure()
        initial_3d_trace = go.Scatter3d(
            x=[self.x_pos[i] for i in indices],
            y=[self.y_pos[i] for i in indices],
            z=[self.z_pos[i] for i in indices],
            mode='markers',
            marker=dict(color=[self.colors[i] for i in indices], size=marker_size, symbol=[self.symbols[i] for i in indices]) 
        )
        self.fig.add_trace(initial_3d_trace) 

        for color in self.colors:
            self.fig.add_trace(go.Scatter3d(
                x=[], y=[], z=[],
                mode='lines',
                line=dict(color=color, width=line_width),
                showlegend=False
            ))

        x_size = view_dict["max_x"]-view_dict["min_x"]
        y_size = view_dict["max_y"]-view_dict["min_y"]
        z_size = view_dict["max_z"]-view_dict["min_z"]

        max_size = max([x_size, y_size, z_size])

        if total_rewards is not None:
            start_timestep = 0.
            initial_rewards_text = "<br>".join([f"Total Rewards Agent {agent}: {total_rewards[agent][start_timestep]:.2f}" for agent in agents])
        else:
            initial_rewards_text = ''

        self.fig.update_layout(scene=dict(xaxis=dict(range=[view_dict["max_x"], view_dict["min_x"]]),
                                yaxis=dict(range=[view_dict["min_y"], view_dict["max_y"]]),
                                zaxis=dict(range=[view_dict["min_z"], view_dict["max_z"]]),
                                aspectmode="manual",
                                aspectratio=dict(x=plot_scale*x_size/max_size,
                                                y=plot_scale*y_size/max_size,
                                                z=plot_scale*z_size/max_size)))
        self.fig.update_layout(
            annotations=[
                dict(
                    x=0.1,
                    y=1.1,
                    xref="paper",
                    yref="paper",
                    text=initial_rewards_text,
                    showarrow=False,
                    align="left"
                )
            ]
        )

        # Now update the frame data
        self.frames = []
        slider_subset = self.slider_labels[::plot_every]

        if slider_subset[-1] != self.slider_labels[-1]:
            # Append on the final datapoint
            slider_subset = np.concatenate([self.slider_labels[::plot_every], [self.slider_labels[-1]]])

        self.slider_labels_transformed = np.round(slider_subset, 2)

        self.x_traj_subset = [[] for i, _ in enumerate(self.times)]
        self.y_traj_subset = [[] for i, _ in enumerate(self.times)]
        self.z_traj_subset = [[] for i, _ in enumerate(self.times)]
        
        show_obj_dict = {obj_idx: False for obj_idx in range(len(self.times))}
        obj_idx_to_t_idx = {obj_idx: 0 for obj_idx in range(len(self.times))}
        for t_idx in tqdm(range(len(slider_subset))):

            slider_t = slider_subset[t_idx]

            # Initialize the frame_data for this frame
            frame_data = []
            
            for i, time_array in enumerate(self.times):
                if not show_obj_dict[i]:
                    # Check to see if the new object shows up
                    if time_array[0] <= slider_t and time_array[-1] >= slider_t:
                        # We know that the object should be included
                        for obj_t_idx, obj_t in enumerate(time_array):
                            if np.abs(obj_t - slider_t) < 1e-6:
                               obj_idx_to_t_idx[i] = obj_t_idx
                               show_obj_dict[i] = True
                               break
                    else:
                        self.x_pos[i] = None
                        self.y_pos[i] = None
                        self.z_pos[i] = None

                if show_obj_dict[i]:
                    obj_t_idx = obj_idx_to_t_idx[i]
                    self.x_pos[i] = self.x_traj[i][obj_t_idx]
                    self.y_pos[i] = self.y_traj[i][obj_t_idx]
                    self.z_pos[i] = self.z_traj[i][obj_t_idx]
                    self.x_traj_subset[i].append(self.x_traj[i][obj_t_idx])
                    self.y_traj_subset[i].append(self.y_traj[i][obj_t_idx])
                    self.z_traj_subset[i].append(self.z_traj[i][obj_t_idx])

                frame_data.append(go.Scatter3d(
                    x=self.x_traj_subset[i],
                    y=self.y_traj_subset[i],
                    z=self.z_traj_subset[i],
                    mode='lines',
                    line=dict(color=self.colors[i], width=line_width),
                ))

                # Update which objects are shown based on the situation
                if show_obj_dict[i]:
                    # Check to see if the object has dissapeared
                    if obj_t_idx + plot_every >= len(self.x_traj[i]):
                        obj_idx_to_t_idx[i] = len(self.x_traj[i])
                        show_obj_dict[i] = False
                    else:
                        obj_t_idx += plot_every
                        obj_idx_to_t_idx[i] = obj_t_idx

            # Add the points
            frame_data.append(
                go.Scatter3d(
                    x=self.x_pos, y=self.y_pos, z=self.z_pos,
                    mode='markers',
                    marker=dict(color=self.colors, size=marker_size, symbol=self.symbols)
                )
            )

            if total_rewards is not None:
                rewards_text = "<br>".join([f"Total Rewards Agent {agent}: {self.total_rewards[agent][slider_t]:.2f}" for agent in self.agents])
            else:
                rewards_text = ''

            frame_layout = go.Layout(
                annotations=[
                    dict(
                        x=0.1,
                        y=1.1,
                        xref="paper",
                        yref="paper",
                        text=rewards_text,
                        showarrow=False,
                        align="left"
                    )
                ]
            )

            curr_time = np.round(slider_t,2)
                
            self.frames.append(go.Frame(data=frame_data,
                                        name=f'{curr_time}',
                                        layout=frame_layout))

        print("Updating the figure...")

        self.fig.update(frames=self.frames)

        # Add play and pause buttons with slider
        self.add_buttons_and_sliders()

        print("Writing to html file...")

        self.fig.write_html(filepath)

        if show_plot:
            PORT = 8000

            # Create a simple handler to serve files from the current directory
            Handler = http.server.SimpleHTTPRequestHandler

            # Create the server
            httpd = socketserver.TCPServer(("", PORT), Handler)

            def signal_handler(sig, frame):
                print('\nShutting down the server...')
                httpd.shutdown()
                httpd.server_close()
                sys.exit(0)

            # Register the signal handler
            signal.signal(signal.SIGINT, signal_handler)

            def run_server():
                print(f"Serving at port {PORT}")
                print(f"Open your browser and go to http://localhost:{PORT}")
                httpd.serve_forever()

            # Run the server in a separate thread
            server_thread = threading.Thread(target=run_server)
            server_thread.start()

            # You can also run `python -m http.server 8000` from the terminal

        
    def plot_3D(self, block = True):
        """
        Plots the trajectories of all agents in 3D.
        
        Arguments:
            block (bool): If true, code blocks until figure is closed.
        
        """
        ax = plt.figure().add_subplot(projection='3d')
        plot_range= { 
                     "North": [np.inf, -np.inf], 
                     "East":[np.inf, -np.inf],
                     "Altitude": [np.inf, -np.inf]
                    }
        
        blue_vehicles = {}
        red_vehicles = {}
        blue_stations = {}
        red_stations = {}
        blue_battery_stations = {}
        red_battery_stations = {}
        
        if "blue_vehicle" in self.data.keys():
            blue_vehicles = self.data["blue_vehicle"]
        if "red_vehicle" in self.data.keys():
            red_vehicles = self.data["red_vehicle"]
        if "blue_station" in self.data.keys():
            blue_stations = self.data["blue_station"]
        if "red_station" in self.data.keys():
            red_stations = self.data["red_station"]
        if "blue_battery_station" in self.data.keys():
            blue_battery_stations = self.data["blue_battery_station"]
        if "red_battery_station" in self.data.keys():
            red_battery_stations = self.data["red_battery_station"]
            
        self._plot_group_3D_trajectories(ax, blue_vehicles, plot_range, color = 'blue')
        self._plot_group_3D_trajectories(ax, red_vehicles, plot_range, color = 'red')
        self._plot_group_3D_trajectories(ax, blue_battery_stations, plot_range, color = 'blue')
        self._plot_group_3D_trajectories(ax, red_battery_stations, plot_range, color = 'red')
        self._plot_group_3D_trajectories(ax, blue_stations, plot_range, color = 'blue', marker='*')
        self._plot_group_3D_trajectories(ax, red_stations, plot_range, color = 'red', marker='*')
        
        ax.set_box_aspect([plot_range["East"][1] - plot_range["East"][0], plot_range["North"][1] - plot_range["North"][0], plot_range["Altitude"][1] - plot_range["Altitude"][0]])
        ax.set_xlabel("East (m)", labelpad = 20)
        ax.set_ylabel("North (m)", labelpad = 10)
        ax.set_zlabel("Altitude (m)", labelpad = 10)
        ax.set_title("3D Trajectories")
        plt.show(block = block)
    
    def _plot_group_3D_trajectories(self, ax, group, plot_range, color, marker='o'):
        for agent in group.values():
            plot_range["North"] = self._get_plot_range(plot_range["North"], agent["Pos North"])
            plot_range["East"] = self._get_plot_range(plot_range["East"], agent["Pos East"])
            plot_range["Altitude"] = self._get_plot_range(plot_range["Altitude"], agent["Altitude"])
            
            if agent.attrs["dynamics_model"] == "stationary":
                ax.scatter(agent["Pos East"][0], agent["Pos North"][0], agent["Altitude"][0], c = color, marker=marker)
            else:
                ax.plot(np.array(agent["Pos East"]), np.array(agent["Pos North"]), np.array(agent["Altitude"]), color = color)
    
    def plot_2D(self, block = True):
        """
        Plots the trajectories of all agents in 2D (projected onto the North-East plane).
        
        Arguments:
            block (bool): If true, code blocks until figure is closed.
        
        """
        
        plt.figure()
        
        blue_vehicles = {}
        red_vehicles = {}
        blue_stations = {}
        red_stations = {}
        
        if "blue_vehicle" in self.data:
            blue_vehicles = self.data["blue_vehicle"]
        else:
            blue_vehicles = None
            print("blue_vehicle:", blue_vehicles)

        if "red_vehicle" in self.data:
            red_vehicles = self.data["red_vehicle"]
        else:
            red_vehicles = None
            print("red_vehicle:", red_vehicles)

        if "blue_station" in self.data:
            blue_stations = self.data["blue_station"]
        else:
            blue_stations = None
            print("blue_station:", blue_stations)

        if "red_station" in self.data:
            red_stations = self.data["red_station"]
        else:
            red_stations = None
            print("red_station:", red_stations)

        
        self._plot_group_2D_trajectories(blue_vehicles, 'blue')
        self._plot_group_2D_trajectories(red_vehicles, 'red')
        self._plot_group_2D_trajectories(blue_stations, 'blue')
        self._plot_group_2D_trajectories(red_stations, 'red')
        
        plt.xlabel("East (m)")
        plt.ylabel("North (m)")
        plt.title("2D Trajectories (NE-plane)")
        plt.gca().axis('equal')
        plt.show(block = block)
        
    def _plot_group_2D_trajectories(self, group, color):
        for agent in group.values():
            if agent.attrs["dynamics_model"] == 'stationary':
                plt.scatter(agent["Pos East"][0], agent["Pos North"][0], color = color)
            else:
                plt.plot(np.array(agent["Pos East"]), np.array(agent["Pos North"]), color = color) 

    def plot_2D_ED_plane(self, block = True):
        """
        Plots the trajectories of all agents in 2D (projected onto the North-East plane).
        
        Arguments:
            block (bool): If true, code blocks until figure is closed.
        
        """
        
        plt.figure()
        
        blue_vehicles = {}
        red_vehicles = {}
        blue_stations = {}
        red_stations = {}
        
        if "blue_vehicle" in self.data.keys():
            blue_vehicles = self.data["blue_vehicle"]
        if "red_vehicle" in self.data.keys():
            red_vehicles = self.data["red_vehicle"]
        if "blue_station" in self.data.keys():
            blue_stations = self.data["blue_station"]
        if "red_station" in self.data.keys():
            red_stations = self.data["red_station"]
        
        self._plot_group_2D_ED_trajectories(blue_vehicles, 'blue')
        self._plot_group_2D_ED_trajectories(red_vehicles, 'red')
        self._plot_group_2D_ED_trajectories(blue_stations, 'blue')
        self._plot_group_2D_ED_trajectories(red_stations, 'red')
        
        plt.xlabel("East (m)")
        plt.ylabel("Altitude (m)")
        plt.title("2D Trajectories (ED-plane)")
        ylim = plt.gca().get_ylim()
        ylim = (0, ylim[1])
        plt.ylim(ylim)
        plt.gca().set_aspect('equal')
        
        
        plt.show(block = block)

    def _plot_group_2D_ED_trajectories(self, group, color):
        for agent in group.values():
            if agent.attrs["dynamics_model"] == 'stationary':
                plt.scatter(agent["Pos East"][0], agent["Altitude"][0], color = color)
            else:
                plt.plot(np.array(agent["Pos East"]), np.array(agent["Altitude"]), color = color) 

    def _get_plot_range(self, plot_range, list):
        list_min = np.min(list)
        list_max = np.max(list)
        if list_min < plot_range[0]:
            plot_range[0] = list_min
        if list_max > plot_range[1]:
            plot_range[1] = list_max

        # if the range is 0, make it 1
        if plot_range[0] == plot_range[1]:
            plot_range[1] +=1
        return plot_range
            
    def plot_cmds_and_responses(self, block = True, agent_ids = []):
        """
        Plots the history of the autopilot commands and the state responses for each agent specified. Current functionality only generates plots for attributes
        relevant to a 3DOF model.
        
        Arguments:
            block (bool): If true, code blocks until figure is closed.
            agent_ids (array_like): Contains the string IDs of agents to plot. If empty, will plot all agents (not recommended, as it is very slow).

        TODO list:
            - add other subplots for 6DOF model (i.e. deflection angles, etc.).
        
        """
        
        for group in self.data.values():
            for key, agent in group.items():
                id = key[7:] # This assumes that every key is formatted as "Agent: <id>"
                if (not agent_ids) or (id in agent_ids): # if agent_ids were not specified, or they were and the id is in agent_ids
                    self._plot_agent_cmds_and_responses(agent, id) # plot the agent's commands and responses
        
        plt.show(block = block)
                    
    def _plot_agent_cmds_and_responses(self, agent, id):
        fig, axs = plt.subplots(3, 3)
        fig.suptitle("Agent " + id)
        numsteps = len(agent['Time'])
        time = np.array(agent['Time'])
        
        speed_ms = np.zeros(numsteps)
        for i in range(numsteps):
            speed_ms[i] = np.linalg.norm([agent["Vel X"][i], agent["Vel Y"][i], agent["Vel Z"][i]])
        
        axs[0, 0].set_ylabel("Speed (m/s)")
        axs[0, 0].plot(time, speed_ms)
        
        axs[1, 0].set_ylabel("Speed (Mach)")
        axs[1, 0].plot(time, np.array(agent["Speed (Mach)"]))
        
        axs[2, 0].set_ylabel("Thrust (N)")
        axs[2, 0].set_xlabel("Time (s)")
        axs[2, 0].plot(time, np.array(agent["Thrust"]))
        
        if False in np.isnan(agent["Altitude Cmd"]):
            axs[0, 1].set_ylabel("Altitude (m)")
            axs[0, 1].plot(time, np.array(agent["Altitude"]), label = "Response")
            axs[0, 1].plot(time, np.array(agent["Altitude Cmd"]), label = "Command")
            
        elif False in np.isnan(agent["Dynamic Pressure Cmd"]):
            axs[0, 1].set_ylabel("Dynamic Pressure (Pa)")
            axs[0, 1].plot(time, 1/2*np.array(agent["Air Density"])*speed_ms**2, label = "Response")
            axs[0, 1].plot(time, np.array(agent["Dynamic Pressure Cmd"]), label = "Command")
            
        axs[0,1].legend()
        
        axs[1, 1].set_ylabel("Alpha (deg)")
        axs[1, 1].plot(time, np.rad2deg(agent["Angle of Attack"]))
        
        axs[2, 1].set_ylabel("Z Accel Cmd (m/s2)")
        axs[2, 1].set_xlabel("Time (s)")
        axs[2, 1].plot(time, np.array(agent["Accel Z Cmd"]))
        
        axs[0, 2].set_ylabel("Heading (degrees)")
        axs[0, 2].plot(time, np.rad2deg(agent["Attitude Psi"]), label = "Response")
        axs[0, 2].plot(time, np.rad2deg(agent["Course Cmd"]), label = "Command")
        axs[0, 2].legend()
        
        axs[1, 2].set_ylabel("Beta (deg)")
        axs[1, 2].plot(time, np.rad2deg(agent["Side-slip Angle"]))
        
        axs[2, 2].set_ylabel("Y Accel Cmd (m/s2)")
        axs[2, 2].set_xlabel("Time (s)")
        axs[2, 2].plot(time, np.array(agent["Accel Y Cmd"]))
      
    def plot_state(self, block = True, agent_ids = []):
        """
        Plots the state history for each agent specified. 
        
        Arguments:
            block (bool): If true, code blocks until figure is closed.
            agent_ids (array_like): Contains the string IDs of agents to plot. If empty, will plot all agents (not recommended, as it is very slow).

        TODO list:
            - current functionality does not plot angular rates
        """
        
        for group in self.data.values():
            for key, agent in group.items():
                id = key[7:] # This assumes that every key is formatted as "Agent: <id>"
                if (not agent_ids) or (id in agent_ids): # if agent_ids were not specified, or they were and the id is in agent_ids
                    self._plot_agent_state(agent, id) # plot the agent's commands and responses
                    
        plt.show(block = block)
        
    def _plot_agent_state(self, agent, id):
        time = np.array(agent["Time"])
        
        numsteps = len(time)
        
        speed_ms = np.zeros(numsteps)
        accel_mag = np.zeros(numsteps)
        for i in range(numsteps):
            speed_ms[i] = np.linalg.norm([agent["Vel X"][i], agent["Vel Y"][i], agent["Vel Z"][i]])
            accel_mag[i] = np.linalg.norm([agent["Accel X"][i], agent["Accel Y"][i], agent["Accel Z"][i]])
        
        fig, axs = plt.subplots(3, 4)
        fig.suptitle("Agent " + id)
        fig.tight_layout()
        
        axs[0, 0].set_ylabel("Pos N (m)")
        axs[0, 0].plot(time, np.array(agent["Pos North"]))
        
        axs[1, 0].set_ylabel("Pos E (m)")
        axs[1, 0].plot(time, np.array(agent["Pos East"]))
        
        axs[2, 0].set_ylabel("Altitude (m)")
        axs[2, 0].set_xlabel("Time (s)")
        axs[2, 0].plot(time, np.array(agent["Altitude"]))
        
        axs[0, 1].set_ylabel("Vel X (m/s)")
        axs[0, 1].plot(time, np.array(agent["Vel X"]))
        
        axs[1, 1].set_ylabel("Vel Y (m/s)")
        axs[1, 1].plot(time, np.array(agent["Vel Y"]))
        
        axs[2, 1].set_ylabel("Vel Z (m/s)")
        axs[2, 1].set_xlabel("Time (s)")
        axs[2, 1].plot(time, np.array(agent["Vel Z"]))
        
        axs[0, 2].set_ylabel("Accel Magnitude (m/s2)")
        axs[0, 2].plot(time, accel_mag)
        
        axs[1, 2].set_ylabel("Speed (m/s)")
        axs[1, 2].plot(time, speed_ms)
        
        axs[2, 2].set_ylabel("Speed (Mach)")
        axs[2, 2].set_xlabel("Time (s)")
        axs[2, 2].plot(time, np.array(agent["Speed (Mach)"]))
        
        axs[0, 3].set_ylabel("Roll (deg)")
        axs[0, 3].plot(time, np.rad2deg(agent["Attitude Phi"]))
        
        axs[1, 3].set_ylabel("Pitch (deg)")
        axs[1, 3].plot(time, np.rad2deg(agent["Attitude Theta"]))
        
        axs[2, 3].set_ylabel("Yaw (deg)")
        axs[2, 3].plot(time, np.rad2deg(agent["Attitude Psi"]))
        

if __name__ == "__main__":
    # Some random testing/debugging code
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help = "The name of the file stored in hypersonics/saved_data/")
    parser.add_argument("-3D", "--plot-3D", action="store_true", help = "Plot the 3D trajectories of all agents")
    parser.add_argument("-2D", "--plot-2D", action="store_true", help = "Plot the 2D trajectories projected on the North-East plane of all agents")
    parser.add_argument("-2D-ED", "--plot-2D-ED", action = "store_true", help = "Plot the 2D trajectories projected on the North-Down plane of all agents")
    parser.add_argument("-cmds", "--plot-cmds", action="store_true", help = "Plot the autopilot commands and responses for agents. Use -s argument to select which agents to plot. Default plots all of them.")
    parser.add_argument("-state", "--plot-state", action="store_true", help = "Plot the state history of agents.  Use -s argument to select which agents to plot. Default plots all of them. ")
    parser.add_argument("-s", "--select-agents", nargs="+", metavar = "<agent_id>", help="Select which agents to plot for '--plot-cmds' and '--plot-state'")
    parser.add_argument("-f", "--full-path", action = "store_true", help = "Indicates that the string given for 'file_name' is a full path to a directory (use to specify a file in a directory other than 'hypersonics/saved_data/')")
    parser.add_argument("-html", "--plot-html", action="store_true", help = "Create an interactive html plot of the data")


    args = parser.parse_args()

    if args.full_path:
        filepath = args.file_name
    else:
        filepath = 'hypersonics/saved_data/' + args.file_name

    if not os.path.isfile(filepath):
        print("File not found. Please input a valid file location.")
        print("File path given:", filepath)
        exit(1)
    if not filepath.endswith(".hf"):
        print("Indicated file is not of type .hf")
        print("File path given:", filepath)
        exit(1)

    dp = DataPlotter(filepath)

    print("Plotting data from:", filepath)

    if not (args.plot_3D or args.plot_2D or args.plot_cmds or args.plot_state or args.plot_2D_ED or args.plot_html):
        print("Please specify at least one plotting option ('-3D', '-2D', '-2D-ED', '-cmds', '--html' and/or '-state')")
        parser.print_help()
        exit(1)

    if args.plot_html:
        print("Generating interactive html file")
        dp.plot_html()
    if args.plot_3D:
        print("Generating 3D plot")
        dp.plot_3D(block = False)
    if args.plot_2D:
        print("Generating 2D plot of NE plane")
        dp.plot_2D(block = False)
    if args.plot_2D_ED:
        print("Generating 2D plot of ED plane")
        dp.plot_2D_ED_plane(block = False)
    if args.plot_cmds:
        if args.select_agents is None:
            print("Generating plots of autopilot commands for all agents")
        else:
            print("Generating plots of autopilot commands for agents:", args.select_agents)
        dp.plot_cmds_and_responses(block = False, agent_ids = args.select_agents)
    if args.plot_state:
        if args.select_agents is None:
            print("Generating plots of state history for all agents")
        else:
            print("Generating plots of state history for agents:", args.select_agents)
        dp.plot_state(block = False, agent_ids = args.select_agents)

    input("Press enter to close plots")