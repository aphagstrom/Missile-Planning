"""
    - This file displays the hypersonic vehicle
    - Update history:  
        5/15/23 - RWB
        7/14/23 - RWB
"""
from viewers.draw_hsv import DrawHSV
from viewers.draw_path import DrawPath
from viewers.draw_waypoints import DrawWaypoints
from viewers.draw_target import DrawTarget
import pyqtgraph.opengl as gl
from planners.path_manager import PathManager
from models.models import HSV_6DOF, StationaryTarget
import numpy as np


class WorldViewer:
    def __init__(self, app, dt = 0.01,
                 plot_period = 0.2,
                 scale=200000,
                 gridFlag=True): # time interval between a plot update):
        self._dt = dt
        self._time = 0
        self._plot_period = plot_period
        self._plot_delay = 0        
        self.scale = scale
        # initialize Qt gui application and window
        self.app = app  # initialize QT, external so that only one QT process is running
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        self.window.setGeometry(0, 0, 950, 950)  # args: upper_left_x, upper_right_y, width, height
        if gridFlag is True:
            grid = gl.GLGridItem() # make a grid to represent the ground
            grid.scale(self.scale/20, self.scale/20, self.scale/20) # set the size of the grid (distance between each line)
            self.window.addItem(grid) # add grid to viewer
        center = self.window.cameraPosition()
        center.setX(0)
        center.setY(0)
        center.setZ(0)
        self.window.setCameraPosition(pos=center, distance=self.scale, elevation=50, azimuth=-90)
        self.window.setBackgroundColor('k')  # set background color to black
        #self.window.setBackgroundColor('w')  # set background color to white
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False  # has the mav been plotted yet?
        self.hsv_plot = []
        self.target_plot = []
        self.target_circle = None  # Variable to store the circle object
        self.yaw_marker = None  # Initialize the plot item for the yaw marker

    








    def _draw_target_circle(self, ally_vehicle, positions,sigma):
        terminal_radius = 16500
        north_coord = ally_vehicle.target.state.pos[0, 0]+sigma*1000
        east_coord = ally_vehicle.target.state.pos[1, 0]
        down_coord = 100000  
        radius = 16500  
        segments = 100  

        # Remove previous items if they exist
        if hasattr(self, 'target_circles'):
            for circle in self.target_circles:
                self.window.removeItem(circle)
        if hasattr(self, 'yaw_markers'):
            for marker in self.yaw_markers:
                self.window.removeItem(marker)

        self.target_circles = []
        self.yaw_markers = []

        for position in positions:
            # Generate points for the circle
            theta = np.linspace(0, 2 * np.pi, segments)
            x_circle = east_coord + radius * np.cos(theta)
            y_circle = north_coord + radius * np.sin(theta)
            z_circle = down_coord * np.ones_like(theta)

            # Draw the circle
            target_circle = gl.GLLinePlotItem(pos=np.column_stack([x_circle, y_circle, z_circle]), color=(0, 255, 0, 255))
            self.window.addItem(target_circle)
            self.target_circles.append(target_circle)

            # Calculate position for yaw marker
            # pos = ally_vehicle.target.state.pos \
            #     - terminal_radius * np.array([[np.cos(term_yaw)], [np.sin(term_yaw)], [0]]) \
            #     + (np.array([[0], [0], [ally_vehicle.state.pos[2, 0]]])
            pos=ally_vehicle.target.state.pos+sigma*np.array([[1000,0,0]]).T+16500*np.array([[np.cos(position), -np.sin(position), 0],
                    [np.sin(position), np.cos(position), 0],
                    [0, 0, 1]]) @np.array([[1000,0,0]]).T/(np.linalg.norm(np.array([[1000,0,0]]).T))
           
            marker_x = pos.item(1)
            marker_y = pos.item(0)

            # Draw the yaw marker as a blue circle
            yaw_marker = gl.GLScatterPlotItem(pos=np.array([[marker_x], [marker_y], [down_coord]]).T,
                                            color=(0, 0, 255, 255), size=40.0)
            self.window.addItem(yaw_marker)
            self.yaw_markers.append(yaw_marker)







    

    # def _draw_target_circle(self, ally_vehicles,term_yaw):
    #     # Get target position
    
    #     terminal_radius=16500
        

    #     north_coord = ally_vehicles.target.state.pos[0, 0]
    #     east_coord = ally_vehicles.target.state.pos[1, 0]
    #     # down_coord = -target_state.pos[2, 0]
    #     down_coord=100000
    #     # Define the circle parameters
    #     radius = 16500  # Adjust as needed
    #     segments = 100  # Number of segments to approximate the circle
        
    #     # Generate points for the circle
    #     theta = np.linspace(0, 2 * np.pi, segments)
    #     x_circle = east_coord + radius * np.cos(theta)
    #     y_circle = north_coord + radius * np.sin(theta)
    #     z_circle = down_coord * np.ones_like(theta)  # Shift with down coordinate
        
    #     # Draw the circle
    #     if self.target_circle is not None:
    #         self.window.removeItem(self.target_circle)
    #     self.target_circle = gl.GLLinePlotItem(pos=np.column_stack([x_circle, y_circle,z_circle]), color=(0, 255, 0, 255))

    #     self.window.addItem(self.target_circle)

    #     pos=ally_vehicles.target.state.pos \
    #         - terminal_radius*np.array([[np.cos(term_yaw)], [np.sin(term_yaw)], [0]]) \
    #         + np.array([[0], [0], [ally_vehicles.state.pos[2, 0]]])
    #     # print("pos is:",pos)
    #     marker_x = pos.item(1)
    #     marker_y = pos.item(0) 

    #     # Draw the yaw marker as a blue circle
    #     if self.yaw_marker is not None:
    #         self.window.removeItem(self.yaw_marker)
    #     self.yaw_marker = gl.GLScatterPlotItem(pos=np.array([[marker_x], [marker_y], [down_coord]]).T,
    #                                            color=(0, 0, 255, 255), size=10.0)
    #     self.window.addItem(self.yaw_marker)

    def update(self, ally_vehicles, enemy_vehicles, targets,term_yaw,position,sigma):
        # initialize the drawing the first time update() is called
        red = np.array([[1., 0., 0., 1]])
        blue = np.array([[30, 144, 255, 255]])/255.
        if not self.plot_initialized:
            self.allies_plot = [None]*len(ally_vehicles)
            self.allies_path=[None]*len(ally_vehicles)
            self.allies_waypoints=[None]*len(ally_vehicles)
            self.enemies_plot = [None]*len(enemy_vehicles)
            self.targets_plot = [None]*len(targets)
            for i in range(len(ally_vehicles)):
                if type(ally_vehicles[i].dynamics) == HSV_6DOF:
                    self.allies_plot[i] = DrawHSV(ally_vehicles[i].state, self.window)
                    if ally_vehicles[i].path_manager.path.type != 'terminal':
                        self.allies_path[i]=DrawPath(ally_vehicles[i].path_manager.update(ally_vehicles[i]), red,self.window)
                        self._draw_target_circle(ally_vehicles[i],position,sigma)
                        # self.allies_waypoints[i]=DrawWaypoints(ally_vehicles[i],ally_vehicles[i].path_manager.waypoints, ally_vehicles[i].dynamics,ally_vehicles[i].dynamics.params.min_turn_radius,blue,self.window)
            for i in range(len(enemy_vehicles)):
                if type(enemy_vehicles[i].dynamics) == HSV_6DOF:
                    self.enemies_plot[i] = DrawHSV(enemy_vehicles[i].state, self.window)
                # elif type(enemy_vehicles[i].dynamics) == Target1:
                #     self.enemies_plot[i] = DrawHSV(enemy_vehicles[i].state, self.window)
            # for i in range(len(targets)):
                # if type(targets[i].dynamics) == StationaryTarget:
                    # self.targets_plot[i] = DrawTarget(targets[i].state, self.window)
                # elif type(targets[i].dynamics) == HSV_6DOF:
                #     self.targets_plot[i] = DrawHSV(targets[i].state, self.window)
                    
            self.plot_initialized = True
            self.app.processEvents()  # draw
        # else update drawing on all other calls to update()
        else:
            if self._plot_delay >= self._plot_period:
                self._plot_delay = 0
                for i in range(len(ally_vehicles)):
                    self.allies_plot[i].update(ally_vehicles[i].state)
                    self._draw_target_circle(ally_vehicles[i],position,sigma)
                    if ally_vehicles[i].path_manager.path.type != 'terminal':
                        self.allies_path[i].update(ally_vehicles[i].path_manager.update(ally_vehicles[i]),red)
                        
                        # self.allies_waypoints[i].update(ally_vehicles[i],ally_vehicles[i].path_manager.waypoints,ally_vehicles[i].dynamics)
                for i in range(len(enemy_vehicles)):
                    self.enemies_plot[i].update(enemy_vehicles[i].state)
                # for i in range(len(targets)):
                #     self.targets_plot[i].update(targets[i].state)
                
                self.app.processEvents()  # redraw
            self._plot_delay += self._dt
        
    def close(self):
        self.window.close()
