"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
        3/30/2022 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl
from planners.dubins_parameters import DubinsParameters
import parameters.hsv_6DOF_parameters as params1


class DrawWaypoints:
    def __init__(self, hsv,waypoints,model, radius, color, window):
        self.radius = radius
        self.color = color
        self.hsv=hsv
        self.min_turn_radius=params1.min_turn_radius
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            self.dubins_path = DubinsParameters(model)
            points = self.dubins_points(hsv,waypoints, model,self.radius, 0.1)
        waypoint_color = np.tile(color, (points.shape[0], 1))
        self.waypoint_plot_object = gl.GLLinePlotItem(pos=points,
                                                      color=waypoint_color,
                                                      width=2,
                                                      antialias=False,
                                                      mode='lines',
                                                      )
        self.waypoint_plot_object.setGLOptions('translucent')  # puts waypoint behind obstacles
        # ============= options include
        # opaque        Enables depth testing and disables blending
        # translucent   Enables depth testing and blending
        #               Elements must be drawn sorted back-to-front for
        #               translucency to work correctly.
        # additive      Disables depth testing, enables blending.
        #               Colors are added together, so sorting is not required.
        # ============= ======================================================
        window.addItem(self.waypoint_plot_object)

    def update(self, hsv,waypoints,model):
        if waypoints.type=='straight_line' or waypoints.type=='fillet':
            points = self.straight_waypoint_points(waypoints)
        elif waypoints.type=='dubins':
            points = self.dubins_points(hsv,waypoints, model,self.radius, 0.1)
        self.waypoint_plot_object.setData(pos=points)

    def straight_waypoint_points(self, waypoints):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(waypoints.ned)
        return points.T

    def dubins_points(self, hsv,waypoints,model, radius, Del):
        # returns a list of points along the dubins path
        #initialize_points = True
        dubins_path = DubinsParameters(model)
        for j in range(0, waypoints.num_waypoints-1):
            dubins_path.update(hsv,
                waypoints.ned[:, j:j+1],
                waypoints.course.item(j),
                waypoints.ned[:, j+1:j+2],
                waypoints.course.item(j+1),
                radius)
            if j == 0:
                points = dubins_path.compute_points()
            else:
                points = np.concatenate((points, dubins_path.compute_points()), axis=0)
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = points @ R.T
        return points

    def mod(self, x):
        # force x to be between 0 and 2*pi
        while x < 0:
            x += 2*np.pi
        while x > 2*np.pi:
            x -= 2*np.pi
        return x
