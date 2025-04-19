"""
data_viewer
    - Update history:
        7/13/2023 - RWB - Adapted from David Christensen
"""
import numpy as np
from tools.plotter import Plotter
from tools.angles import wrap
class DataViewer:
    def __init__(self, app,  dt = 0.01,
                 time_window_length = 30, # number of data points plotted at a time
                 plot_period = 0.2, # time interval between a plot update
                 data_recording_period = 0.1,
                 window_title = ""): # time interval between recording a data update
        self._dt = dt
        self._data_window_length = time_window_length/data_recording_period
        self._update_counter = 0
        self._plotter = Plotter(app=app, plots_per_row=3, geometry=[950, 0, 1200, 950])
        self._plot_period = plot_period
        self._data_recording_period = data_recording_period
        self._plot_delay = 0
        self._data_recording_delay = 0
        self._time = 0

        #define colors
        # myblack = (0, 0, 0)
        # myblue = (0, 0, 153)
        # mygreen = (0, 102, 51)
        # myred = (255, 0, 0)
        # myteal = (0, 102, 102)
        # mygray = (128, 128, 128)
        mywhite = (255, 255, 255)

        self._plotter._window.setWindowTitle(window_title)

        # define first row
        self._plotter.create_plot_widget(plot_id='north', xlabel='Time (s)', ylabel='north (m)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="north", data_label="north", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='east', xlabel='Time (s)', ylabel='east(m)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="east", data_label="east", data_color=mywhite) 
        #
        self._plotter.create_plot_widget(plot_id='altitude', xlabel='Time (s)', ylabel='altitude (m)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="altitude", data_label="altitude", data_color=mywhite)               
        
        # define second row
        self._plotter.create_plot_widget(plot_id='Va', xlabel='Time (s)', ylabel='Va (m/s)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="Va", data_label="Va", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='alpha', xlabel='Time (s)', ylabel='alpha (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="alpha", data_label="alpha", data_color=mywhite)       
        #
        self._plotter.create_plot_widget(plot_id='beta', xlabel='Time (s)', ylabel='beta (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="beta", data_label="beta", data_color=mywhite)
        
        # define third row
        self._plotter.create_plot_widget(plot_id='roll', xlabel='Time (s)', ylabel='roll (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="roll", data_label="roll", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='pitch', xlabel='Time (s)', ylabel='pitch (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="pitch", data_label="pitch", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='yaw', xlabel='Time (s)', ylabel='yaw (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="yaw", data_label="yaw", data_color=mywhite)

        # define fourth row
        self._plotter.create_plot_widget(plot_id='omega_x', xlabel='Time (s)', ylabel='omega_x (deg/s)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="omega_x", data_label="omega_x", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='omega_y', xlabel='Time (s)', ylabel='omega_y (deg/s)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="omega_y", data_label="omega_y", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='omega_z', xlabel='Time (s)', ylabel='omega_z (deg/s)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="omega_z", data_label="omega_z", data_color=mywhite)

        # define fifth row
        self._plotter.create_plot_widget(plot_id='rudder_x_cmd', xlabel='Time (s)', ylabel='rudder_x_cmd (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="rudder_x_cmd", data_label="rudder_x_cmd", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='rudder_y_cmd', xlabel='Time (s)', ylabel='rudder_y_cmd (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="rudder_y_cmd", data_label="rudder_y_cmd", data_color=mywhite)
        #
        self._plotter.create_plot_widget(plot_id='rudder_z_cmd', xlabel='Time (s)', ylabel='rudder_z_cmd (deg)', window_length=self._data_window_length)
        self._plotter.create_data_set(plot_id="rudder_z_cmd", data_label="rudder_z_cmd", data_color=mywhite)

        self._plotter.show_window()

    def update(self, hsv, delta_c, target):
        if self._data_recording_delay >= self._data_recording_period:
            self.__update_data(hsv, delta_c, target, self._time)
            self._data_recording_delay = 0
        if self._plot_delay >= self._plot_period:
            self.__update_plot()
            self._plot_delay = 0
        self._plot_delay += self._dt
        self._data_recording_delay += self._dt
        self._time += self._dt
        
    def __update_data(self, hsv, delta_c, target, t):
        self._plotter.add_data_point(plot_id='north', data_label='north', xvalue=t, yvalue=hsv.pos[0,0])
        self._plotter.add_data_point(plot_id='east', data_label='east', xvalue=t, yvalue=hsv.pos[1,0])
        self._plotter.add_data_point(plot_id='altitude', data_label='altitude', xvalue=t, yvalue=hsv.pos[2,0])
        Va = np.linalg.norm(hsv.vel)
        self._plotter.add_data_point(plot_id='Va', data_label='Va', xvalue=t, yvalue=Va)
        if hsv.vel[0,0] == 0:
            alpha = np.sign(hsv.vel[2,0])*np.pi/2.
        else:
            alpha = np.arctan(hsv.vel[2,0]/hsv.vel[0,0])            
        self._plotter.add_data_point(plot_id='alpha', data_label='alpha', xvalue=t, yvalue=alpha)
        beta = np.arcsin(hsv.vel[1,0]/Va)
        self._plotter.add_data_point(plot_id='beta', data_label='beta', xvalue=t, yvalue=beta)
        phi, theta, psi = hsv.get_euler()
        self._plotter.add_data_point(plot_id='roll', data_label='roll', xvalue=t, yvalue=self.__rad_to_deg(phi))
        self._plotter.add_data_point(plot_id='pitch', data_label='pitch', xvalue=t, yvalue=self.__rad_to_deg(theta))
        self._plotter.add_data_point(plot_id='yaw', data_label='yaw', xvalue=t, yvalue=self.__rad_to_deg(psi))
        self._plotter.add_data_point(plot_id='omega_x', data_label='omega_x', xvalue=t, yvalue=self.__rad_to_deg(hsv.omega[0,0]))
        self._plotter.add_data_point(plot_id='omega_y', data_label='omega_y', xvalue=t, yvalue=self.__rad_to_deg(hsv.omega[1,0]))
        self._plotter.add_data_point(plot_id='omega_z', data_label='omega_z', xvalue=t, yvalue=self.__rad_to_deg(hsv.omega[2,0]))
        self._plotter.add_data_point(plot_id='rudder_x_cmd', data_label='rudder_x_cmd', xvalue=t, yvalue=self.__rad_to_deg(delta_c.x_rudder))
        self._plotter.add_data_point(plot_id='rudder_y_cmd', data_label='rudder_y_cmd', xvalue=t, yvalue=self.__rad_to_deg(delta_c.y_rudder))
        self._plotter.add_data_point(plot_id='rudder_z_cmd', data_label='rudder_z_cmd', xvalue=t, yvalue=self.__rad_to_deg(delta_c.z_rudder))

    def process_app(self):
        self._plotter.process_app(0)

    def __update_plot(self):
        self._plotter.update_plots()

    def close_data_viewer(self):
        self._plotter.close_window()

    def save_plot_image(self, plot_name):
        self._plotter.save_image(plot_name)

    def __rad_to_deg(self, radians):
        rad = wrap(radians,0)
        return rad*180/np.pi


