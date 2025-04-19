# dubins_extender
#   - Tool that iterates through various methods to extend a dubins path to a desired length
import numpy as np
from tools.angles import rotz, inHalfSpace
from message_types.msg_waypoints import MsgWaypoints
from planners.dubins_parameters import DubinsParameters
from scipy.optimize import brentq, newton

class DubinsExtender:
    def __init__(self, hsv, max_path):
        self.path_manager = hsv.path_manager
        self.hsv = hsv
        self.waypoints = hsv.path_manager.waypoints
        self.R = hsv.dynamics.params.min_turn_radius
        self.max_path = max_path
        self.pot_ext_type = "radius"
        self.min_overestimate = None
      
    
    def extend_dubins(self):
        self.db = self.path_manager.dubins_path
        print("Extending with radius:")
        self.ext_increase_radius()
        if self.pot_ext_type == "detour": # if, after trying to extend radius, an exact solution cannot be found, try detour method
            print("Extending with detour:")
            self.extend_dubins_w_detour(self.prev_max) # move on to path elongation
        
        return self.waypoints
    
    def ext_increase_radius(self, prev_max = None):
        turn_type = self.db.L_list[self.db.turn_type_index][1]
        
        obj_func = lambda R_new: self.db.compute_L(turn_type, R_new, recompute_circles = True)[0] - self.max_path[1]
        root_R = brentq(obj_func, self.hsv.dynamics.params.min_turn_radius, self.max_path[1]*10) 
        calculated_path = self.db.compute_L(turn_type, root_R, recompute_circles = True)[0]
        
        # if you can get the desired path length by increasing the radius
        if np.isclose(calculated_path, self.max_path[1]): 
            self.R = root_R
            print("Extending radius to", self.R, "for path type", self.db.L_list[self.db.turn_type_index][1], "to a path length of", calculated_path, "\n")
            self.update_dubins(reset_dubins = True) 
            return
        
        # if you cannot get exact length desired
        else:
            if prev_max == np.inf: # if you are approximating with the current value (if this is the best solution)
                if calculated_path < self.max_path[1]:
                    path_after_discontinuity = self.db.compute_L(turn_type, root_R + 1.0, recompute_circles = True)[0] # check length after discontinuity
                else:
                    path_after_discontinuity = calculated_path
                    calculated_path = self.db.compute_L(turn_type, root_R - 1, recompute_circles = True)[0]
                    root_R -= 1
                if abs(calculated_path - self.max_path[1]) < abs(path_after_discontinuity - self.max_path[1]):
                    self.R = root_R
                    print("Extending radius to", self.R, "for path type", self.db.L_list[self.db.turn_type_index][1], "to a path length of", calculated_path, "\n")
                    self.update_dubins(reset_dubins = True)
                    return
                else:
                    self.R = root_R + 1
                    print("Extending radius to", self.R, "for path type", self.db.L_list[self.db.turn_type_index][1], "to a path length of", calculated_path, "\n")
                    self.update_dubins(reset_dubins = True)
                    return
                
            ## set up to try another path (manage discontinuity)  
            if calculated_path < self.max_path[1]: 
                path_after_discontinuity = self.db.compute_L(turn_type, root_R + 1.0, recompute_circles = True)[0] # check length after discontinuity
            else:
                path_after_discontinuity = calculated_path
                calculated_path = self.db.compute_L(turn_type, root_R - 1.0, recompute_circles = True)[0]
            
            if path_after_discontinuity < self.max_path[1] and path_after_discontinuity > calculated_path:
                calculated_path = path_after_discontinuity 
            elif self.min_overestimate == None or path_after_discontinuity < self.min_overestimate[0]:
                    self.min_overestimate = (path_after_discontinuity, self.db.turn_type_index, "radius")
                
            self.try_another_dubins_path(calculated_path, prev_max)
            
                
        
    def extend_dubins_w_detour(self, prev_max = None):
        len_to_add = self.max_path[1] - self.db.length
        s_len_to_add = len_to_add*0.5 + (2-np.pi)*self.R # this is the "height" of the tower shape you are creating
        
        # Case 1 and Case 2
        if self.db.L_s < 4*self.R: # if you can only max out the path so much
            self.get_waypoints_with_upper_limit(prev_max)
                    
        #Case 3: L_s long enough to do full tower, and tower is needed
        elif s_len_to_add >= 0: # full "tower"
            self.full_tower(s_len_to_add)
        
        #Case 4: L_s long enough to do full tower, but full tower is not needed
        else: # no "tower"
            print("Case 4: No limit to max extension, but full tower not needed\n")
            _lambda = self.root_find_lambda()
                        
            self.extend_no_tower(_lambda)
    
    def root_find_lambda(self):
        y_int = self.db.L_s - (self.max_path[1] - self.db.L_c1 - self.db.L_c2)
        
        f = lambda angle: self.new_L_s(angle) + y_int 
        f_prime = lambda angle: self.deriv_new_L_s(angle)
        _lambda = newton(f, np.pi, fprime = f_prime) 
            
        return _lambda
     
    def new_L_s(self, angle):
        return -4*np.sin(angle)*self.R + 4*angle*self.R

    def deriv_new_L_s(self, angle):
        return -4*np.cos(angle)*self.R + 4*self.R

    def get_waypoints_with_upper_limit(self, prev_max):
        _lambda_max = np.arcsin(self.db.L_s/(4*self.R))
        curr_max_len = 4*_lambda_max*self.R + self.db.L_c1 + self.db.L_c2
        
        # Case 1: L_s not long enough to do full tower, path can be extend to match max_path
        if curr_max_len >= self.max_path[1]:
            print("Case 1: Max path extension is limited but sufficient \n")
            _lambda = self.root_find_lambda()
            
            self.extend_no_tower(_lambda)
            
        # Case 2: L_s not long enough to do full tower, and path cannot be extend to match max_path
        else: 
            
            # if this is the final iteration of the recursive function and you are approximating to max the path out as much as possible
            if prev_max == np.inf:
                print("Extending path of turn type ", self.db.L_list[self.db.turn_type_index][1], " for a distance of ", curr_max_len, "\n")
                self.extend_no_tower(_lambda_max)
                return
            
            self.try_another_dubins_path(curr_max_len, prev_max, _lambda_max = _lambda_max)
            
    def try_another_dubins_path(self, curr_max_len, prev_max, _lambda_max = None):
        if self.pot_ext_type == "detour":
            print("Case 2: Max extension is limited.")
        print("Max extension (",  curr_max_len, ") for this dubins path is not long enough (needs to be ", self.max_path[1],").", sep="")

        if prev_max is None or curr_max_len > prev_max[0]:
                prev_max = (curr_max_len, self.db.turn_type_index, self.pot_ext_type)
        
        # Case 2a: haven't tried all dubins paths yet
        if (self.db.turn_type_index <= len(self.db.L_list) - 1) and (self.db.L_list[self.db.turn_type_index + 1][0] < self.max_path[1]): # if you haven't tried all dubins paths yet and the next one is still less than max_path
            print("Going to the next dubins turn type", self.db.L_list[self.db.turn_type_index + 1][1], "of length", self.db.L_list[self.db.turn_type_index + 1][0])
            self.path_manager.dubins_path.turn_type_index += 1
            self.update_dubins()
            if self.pot_ext_type == "detour":
                self.extend_dubins_w_detour(prev_max) # try extending the next longest dubins path. Keep going until you are successful or you have tried all 4 paths
            elif self.pot_ext_type == "radius":
                self.ext_increase_radius(prev_max)
            else:
                print("Invalid extension type for dubins extension")
                exit()
            
        # Case 2b: exact extension not possible
        else: # if you have tried all dubins paths that are shorter than the max_path
            if self.pot_ext_type == "detour":
                self.approximate_extension(prev_max, curr_max_len, _lambda_max)
            elif self.pot_ext_type == "radius":
                self.path_manager.dubins_path.turn_type_index = 0
                self.update_dubins()
                self.pot_ext_type = "detour"
                self.prev_max = prev_max
            else:
                print("Invalid extension type for dubins extension")
                exit()
    
    def approximate_extension(self, prev_max, curr_max_len, _lambda_max):
        print("Cannot get exact extension. Approximate solution is required")
        
        # check to see if an unaltered dubins path is better than your best extended path
        if self.db.turn_type_index <= (len(self.db.L_list) - 1): # if there are more dubins paths (unaltered that are longer than max_path)
            if prev_max is None:
                max_modified_len = curr_max_len
            else:
                max_modified_len = max(prev_max[0], curr_max_len)
        
            for i in range(self.db.turn_type_index + 1, len(self.db.L_list)):
                print("Next unmodified dubins path length is", self.db.L_list[i][0])
                if abs(self.db.L_list[i][0] - self.max_path[1]) < abs(self.max_path[1] - max_modified_len): # if the next unaltered dubins path is closer than extending up to the max that the previous path can be extendd
                    if abs(self.db.L_list[i][0] - self.max_path[1]) < abs(self.min_overestimate[0] - self.max_path[1]): # if the next unaltered dubins path is closer than the minimum overshoot (after discontinuity)
                        self.path_manager.dubins_path.turn_type_index = i
                        print("Closest path is un-modified dubins path of turn type", self.db.L_list[self.db.turn_type_index][1], "(", self.db.L_list[self.db.turn_type_index][0], ")", "\n")
                        self.update_dubins() 
                        return 
                    else:
                        self.path_manager.dubins_path.turn_type_index = self.min_overestimate[1]
                        self.ext_increase_radius(np.inf)
                
        # find your best extended path and apply it
        if (prev_max is None) or prev_max[0] <= curr_max_len: 
            print("Extending path of turn type", self.db.L_list[self.db.turn_type_index][1], "for a distance of ", curr_max_len, "\n")
            self.extend_no_tower(_lambda_max)
        else:  # if the closest path was a previously extended path
            self.path_manager.dubins_path.turn_type_index = prev_max[1]
            self.update_dubins(reset_dubins=True)
            if prev_max[2] == "detour":
                self.extend_dubins_w_detour(np.inf)
            if prev_max[2] == "radius":
                self.ext_increase_radius(np.inf)
                        
    def extend_no_tower(self, _lambda):
        e1 = np.array([[1], [0], [0]])
        
        num_new_waypoints = 5
            
        waypoints_pos = [None]*num_new_waypoints 
        waypoints_crs = [None]*num_new_waypoints 
        
        waypoints_pos[0] = self.waypoints.ned[:, 0][:, np.newaxis]
        waypoints_crs[0] = rotz(self.waypoints.course[0])@e1
        
        # find 3 new dubins waypoints (4th will be the last one from before)
        z = (self.db.L_s - 4*self.R*np.sin(_lambda))/2 # first straight line segment
        if np.isclose(z, 0):
            z = 0 
        way1_pos_options = (self.db.r1 + z*self.db.n1 + self.R*(rotz(-np.pi/2) + rotz(-_lambda + np.pi/2))@self.db.n1, \
                                self.db.r1 + z*self.db.n1 + self.R*(rotz(np.pi/2)  + rotz(_lambda - np.pi/2))@self.db.n1)
        way1_crs_options = (rotz(-_lambda)@self.db.n1, rotz(_lambda)@self.db.n1)
        
        #decide which direction to go in extension
        if z == 0:
            if not inHalfSpace(self.db.p_s, self.db.r1, rotz(-np.pi/2)@self.db.n1):
                waypoints_pos[1] = way1_pos_options[0]
                waypoints_crs[1] = way1_crs_options[0]
            else:
                waypoints_pos[1] = way1_pos_options[1]
                waypoints_crs[1] = way1_crs_options[1]
        else: 
            if inHalfSpace(self.hsv.target.state.pos, self.db.r1, rotz(-np.pi/2)@self.db.n1) and inHalfSpace(way1_pos_options[0], self.db.r1, rotz(-np.pi/2)@self.db.n1):
                waypoints_pos[1] = way1_pos_options[1]
                waypoints_crs[1] = way1_crs_options[1]
            else:
                waypoints_pos[1] = way1_pos_options[0]
                waypoints_crs[1] = way1_crs_options[0]
        
        waypoints_pos[2] = self.db.r1 + (self.db.L_s - z)*self.db.n1
        waypoints_crs[2] = self.db.n1
        
        # keep last waypoint from before
        waypoints_pos[3] = self.waypoints.ned[:, 1]
        waypoints_pos[3] = waypoints_pos[3][:, np.newaxis]
        waypoints_crs[3] = rotz(self.waypoints.course[1])@e1
        
        waypoints_pos[4] = self.waypoints.ned[:, 2]
        waypoints_pos[4] = waypoints_pos[4][:, np.newaxis]
        
        # wipe waypoints and then add the following: current pose, dubins 1-4
        self.waypoints = MsgWaypoints()
        self.waypoints.type = 'dubins'
        for j in range(num_new_waypoints - 1):
            course = np.arctan2(waypoints_crs[j][1, 0], waypoints_crs[j][0, 0])
            self.waypoints.add(ned = waypoints_pos[j], course = course)
            
        self.waypoints.add(ned = waypoints_pos[4])

        self.update_dubins(reset_dubins = True)

    def full_tower(self, s_len_to_add):
        print("Case 3: Full tower\n")
        # do four ninety degree turns plus whatever straight distance needs to be added
        _lambda = np.pi/2
        e1 = np.array([[1], [0], [0]])
        num_new_waypoints = 6
        
        waypoints_pos = [None]*num_new_waypoints 
        waypoints_crs = [None]*num_new_waypoints 
        
        # generate new waypoints (get rid of old ones)
        waypoints_pos[0] = self.waypoints.ned[:, 0][:, np.newaxis]
        waypoints_crs[0] = rotz(self.waypoints.course[0])@e1
        
        # find 3 new dubins waypoints (4th will be the last one from before)
        way1_crs_options = (rotz(self.db.dir_s*-np.pi/2)@self.db.n1, rotz(self.db.dir_s*np.pi/2)@self.db.n1)
        way1_pos_options = (self.db.center_s + (self.db.L_s/2 - self.R)*self.db.n1  + 2*self.R*way1_crs_options[0], 
                            self.db.center_s + (self.db.L_s/2 - self.R)*self.db.n1)
        if inHalfSpace(self.hsv.target.state.pos, self.db.r1, rotz(-np.pi/2)@self.db.n1) and inHalfSpace(way1_pos_options[0], self.db.r1, rotz(-np.pi/2)@self.db.n1):
            waypoints_crs[1] = way1_crs_options[1]
            waypoints_pos[1] = way1_pos_options[1]
        else:
            waypoints_crs[1] = way1_crs_options[0]
            waypoints_pos[1] = way1_pos_options[0]
        
        waypoints_pos[2] = waypoints_pos[1] + s_len_to_add*waypoints_crs[1] + 2*self.R*self.db.n1
        waypoints_crs[2] = rotz(np.pi)@waypoints_crs[1]
        
        waypoints_pos[3] = self.db.r1 + (self.db.L_s/2 + 2*self.R)*self.db.n1
        waypoints_crs[3] = self.db.n1
        
        # keep last waypoint from before
        waypoints_pos[4] = self.waypoints.ned[:, 1]
        waypoints_pos[4] = waypoints_pos[4][:, np.newaxis]
        waypoints_crs[4] = rotz(self.waypoints.course[1])@e1
        
        waypoints_pos[5] = self.waypoints.ned[:, 2]
        waypoints_pos[5] = waypoints_pos[5][:, np.newaxis]
        
        # wipe waypoints and then add the following: current pose, dubins 1-4
        self.waypoints = MsgWaypoints()
        self.waypoints.type = 'dubins'
        for i in range(num_new_waypoints - 1):
            course = np.arctan2(waypoints_crs[i][1, 0], waypoints_crs[i][0, 0])
            self.waypoints.add(ned = waypoints_pos[i], course = course)
        
        self.waypoints.add(ned = waypoints_pos[5])

        self.update_dubins(reset_dubins = True)
        
    def update_dubins(self, reset_dubins = False):
        self.waypoints.flag_waypoints_changed = True
        self.hsv.path_manager.waypoints = self.waypoints
        if reset_dubins: # TODO: something wrong here with detour
            turn_type = self.db.L_list[self.db.turn_type_index][1]
            self.hsv.path_manager.dubins_path = DubinsParameters(self.hsv.dynamics)
            self.hsv.path_manager.waypoints.flag_waypoints_changed = True
            self.hsv.path_manager.dubins_path.match_start_turn = turn_type
        self.hsv.path_manager.manager_state = 1
        self.hsv.path_manager.update(self.hsv, self.R)
           