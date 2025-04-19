# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB
#         3/30/2022 - RWB
#         7/13/2023 - RWB

import numpy as np
import matplotlib.pyplot as plt
from tools.plot_dubins import plot_dubins
import parameters.simulation_parameters as SIM
from tools.angles import rotz

class DubinsParameters:
    def __init__(self, model):
        self.L_list = [[None, i] for i in range(4)]
        self.turn_type_index = 0
        self.match_start_turn = None
        self.model = model
        
    def update(self, hsv, ps, chis, pe, chie, R):
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.compute_parameters(hsv)

    def compute_parameters(self, hsv):
        target_pos = hsv.target.state.pos
        ps = self.p_s
        pe = self.p_e
        chis = self.chi_s
        chie = self.chi_e
        R = self.radius
        ell = np.linalg.norm(ps[0:2] - pe[0:2])
        
        if self.L_list[0][0] == None:
            # compute start and end circles
            self.crs = ps + R * rotz(np.pi / 2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            # print("CRS is",self.crs)
            self.cls = ps + R * rotz(-np.pi / 2) @ np.array([[np.cos(chis)], [np.sin(chis)], [0]])
            # print("CLS is",self.cls)
            self.cre = pe + R * rotz(np.pi / 2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            # print("CRE is",self.cre)
            self.cle = pe + R * rotz(-np.pi / 2) @ np.array([[np.cos(chie)], [np.sin(chie)], [0]])
            # print("CLE is",self.cle)

            numDubinsPaths = 4
            self.all_lengths = [[None]*numDubinsPaths]*numDubinsPaths # TODO: change the fours in this file to a variable (which will always be four)       

            for i in range(numDubinsPaths):
                self.all_lengths[i] = self.compute_L(i, R)
        
            for i in range(numDubinsPaths):
                self.L_list[i][0] = self.all_lengths[i][0]
            self.L_list.sort()
            
        turn_type = self.L_list[self.turn_type_index][1]
        
        # Changes the turn type so that the first turn matches the original DB turn (before extension)
        if R == self.model.params.min_turn_radius:
            if self.match_start_turn != None:
                if self.match_start_turn == 0 or self.match_start_turn == 1:
                    while turn_type != 0 and turn_type != 1:
                        self.turn_type_index += 1
                        turn_type = self.L_list[self.turn_type_index][1]
                else:
                    while turn_type != 2 and turn_type != 3:
                        self.turn_type_index += 1
                        turn_type = self.L_list[self.turn_type_index][1]
        else:
            turn_type = self.match_start_turn
        
        e1 = np.array([[1, 0, 0]]).T
        
        # RR
        if turn_type == 0:
            cs = self.crs
            lams = 1
            ce = self.cre
            lame = 1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            w1 = cs + R * rotz(-np.pi / 2) @ q1
            w2 = ce + R * rotz(-np.pi / 2) @ q1
            
        # RL
        elif turn_type == 1:
            cs = self.crs
            lams = 1
            ce = self.cle
            lame = -1
            ell = np.linalg.norm(ce - cs)
            if np.isclose(ell, 2*R) and 2*R > ell:
                ell = 2*R
            theta = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            theta2 = theta - np.pi / 2 + np.arcsin(2 * R / ell)
            q1 = rotz(theta2 + np.pi / 2) @ e1
            w1 = cs + R * rotz(theta2) @ e1
            w2 = ce + R * rotz(theta2 + np.pi) @ e1
        
        # LR
        elif turn_type == 2:
            cs = self.cls
            lams = -1
            ce = self.cre
            lame = 1
            ell = np.linalg.norm(ce - cs)
            if np.isclose(ell, 2*R) and 2*R > ell:
                ell = 2*R
            theta = np.arctan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            theta2 = np.arccos(2 * R / ell)
            q1 = rotz(theta + theta2 - np.pi / 2) @ e1
            w1 = cs + R * rotz(theta + theta2) @ e1
            w2 = ce + R * rotz(-np.pi + theta + theta2) @ e1
        
        # LL
        elif turn_type == 3:
            cs = self.cls
            lams = -1
            ce = self.cle
            lame = -1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            w1 = cs + R * rotz(np.pi / 2) @ q1
            w2 = ce + R * rotz(np.pi / 2) @ q1
        w3 = pe
        q3 = rotz(chie) @ e1
        self.length = self.all_lengths[turn_type][0]
        self.center_s = cs
        self.dir_s = lams
        self.center_e = ce
        self.dir_e = lame
        self.r1 = w1
        self.n1 = q1
        self.r2 = w2
        self.r3 = w3
        self.n3 = q3
        self.L_s = self.all_lengths[turn_type][1]
        self.L_c1 = self.all_lengths[turn_type][2]
        self.L_c2 = self.all_lengths[turn_type][3]
        
        if SIM.plot_dubins:
            plot_dubins(hsv.missile_id, target_pos, self.p_s, self.p_e, self.crs, self.cls, self.cre, self.cle, w1, w2, w3, q1, q3, R)

    def compute_L(self, turn_type, R, recompute_circles = False):
        
        if recompute_circles: # this is important for finding a new radius in dubins_extender.py
            crs = self.p_s + R * rotz(np.pi / 2) @ np.array([[np.cos(self.chi_s)], [np.sin(self.chi_s)], [0]])
            cls = self.p_s + R * rotz(-np.pi / 2) @ np.array([[np.cos(self.chi_s)], [np.sin(self.chi_s)], [0]])
            cre = self.p_e + R * rotz(np.pi / 2) @ np.array([[np.cos(self.chi_e)], [np.sin(self.chi_e)], [0]])
            cle = self.p_e + R * rotz(-np.pi / 2) @ np.array([[np.cos(self.chi_e)], [np.sin(self.chi_e)], [0]])
        else:
            crs = self.crs
            cls = self.cls
            cre = self.cre
            cle = self.cle
        
        # RR
        if turn_type == 0:
            theta = np.arctan2(cre.item(1) - crs.item(1), cre.item(0) - crs.item(0))
           
            L_s = np.linalg.norm(crs - cre)
            L_c1 = R * mod(2 * np.pi + mod(theta - np.pi / 2) - mod(self.chi_s - np.pi / 2))
            L_c2 = R * mod(2 * np.pi + mod(self.chi_e - np.pi / 2) - mod(theta - np.pi / 2))
        
        # RL
        elif turn_type == 1:
            ell = np.linalg.norm(cle - crs)
            theta = np.arctan2(cle.item(1) - crs.item(1), cle.item(0) - crs.item(0))
            if np.isclose(ell, 2*R) and 2*R > ell:
                ell = 2*R
            if 2*R > ell:
                return np.inf, np.inf, np.inf, np.inf
            
            theta2 = theta - np.pi / 2 + np.arcsin(2 * R / ell)
            
            if np.isclose(ell**2, 4*R**2, atol = 1e-06):
                L_s = 0
            else:
                L_s = np.sqrt(ell ** 2 - 4 * R ** 2)
            L_c1 = R * mod(2 * np.pi + mod(theta2) - mod(self.chi_s - np.pi / 2))
            L_c2 = R * mod(2 * np.pi + mod(theta2 + np.pi) - mod(self.chi_e + np.pi / 2))
        
        # LR
        elif turn_type == 2:
            ell = np.linalg.norm(cre - cls)
            theta = np.arctan2(cre.item(1) - cls.item(1), cre.item(0) - cls.item(0))
            if np.isclose(ell, 2*R) and 2*R > ell:
                ell = 2*R # TODO: what to do about this and changing radius?
            if 2*R > ell:
                return np.inf, np.inf, np.inf, np.inf
                
            theta2 = np.arccos(2 * R / ell)
            
            if np.isclose(ell**2, 4*R**2, atol = 1e-06):
                L_s = 0
            else:
                L_s = np.sqrt(ell ** 2 - 4 * R ** 2)
            L_c1 =  R * mod(2 * np.pi - mod(theta + theta2) + mod(self.chi_s + np.pi / 2))
            L_c2 = R * mod(2 * np.pi - mod(theta + theta2 - np.pi) + mod(self.chi_e - np.pi / 2))
            
        # LL
        elif turn_type == 3:
            theta = np.arctan2(cle.item(1) - cls.item(1), cle.item(0) - cls.item(0))
            L_s = np.linalg.norm(cls - cle)
            L_c1 = R * mod(2 * np.pi - mod(theta + np.pi / 2) + mod(self.chi_s + np.pi / 2))
            L_c2 = R * mod(2 * np.pi - mod(self.chi_e + np.pi / 2) + mod(theta + np.pi / 2))
            
        L = L_s + L_c1 + L_c2
        return [L, L_s, L_c1, L_c2]

    def compute_points(self):
        Del = .01  # distance between point

        # points along start circle
        th1 = np.arctan2(self.p_s.item(1) - self.center_s.item(1),
                         self.p_s.item(0) - self.center_s.item(0))
        th1 = mod(th1)
        th2 = np.arctan2(self.r1.item(1) - self.center_s.item(1),
                         self.r1.item(0) - self.center_s.item(0))
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if self.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2*np.pi - Del:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2 - Del:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2*np.pi + Del:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2 + Del:
                    th -= Del
                    theta_list.append(th)

        points = np.array([[self.center_s.item(0) + self.radius * np.cos(theta_list[0]),
                            self.center_s.item(1) + self.radius * np.sin(theta_list[0]),
                            self.center_s.item(2)]])
        for angle in theta_list:
            new_point = np.array([[self.center_s.item(0) + self.radius * np.cos(angle),
                                   self.center_s.item(1) + self.radius * np.sin(angle),
                                   self.center_s.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        sig = 0
        while sig <= 1:
            new_point = np.array([[(1 - sig) * self.r1.item(0) + sig * self.r2.item(0),
                                   (1 - sig) * self.r1.item(1) + sig * self.r2.item(1),
                                   (1 - sig) * self.r1.item(2) + sig * self.r2.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
            sig += Del

        # points along end cirself.cle
        th2 = np.arctan2(self.p_e.item(1) - self.center_e.item(1),
                         self.p_e.item(0) - self.center_e.item(0))
        th2 = mod(th2)
        th1 = np.arctan2(self.r2.item(1) - self.center_e.item(1),
                         self.r2.item(0) - self.center_e.item(0))
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if self.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi - Del:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2 - Del:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi + Del:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2 + Del:
                    th -= Del
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array([[self.center_e.item(0) + self.radius * np.cos(angle),
                                   self.center_e.item(1) + self.radius * np.sin(angle),
                                   self.center_e.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        return points

def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi or np.isclose(x, 2*np.pi):
        x -= 2*np.pi
    return x
