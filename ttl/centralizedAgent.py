'''
Author: Tianqi Li
Institution: Mechanical Engineering, TAMU
Date: 2020/09/18

This file defines the class agent with centralized JPDA algorithm, which is 
based on sequential JPDA and a greedy base policy.
Properities:
    1. track-to-track assign function: self.match, which is based on Hungarian algorithm;
    2. TODO exchange information procotol: each agent generates the local message regardless of 
    whether there is an observation associated with the track, which will be enhanced later;
    3. TODO some noisy tracks are confirmed after JDPA, which is supposed to be enhanced later;
'''

from ttl.jpdaSeq import jpdaSeq
from consensus_target_tracking.msg import Agent_basic, Single_track, Info_sense
import numpy as np
from ttl.util import EKFcontrol, measurement, track, vertex
from scipy.optimize import linear_sum_assignment   # Hungarian alg, minimun bipartite matching
import copy
import bisect

class cenAgent:
    def __init__(self, sensor_para_list, dt, v = 5, P0 = 2, isObsdyn = False, isRotate = False, isSimulation=True):
        
        self.sensor_para_list = sensor_para_list
        self.sensor_num = len(sensor_para_list)
        # self.dc = dc
        self.dt = dt
        self.default_bbsize = [1,1]
        # a threshold d0 for metrics
        self.t = 0
        self.isObsdyn = isObsdyn
        self.isSimulation = isSimulation
        self.tracker = jpdaSeq(dt, sensor_para_list, len(sensor_para_list), 
                isSimulation=self.isSimulation, isObsdyn=self.isObsdyn,
                ConfirmationThreshold = [8, 10],
                DeletionThreshold = [10, 12])
        self.v_list = [[0, 0]] * self.sensor_num
        self.isRotate = isRotate
        

    def euclidan_dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def cal_R(self, z, xs, sen_id):
        dx = z[0] - xs[0]
        dy = z[1] - xs[1]
        if np.isclose(dx, 0):
            theta = np.sign(dy) * 0.5 * np.pi- xs[2]
        else:
            theta = np.arctan(dy / dx) - xs[2]
        r = max(self.sensor_para_list[sen_id]["r0"], np.sqrt(dx**2 + dy**2))
        G = np.matrix([[np.cos(theta), -np.sin(theta)], 
                        [np.sin(theta), np.cos(theta)]])
        M = np.diag([0.1 * r, 0.1 * np.pi * r])
        R = self.sensor_para_list[sen_id]["quality"] * np.dot(np.dot(G, M), G.T)
        R0 = 1 * np.diag([0.1, 0.1])
        if np.trace(R) < np.trace(R0):
            R = R0
        return R

    def obs_update_callback(self, t, dt, z_k, size_k):
        if self.isSimulation:
            # add noise to targets 
            multi_size_k = []
            multi_z_k = []
            obs_points_k = []

            # extract the previous track list
            preTrackList = copy.deepcopy(self.tracker.track_list_next_index)

            if self.isObsdyn:
                
                for j in range(self.sensor_num):
                    
                    size_k_i = []
                    z_k_i = copy.deepcopy(z_k)

                    for i in range(len(z_k)):
                        R = self.cal_R(z_k[i], self.sensor_para_list[j]["position"], j)
                        z_k_i[i][0] += np.random.normal(0, R[0,0])
                        z_k_i[i][1] += np.random.normal(0, R[1,1])
                        size_k_i.append([1, 1, 2, 0.1])
                    
                    z_k_i, size_k = self.tracker.obs_fov(z_k_i, size_k_i, self.sensor_para_list[j]["position"], j)
                    obs_points_k += copy.deepcopy(z_k_i)
                    multi_z_k.append(z_k_i)
                    multi_size_k.append(size_k)
                ellips_inputs_k, bb_output_k = self.tracker.track_update(t, dt, multi_z_k, multi_size_k, self.sensor_para_list)

            else:
                for j in range(self.sensor_num):
                    
                    size_k_i = []
                    z_k_i = copy.deepcopy(z_k)
                    for i in range(len(z_k)):
                        
                        z_k_i[i][0] += np.random.normal(0, self.tracker.R_list[j][0,0])
                        z_k_i[i][1] += np.random.normal(0, self.tracker.R_list[j][1,1])
                        size_k_i.append([1, 1, 2, 0.1])

                    z_k_i, size_k = self.tracker.obs_fov(z_k_i, size_k_i, self.sensor_para_list[j]["position"], j)
                    obs_points_k += copy.deepcopy(z_k_i)
                    multi_z_k.append(z_k_i)
                    multi_size_k.append(size_k)
                ellips_inputs_k, bb_output_k = self.tracker.track_update(t, dt, multi_z_k, multi_size_k, self.sensor_para_list)
            curTrackList = copy.deepcopy(self.tracker.track_list_next_index)
            deletedTrackList = list(set(preTrackList) - set(curTrackList))
            return ellips_inputs_k, bb_output_k, deletedTrackList
        
        else:
            ellips_inputs_k, bb_output_k = self.tracker.track_update(t, dt, z_k, size_k, self.sensor_para_list)
            return ellips_inputs_k, bb_output_k
    
    def base_policy(self, sen_id, tar_pos):
        sen_pos = self.sensor_para_list[sen_id]["position"][0:2]
        dx = tar_pos[0] - sen_pos[0]
        dy = tar_pos[1] - sen_pos[1]
        d = np.sqrt(dx**2 + dy**2)
        alpha = 1
        # min(worst_P / self.P0, 1)
        v_bar = self.sensor_para_list[sen_id]["v"]
        self.v_list[sen_id] = [alpha * v_bar* dx / d, alpha * v_bar * dy / d]
        
        return

    def central_base_policy(self, ellips_inputs_k):

        # 1. rank all current tracks, and picked the worst sensor_num tracks among them
        trace_list = {}
        for i in range(len(ellips_inputs_k)):
            trace_list[i] = np.trace(ellips_inputs_k[i].kf.P_k_k[0:2, 0:2])
        
        sort_orders = sorted(trace_list.items(), key=lambda x: x[1], reverse=True)

        # the number of tracks may be smaller than number of sensor
        m = min(len(trace_list), self.sensor_num)
        worst_points = []
        for i in range(m):
            index = sort_orders[i][0]
            worst_points.append([ellips_inputs_k[index].kf.x_k_k[0,0], ellips_inputs_k[index].kf.x_k_k[1,0]])

        # 2. assign all sensors to these tracks by minimum bipartite matching
        dis_matrix = np.full((self.sensor_num, self.sensor_num), 10000)
        
        for i in range(self.sensor_num):
            for j in range(m):
                sen_pos = self.sensor_para_list[i]["position"][0:2]
                tar_pos = worst_points[j]
                dis_matrix[i, j] = self.euclidan_dist(sen_pos, tar_pos)
        
        # after preparing the bipartite graph matrix, implement Hungarian alg
        row_ind, col_ind = linear_sum_assignment(dis_matrix)

        # then deploy the sensors based on matching
        for i in range(self.sensor_num):
            try:
                tar_pos = worst_points[col_ind[i]]
                self.base_policy(i, tar_pos)
            except:
                self.v_list[i] = [0,0]
        
        # (self.v_list)print
        return
    
    def scenarioPolicy(self, t, scenario):

        if scenario == 1:
            TimeIntervalAg1 = [3, 15, 20,30]
            v1 = [[0, 0], [0, -1.5], [0, -6], [-5, 0], [0,0]]
            self.v_list[0] = v1[bisect.bisect(TimeIntervalAg1, t)]
            TimeIntervalAg2 = [25, 40]
            v2 = [[0, 0], [0, -2], [0,0]]
            self.v_list[1] = v2[bisect.bisect(TimeIntervalAg2, t)]
        elif scenario == 2:
            TimeIntervalAg1 = [3, 15, 20,30,37]
            v1 = [[0, 0], [0, -1.5], [0, -6], [-5, 0], [0, 5], [0,0]]
            self.v_list[0] = v1[bisect.bisect(TimeIntervalAg1, t)]
            TimeIntervalAg2 = [25, 40]
            v2 = [[0, 0], [0, -2], [0,0]]
            self.v_list[1] = v2[bisect.bisect(TimeIntervalAg2, t)]
        else:
            raise RuntimeError("Undefined scenario case, please run python3 main.py ttl 1 or python3 main.py ttl 2")
        
    
    def updateAngle(self, i):
        vx = self.v_list[i][0]
        vy = self.v_list[i][1]
        if abs(vx) + abs(vy) > 0:
            if np.isclose(vx, 0):
                theta = np.sign(vy) * np.pi/2
            else:
                theta = np.arctan(vy/vx)
                if theta > 0:
                    if vx < 0:
                        theta += np.pi
                else:
                    if vx < 0:
                        theta += np.pi
            self.sensor_para_list[i]["position"][2] = theta

    def dynamics(self):
        # update the position of sensors
        for i in range(self.sensor_num):

            self.sensor_para_list[i]["position"][0] += self.v_list[i][0]
            self.sensor_para_list[i]["position"][1] += self.v_list[i][1]
            if self.isRotate:
                self.updateAngle(i)