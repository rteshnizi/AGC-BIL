# version adapted for the demo

import numpy as np
from numpy.linalg import inv
import copy
from scipy.stats import multivariate_normal
from scipy.stats.distributions import chi2
from numpy import linalg as LA
# from matplotlib.patches import Ellipse
from ttl.util import EKFcontrol, dynamic_kf, measurement, track

def normalize(a_list):
    the_sum = sum(a_list)
    a_list = a_list / the_sum
    return a_list

def checkIfDuplicates(listOfElems):
    ''' Check if given list contains any duplicates, for example A B C is good, but A B B is not '''    
    setOfElems = set()
    # special case, all are false alarm, so the sum should be len * -1
    sum_ = sum(listOfElems)
    if sum_ ==  - len(listOfElems):
        return True
    for elem in listOfElems:
        if elem in setOfElems:
            if elem > 0: 
                return True
        else:
            setOfElems.add(elem)         
    return False

def common_fact(beta, P_D, N_T, N_o):
    if N_o > N_T:
        return beta ** (N_o - N_T)
    else:
        return (1 - P_D) ** (N_T - N_o)



class track_seq(track):

    def __init__(self, t0, id_, kf, DeletionThreshold, ConfirmationThreshold, sensor_num):
        track.__init__(self, t0, id_, kf, DeletionThreshold, ConfirmationThreshold)
        self.sensor_num = sensor_num
        self.isObs = False
    
    def update(self, t, kf, isObs, sensor_id):
        if isObs:
            self.kf = kf
        self.isObs += isObs
        if sensor_id == self.sensor_num - 1:
            # update here is there is no obs from all sensors
            if not self.isObs:
                self.kf.x_k_k = self.kf.x_k_k_min
                self.kf.P_k_k = self.kf.P_k_k_min
            self.kf.predict()
            self.record["points"].append(self.kf.getEst())
            
            if self.confirmed:
                # check if there is observation inside the gate
                if self.isObs:
                    self.history.append(1)
                    
                else:
                    self.history.append(0)
                
                
                if len(self.history) > self.DeletionThreshold[1]:
                    self.history.pop(0)
                        
                if len(self.history) == self.DeletionThreshold[1] and sum(self.history) <= self.DeletionThreshold[1] - self.DeletionThreshold[0]:
                    #  reaches the threshold to delete
                    self.deletion(t)
            else:
                # check if there is observation inside the gate
                if self.isObs:
                    self.history.append(1)
                    
                else:
                    self.history.append(0)
                    
                
                #   when enough observation is done, check the N/M
                if len(self.history) == self.ConfirmationThreshold[1]:
                    if sum(self.history) < self.ConfirmationThreshold[0]:
                        #  reaches the threshold to abandon, ow, confirm
                        self.abandoned = True
                    else:
                        self.confirmation()
            self.isObs = False


class jpdaSeq:

    def __init__(self, dt, sensor_para, sensor_num,
        P_G = 0.9, P_D = 0.9,
        ConfirmationThreshold = [6, 8],
        DeletionThreshold = [7, 10],
        isSimulation = True,
        isObsdyn = False):

        self.gating_size = chi2.ppf(P_G, df = 2)
        self.P_D = P_D  # probability of detection
        # Define gating size, for elliposidal gating, we need parameter P_G
        self.P_G = P_G  
        self.dt = dt
        self.F = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        self.Q = 1 * np.diag([0.1, 0.1, 0.5, 0.5])
        self.P = self.Q
        self.R_list = []
        # self.R = 1 * np.diag([0.1, 0.1])
        self.common_P_list = []
        self.isObsdyn = isObsdyn
        for i in range(sensor_num):
            if self.isObsdyn:
                self.R_list.append(np.diag([sensor_para[i]["r"], sensor_para[i]["r"]]))
            else:
                self.R_list.append(sensor_para[i]["quality"] * np.diag([sensor_para[i]["r"], sensor_para[i]["r"]]))
        if self.isObsdyn:
            for i in range(len(sensor_para)):
                common_P = 2 * sensor_para[i]["dm"] * 0.1 * sensor_para[i]["quality"] * 2
                self.common_P_list.append(common_P)
        else:
            for i in range(sensor_num):
                self.common_P_list.append(3 * np.trace(self.R_list[i]))
                
        if DeletionThreshold[1] < ConfirmationThreshold[1]:
            print("Deletion threshold period should be larger than confirmation threshold period")
        self.ConfirmationThreshold = ConfirmationThreshold
        self.DeletionThreshold = DeletionThreshold
        self.isSimulation = isSimulation
        self.track_list = []
        self.track_list_next_index = []
        # self.t = time.time()         ## time synchronize will be important
        self.sensor_para = sensor_para
        self.sensor_num = sensor_num

    def obs_fov(self, z_k, size_k, xs, sen_id):
        
        if self.isObsdyn:
            self.sensor_para[sen_id]["position"][0] = xs[0]
            self.sensor_para[sen_id]["position"][1] = xs[1]
        
        size_inside = []
        if self.isSimulation:
            # only analyse FoV in simulation
            inside_obs = []
            
            if self.sensor_para[sen_id]["shape"][0] == "circle":
                r = self.sensor_para[sen_id]["shape"][1]
                center = self.sensor_para[sen_id]["position"]
                for i in range(len(z_k)):
                    z = z_k[i]
                    distance = np.sqrt((center[0] - z[0])**2 + (center[1] - z[1])**2)
                    if distance > r:
                        continue
                    else:
                        inside_obs.append(z)
                        size_inside.append(size_k[i])
            elif self.sensor_para[sen_id]["shape"][0] == "rectangle":
                # analysis the shape of rectangle
                width = self.sensor_para[sen_id]["shape"][1][0]
                height = self.sensor_para[sen_id]["shape"][1][1]
                angle = self.sensor_para[sen_id]["position"][2]
                center = self.sensor_para[sen_id]["position"][0:2]
                for i in range(len(z_k)):
                    z = z_k[i]
                    dx = z[0] - center[0]
                    dy = z[1] - center[1]
                    dx_trans = dx * np.cos(angle) - dy * np.sin(angle)
                    dy_trans = dx * np.sin(angle) + dy * np.cos(angle)
                    

                    if 2 * abs(dx_trans) <= width and 2 * abs(dy_trans) <= height:
                        inside_obs.append(z)
                        size_inside.append(size_k[i])
                    
            else:
                print("Unknown shape, cannot handle")
        else:
            inside_obs = z_k
            size_inside = size_k
        return inside_obs, size_inside

    def elliposidualGating(self, z_til, S):
    
        value = np.dot(np.dot(z_til.T, inv(S)), z_til)[0,0]
    
        if value <= self.gating_size:
            return 1
        else:
            return 0
    
    def cal_beta(self, N):
        # TODO Don't define the beta yet
        #     normalized volume for ellipsodial gate is 
        V = np.pi * self.gating_size  # equ. 6.28 b
        return N / V

    def track_update(self, t, dt, multi_z_k, multi_size_k, sensor_para_list):
        if self.isObsdyn:
            ellips_inputs_k, bb_output_k = self.track_update_dyn(t, dt, multi_z_k, multi_size_k, sensor_para_list)
        else:
            ellips_inputs_k, bb_output_k = self.track_update_sta(t, dt, multi_z_k, multi_size_k)
        return ellips_inputs_k, bb_output_k

    def track_update_sta(self, t, dt, multi_z_k, multi_size_k):
        # here the format of z_k is 
        # t: time of receiving the observations
        # dt: time between this time and last time of receiving observations
        # z_k: a list of position data, expressed as [[x1, y1], [x2, y2], ...]

        # FoV analysis, to classifiy obs data into 2 parts, inside FoV or outside
        
        if self.isSimulation:
            for i in range(len(multi_z_k)):
                z_k = multi_z_k[i]
                size_k = multi_size_k[i]
                
                z_k, size_k = self.obs_fov(z_k, size_k, self.sensor_para[i]["position"][0:2], i)
                multi_z_k[i] = z_k
                multi_size_k[i] = size_k

        
        # adaptive motion model here
        self.F = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        next_index_list = []
        
        ellips_inputs_k = []
        bb_output_k = []

        bb_boxes = {"id": [], "bb": [], "z": []}
        
        # 1. extract all obs inside elliposidual gates of initialized and confirmed tracks

        obs_matrix_list = []
        for z_k in multi_z_k:
            obs_matrix = np.zeros((len(self.track_list_next_index), len(z_k)))
            obs_matrix_list.append(obs_matrix)

        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            kf = self.track_list[k].kf
            S = np.dot(np.dot(kf.H, kf.P_k_k_min), kf.H.T) + kf.R
            pred_z = np.dot(kf.H, kf.x_k_k_min)
            
            #### usage for plotting ellipse
            self.track_list[k].S = S
            self.track_list[k].pred_z = pred_z
            
            track_id = k
            bb_boxes["id"].append(k)
            bb, zz = [], []
            for l in range(len(multi_z_k)):
                z_k = multi_z_k[l]
                
                for j in range(len(z_k)):
                    z = z_k[j]
                    z_til = pred_z - np.matrix(z).T
                    
                    obs_matrix_list[l][i][j] = self.elliposidualGating(z_til, S)
                    zz.append(multi_z_k[l][j])
                    bb.append(multi_size_k[l][j])
            bb_boxes["bb"].append(bb)
            bb_boxes["z"].append(zz)
        # create a mitrix to check the relationship between obs and measurements

        for l in range(len(obs_matrix_list)):
            # multi sensor change
            obs_matrix = obs_matrix_list[l]
            z_k = multi_z_k[l]

            obs_sum = obs_matrix.sum(axis=0)
            # matrix operation, exclude all observations whose sum is 0, which means its not in any gate
            index = np.where(obs_sum > 0)[0]
            obs_outside_index = np.where(obs_sum == 0)[0]

            # 2. deal with observations inside all gates
            # i. initialize each observation, find out how many gates each observation is in

            obs_class = []
            # now in obs_class, each element is a class with its .track includes all track numbers from 0 - m_k
            # then we need to analyse the gate. First for jth obs z_j in its ith gate we need to calculate
            # g_ij = N(z_j - z_ij|0, S_ij) here z_ij and S_ij is the updated KF given observation z_j


            for i in range(len(z_k)):

                if i in index:
                    try:
                        a_mea = measurement(z_k[i], i)
                    except:
                        print(i, z_k)
                    for j in np.where(obs_matrix[:, i] != 0)[0]:
                        # track is the track id for obs i, in obs_matrix the column value is not 0
                        track_id = self.track_list_next_index[j]
                        a_mea.inside_track(track_id)
                        temp_kf = copy.deepcopy(self.track_list[track_id].kf)
                        temp_kf.update(z_k[i], r=self.R_list[l][0,0])
                        var = multivariate_normal(mean=np.squeeze(temp_kf.z_bar.reshape(2).tolist()[0]), cov=temp_kf.S_k)
                        a_mea.g_ij.append(var.pdf(z_k[i]))
                    obs_class.append(a_mea)
                else:
                    obs_class.append([])

            # ii. for each gate/track, analysis if there exists observations joint different tracks, find out the relation between different tracks


            track_class = []
            for i in range(len(self.track_list_next_index)):
                k = self.track_list_next_index[i]
                a_track = self.track_list[k]   # pointer to track_list class
                a_track.get_measurement(np.where(obs_matrix[i] != 0)[0])
                a_track.measurement.append(-1)
                track_class.append(a_track)



            for i in range(len(track_class)):
                
                # Case 1. if there is no obs inside track gate, 
                # make Kalman's update x_k_k= x_k_k_min, P_k_k = P_k_k_min
                if track_class[i].deleted:
                    continue
                
                if len(track_class[i].measurement) == 1:
                    # there is only false alarm in observation, then only do prediction
                
                    kf = track_class[i].kf
                    #  apparently this is wrong, we need to make observation in Kalman update 
                    #  x_k_k_min, then update the covariance.
                    
                    
                    
                    track_class[i].update(t, kf, False, l)

                    
                        
                    if not (track_class[i].deleted or track_class[i].abandoned):
                        next_index_list.append(self.track_list_next_index[i])
                    continue
                
                # Case 2. there is obs inside the track gate
                # calculate the beta for ith track
                # need the number of measurements inside the gate
                beta = self.cal_beta(len(track_class[i].measurement) - 1)

                
                table_key = [self.track_list_next_index[i]]

                
                # begin find all observations inside this gate that is related to other gates (joint)
                
                for obs_id in track_class[i].measurement:
                    if obs_id != -1:
                        obs = obs_class[obs_id]
                        table_key += obs.track
                    
                table_key = list(set(table_key))
                
                # for each track, figure out how many observations inside the track
                
                # inverse the table
                table_key_inv = []
                for j in table_key:
                    table_key_inv.append(self.track_list_next_index.index(j))
                
                table_key_matrix = obs_matrix[table_key_inv]
                
                ######################
                #         if (table_key_matrix == table_key_matrix[0]).all():
                #             # there are overlaps of tracks, we only remain the one has oldest history
                if (table_key_matrix == table_key_matrix[0]).all() and len(table_key_matrix) > 2:
                    # there are overlaps of tracks, we only remain the one has oldest history
                    seed = min(table_key)
                    for key in table_key:
                        if key != seed:
                            self.track_list[key].deletion(t)
                
                ##### !!!!!!!!
                #         in order to avoid multiple tracks tracking same target, we need to analysis the
                #         obs_matrix
                
                obs_num_tracks = obs_matrix.sum(axis=1)[table_key_inv]

                # number of joint tracks
                N_T = len(table_key)
                # number of observations total
                total_obs = []
                for track_id in table_key:
                    a_track = self.track_list[track_id]
                    total_obs +=a_track.measurement

                total_obs = list(set(total_obs))
                N_o = len(total_obs) - 1

                common_factor = common_fact(beta, self.P_D, N_T, N_o)

                # iii. after merged all related tracks, we generat a hypothesis matrix/table based on the table 
                # generated by the table_key
                obs_num_tracks_ = obs_num_tracks + 1
                total_row = int(obs_num_tracks_.prod())

                # create title for the table


                hyp_matrix = {}
                for a_key in table_key:
                    hyp_matrix[str(a_key)] = []
                hyp_matrix["p"] = []
                
                for row_num in range(total_row):
                    key_num = len(table_key)
                    col_num = 0
                    # build one row of hypothesis
                    while key_num > 0:
                        if col_num == len(table_key) - 1:
                            obs_id = int(row_num)
                            product = 1
                        else:
                            product = obs_num_tracks_[(col_num + 1):].prod()
                            obs_id = int(row_num // product)
                        
                        
                        value = self.track_list[table_key[col_num]].measurement[obs_id]
                        
                        key = str(table_key[col_num])
                        hyp_matrix[key].append(value)
                        row_num = row_num % product
                        col_num += 1
                        key_num -= 1
                    
                    # now we want to calculate the probability of this row's hypothesis
                    hyp_list = []

                    prob = common_factor
                    for key in hyp_matrix.keys():
                        if key != 'p':
                            hyp_list.append(hyp_matrix[key][-1])
                    
                    # print('hyp_list, ', hyp_list)
                    # calculate the prob of this hypothesis
                    if checkIfDuplicates(hyp_list):
                        # this is not one vaild hypothesis.
                        prob = 0
                    else:
                        # this is the valid hypothesis, we should calculate the prob
                        for key in hyp_matrix.keys():
                            if key != 'p':
                                track_id = int(key)
                                obs_id = hyp_matrix[key][-1]
                                # print('obs id ', obs_id)
                                if obs_id == -1:
                                    prob *= (1 - self.P_D) * beta
                                else:
                                    # print(obs_class[obs_id].table, print(obs_class[obs_id].id))
                                    index = obs_class[obs_id].track.index(track_id)
                                    prob *= self.P_D * obs_class[obs_id].g_ij[index]
                    hyp_matrix['p'].append(prob)
                    
                # iv. Then gather the prob in this track, and update the kF
                obs_in_i_track = track_class[i].measurement
                obs_in_i_track_prob = []
                hyp_in_i_track = np.array(hyp_matrix[str(self.track_list_next_index[i])])
                hyp_in_i_track_prob = np.array(hyp_matrix['p'])
                for obs in obs_in_i_track:
                    index_ = np.where(hyp_in_i_track == obs)
                    w_ij_list = hyp_in_i_track_prob[index_]
                    obs_in_i_track_prob.append(w_ij_list.sum())
                
                # then normalize all w_ij s
                obs_in_i_track_prob_norm = normalize(obs_in_i_track_prob)

                # well, we then just need to update the KF of ith track
                kf = track_class[i].kf
                x_k = []
                P_k = []
                bb_k = []
                for obs in obs_in_i_track:
                    if obs == -1:
                        x_k.append(kf.x_k_k)
                        P_k.append(kf.P_k_k)
                    else:
                        
                        z = np.array(z_k[obs]).T

                        # update the kf
                        temp_kf = copy.deepcopy(kf)

                        temp_kf.update(z, r=self.R_list[l][0,0])
                        
                        x_k.append(temp_kf.x_k_k)
                        P_k.append(temp_kf.P_k_k)
                        
                
                x_k_pda = 0 * temp_kf.x_k_k
                P_k_pda = 0 * temp_kf.P_k_k
                for j in range(len(obs_in_i_track_prob_norm)):
                    x_k_pda += obs_in_i_track_prob_norm[j] * x_k[j]

                for j in range(len(obs_in_i_track_prob_norm)):
                    P_k_pda += obs_in_i_track_prob_norm[j] * (P_k[j] + np.dot(x_k_pda - x_k[j], x_k_pda.T - x_k[j].T))

                # update this to kf
                kf.x_k_k = x_k_pda
                kf.P_k_k = P_k_pda
                
                
                if np.trace(kf.P_k_k[0:2, 0:2]) > self.common_P_list[l]:
                    # print("track get deleted here~~")
                    track_class[i].update(t, kf, False, l)
                    # track_class[i].deleted = True
                    continue
                
                
                track_class[i].update(t, kf, True, l)
                
                # # only keep the confirmed ones
                # if track_class[i].confirmed:

                    
                #     # ellips_inputs_k.append([track_class[i].id, track_class[i].kf.x_k_k, track_class[i].kf.P_k_k])
                #     ellips_inputs_k.append(track_class[i])
                #     # pick the bounding box which is the cloest to the associated x_k_k
                #     bb_output_k.append(bb_k[self.find_cloest(x_k_pda, x_k[1:])])
                #     track_class[i].bb_box_size = bb_output_k[-1]
                # save the activated ones for next recursion
                if not (track_class[i].deleted or track_class[i].abandoned):
                    next_index_list.append(self.track_list_next_index[i])



            # 3. deal with observations outside all gates
                
            # now initialize all observations outside of the gate
            for i in obs_outside_index:
                z = z_k[i] + [0, 0]
                x0 = np.matrix(z).T
                kf = EKFcontrol(self.F, self.H, x0, self.P, self.Q, self.R_list[l])
                id_ = len(self.track_list)
                new_track = track_seq(t, id_, kf, self.DeletionThreshold, self.ConfirmationThreshold, self.sensor_num)
                new_track.kf.predict()
                new_track.bb_box_size = [10, 10]  # bb size initialization
                self.track_list.append(new_track)
                next_index_list.append(id_)

        # keep the record
        next_index_list = list(set(next_index_list))
        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            if self.track_list[k].deleted:
                try:
                    next_index_list.remove(k)
                except:
                    pass
            elif self.track_list[k].confirmed:
                ellips_inputs_k.append(self.track_list[k])
                x_k_pda = self.track_list[k].kf.x_k_k
                index_in_bb = bb_boxes["id"].index(k)
                z_list = bb_boxes["z"][index_in_bb]
                index_bb = self.find_cloest(x_k_pda, z_list)
                if index_bb < 0:
                    bb_output_k.append(self.track_list[k].bb_box_size)
                else:
                    bb_output_k.append(bb_boxes["bb"][index_in_bb][index_bb])

                
        #     swap the track list to the new list.
        self.track_list_next_index = next_index_list

        return ellips_inputs_k, bb_output_k
     


    def track_update_dyn(self, t, dt, multi_z_k, multi_size_k, sensor_para_list):
        # here the format of z_k is 
        # t: time of receiving the observations
        # dt: time between this time and last time of receiving observations
        # z_k: a list of position data, expressed as [[x1, y1], [x2, y2], ...]

        # FoV analysis, to classifiy obs data into 2 parts, inside FoV or outside
        
        self.sensor_para = sensor_para_list

        if self.isSimulation:
            for i in range(len(multi_z_k)):
                z_k = multi_z_k[i]
                size_k = multi_size_k[i]
                z_k, size_k = self.obs_fov(z_k, size_k, self.sensor_para[i]["position"][0:2], i)
                multi_z_k[i] = z_k
                multi_size_k[i] = size_k
        
        
        
        # adaptive motion model here
        self.F = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        next_index_list = []
        
        ellips_inputs_k = []
        bb_output_k = []

        bb_boxes = {"id": [], "bb": [], "z": []}
        
        # 1. extract all obs inside elliposidual gates of initialized and confirmed tracks

        obs_matrix_list = []
        for z_k in multi_z_k:
            obs_matrix = np.zeros((len(self.track_list_next_index), len(z_k)))
            obs_matrix_list.append(obs_matrix)

        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            kf = self.track_list[k].kf
            S = np.dot(np.dot(kf.H, kf.P_k_k_min), kf.H.T) + kf.R
            pred_z = np.dot(kf.H, kf.x_k_k_min)
            
            #### usage for plotting ellipse
            self.track_list[k].S = S
            self.track_list[k].pred_z = pred_z
            
            track_id = k
            bb_boxes["id"].append(k)
            bb, zz = [], []
            for l in range(len(multi_z_k)):
                z_k = multi_z_k[l]
                
                for j in range(len(z_k)):
                    z = z_k[j]
                    z_til = pred_z - np.matrix(z).T
                    
                    obs_matrix_list[l][i][j] = self.elliposidualGating(z_til, S)
                    zz.append(multi_z_k[l][j])
                    bb.append(multi_size_k[l][j])
            bb_boxes["bb"].append(bb)
            bb_boxes["z"].append(zz)
        # create a mitrix to check the relationship between obs and measurements

        for l in range(len(obs_matrix_list)):
            # multi sensor change
            obs_matrix = obs_matrix_list[l]
            z_k = multi_z_k[l]

            obs_sum = obs_matrix.sum(axis=0)
            # matrix operation, exclude all observations whose sum is 0, which means its not in any gate
            index = np.where(obs_sum > 0)[0]
            obs_outside_index = np.where(obs_sum == 0)[0]

            # 2. deal with observations inside all gates
            # i. initialize each observation, find out how many gates each observation is in

            obs_class = []
            # now in obs_class, each element is a class with its .track includes all track numbers from 0 - m_k
            # then we need to analyse the gate. First for jth obs z_j in its ith gate we need to calculate
            # g_ij = N(z_j - z_ij|0, S_ij) here z_ij and S_ij is the updated KF given observation z_j


            for i in range(len(z_k)):

                if i in index:
                    try:
                        a_mea = measurement(z_k[i], i)
                    except:
                        print(i, z_k)
                    for j in np.where(obs_matrix[:, i] != 0)[0]:
                        # track is the track id for obs i, in obs_matrix the column value is not 0
                        track_id = self.track_list_next_index[j]
                        a_mea.inside_track(track_id)
                        temp_kf = copy.deepcopy(self.track_list[track_id].kf)
                        temp_kf.update(z_k[i], sensor_para_list[l]["position"], self.sensor_para[l]["quality"])
                        var = multivariate_normal(mean=np.squeeze(temp_kf.z_bar.reshape(2).tolist()[0]), cov=temp_kf.S_k)
                        a_mea.g_ij.append(var.pdf(z_k[i]))
                    obs_class.append(a_mea)
                else:
                    obs_class.append([])

            # ii. for each gate/track, analysis if there exists observations joint different tracks, find out the relation between different tracks


            track_class = []
            for i in range(len(self.track_list_next_index)):
                k = self.track_list_next_index[i]
                a_track = self.track_list[k]   # pointer to track_list class
                a_track.get_measurement(np.where(obs_matrix[i] != 0)[0])
                a_track.measurement.append(-1)
                track_class.append(a_track)



            for i in range(len(track_class)):
                
                # Case 1. if there is no obs inside track gate, 
                # make Kalman's update x_k_k= x_k_k_min, P_k_k = P_k_k_min
                if track_class[i].deleted:
                    continue
                
                if len(track_class[i].measurement) == 1:
                    # there is only false alarm in observation, then only do prediction
                
                    kf = track_class[i].kf
                    #  apparently this is wrong, we need to make observation in Kalman update 
                    #  x_k_k_min, then update the covariance.
                    
                    
                    
                    track_class[i].update(t, kf, False, l)

                    
                        
                    if not (track_class[i].deleted or track_class[i].abandoned):
                        next_index_list.append(self.track_list_next_index[i])
                    continue
                
                # Case 2. there is obs inside the track gate
                # calculate the beta for ith track
                # need the number of measurements inside the gate
                beta = self.cal_beta(len(track_class[i].measurement) - 1)

                
                table_key = [self.track_list_next_index[i]]

                
                # begin find all observations inside this gate that is related to other gates (joint)
                
                for obs_id in track_class[i].measurement:
                    if obs_id != -1:
                        obs = obs_class[obs_id]
                        table_key += obs.track
                    
                table_key = list(set(table_key))
                
                # for each track, figure out how many observations inside the track
                
                # inverse the table
                table_key_inv = []
                for j in table_key:
                    table_key_inv.append(self.track_list_next_index.index(j))
                
                table_key_matrix = obs_matrix[table_key_inv]
                
                ######################
                #         if (table_key_matrix == table_key_matrix[0]).all():
                #             # there are overlaps of tracks, we only remain the one has oldest history
                if (table_key_matrix == table_key_matrix[0]).all() and len(table_key_matrix) > 2:
                    # there are overlaps of tracks, we only remain the one has oldest history
                    seed = min(table_key)
                    for key in table_key:
                        if key != seed:
                            self.track_list[key].deletion(t)
                
                ##### !!!!!!!!
                #         in order to avoid multiple tracks tracking same target, we need to analysis the
                #         obs_matrix
                
                obs_num_tracks = obs_matrix.sum(axis=1)[table_key_inv]

                # number of joint tracks
                N_T = len(table_key)
                # number of observations total
                total_obs = []
                for track_id in table_key:
                    a_track = self.track_list[track_id]
                    total_obs +=a_track.measurement

                total_obs = list(set(total_obs))
                N_o = len(total_obs) - 1

                common_factor = common_fact(beta, self.P_D, N_T, N_o)

                # iii. after merged all related tracks, we generat a hypothesis matrix/table based on the table 
                # generated by the table_key
                obs_num_tracks_ = obs_num_tracks + 1
                total_row = int(obs_num_tracks_.prod())

                # create title for the table


                hyp_matrix = {}
                for a_key in table_key:
                    hyp_matrix[str(a_key)] = []
                hyp_matrix["p"] = []
                
                for row_num in range(total_row):
                    key_num = len(table_key)
                    col_num = 0
                    # build one row of hypothesis
                    while key_num > 0:
                        if col_num == len(table_key) - 1:
                            obs_id = int(row_num)
                            product = 1
                        else:
                            product = obs_num_tracks_[(col_num + 1):].prod()
                            obs_id = int(row_num // product)
                        
                        
                        value = self.track_list[table_key[col_num]].measurement[obs_id]
                        
                        key = str(table_key[col_num])
                        hyp_matrix[key].append(value)
                        row_num = row_num % product
                        col_num += 1
                        key_num -= 1
                    
                    # now we want to calculate the probability of this row's hypothesis
                    hyp_list = []

                    prob = common_factor
                    for key in hyp_matrix.keys():
                        if key != 'p':
                            hyp_list.append(hyp_matrix[key][-1])
                    
                    # print('hyp_list, ', hyp_list)
                    # calculate the prob of this hypothesis
                    if checkIfDuplicates(hyp_list):
                        # this is not one vaild hypothesis.
                        prob = 0
                    else:
                        # this is the valid hypothesis, we should calculate the prob
                        for key in hyp_matrix.keys():
                            if key != 'p':
                                track_id = int(key)
                                obs_id = hyp_matrix[key][-1]
                                # print('obs id ', obs_id)
                                if obs_id == -1:
                                    prob *= (1 - self.P_D) * beta
                                else:
                                    # print(obs_class[obs_id].table, print(obs_class[obs_id].id))
                                    index = obs_class[obs_id].track.index(track_id)
                                    prob *= self.P_D * obs_class[obs_id].g_ij[index]
                    hyp_matrix['p'].append(prob)
                    
                # iv. Then gather the prob in this track, and update the kF
                obs_in_i_track = track_class[i].measurement
                obs_in_i_track_prob = []
                hyp_in_i_track = np.array(hyp_matrix[str(self.track_list_next_index[i])])
                hyp_in_i_track_prob = np.array(hyp_matrix['p'])
                for obs in obs_in_i_track:
                    index_ = np.where(hyp_in_i_track == obs)
                    w_ij_list = hyp_in_i_track_prob[index_]
                    obs_in_i_track_prob.append(w_ij_list.sum())
                
                # then normalize all w_ij s
                obs_in_i_track_prob_norm = normalize(obs_in_i_track_prob)

                # well, we then just need to update the KF of ith track
                kf = track_class[i].kf
                x_k = []
                P_k = []
                bb_k = []
                for obs in obs_in_i_track:
                    if obs == -1:
                        x_k.append(kf.x_k_k)
                        P_k.append(kf.P_k_k)
                    else:
                        
                        z = np.array(z_k[obs]).T

                        # update the kf
                        temp_kf = copy.deepcopy(kf)

                        temp_kf.update(z, sensor_para_list[l]["position"], self.sensor_para[l]["quality"])
                        
                        x_k.append(temp_kf.x_k_k)
                        P_k.append(temp_kf.P_k_k)
                        
                
                x_k_pda = 0 * temp_kf.x_k_k
                P_k_pda = 0 * temp_kf.P_k_k
                for j in range(len(obs_in_i_track_prob_norm)):
                    x_k_pda += obs_in_i_track_prob_norm[j] * x_k[j]

                for j in range(len(obs_in_i_track_prob_norm)):
                    P_k_pda += obs_in_i_track_prob_norm[j] * (P_k[j] + np.dot(x_k_pda - x_k[j], x_k_pda.T - x_k[j].T))

                # update this to kf
                kf.x_k_k = x_k_pda
                kf.P_k_k = P_k_pda
                
                
                if np.trace(kf.P_k_k[0:2, 0:2]) > self.common_P_list[l]:
                    # print("track get deleted here~~")
                    track_class[i].update(t, kf, False, l)
                    # track_class[i].deleted = True
                    continue
                
                
                track_class[i].update(t, kf, True, l)
                
                # # only keep the confirmed ones
                # if track_class[i].confirmed:

                    
                #     # ellips_inputs_k.append([track_class[i].id, track_class[i].kf.x_k_k, track_class[i].kf.P_k_k])
                #     ellips_inputs_k.append(track_class[i])
                #     # pick the bounding box which is the cloest to the associated x_k_k
                #     bb_output_k.append(bb_k[self.find_cloest(x_k_pda, x_k[1:])])
                #     track_class[i].bb_box_size = bb_output_k[-1]
                # save the activated ones for next recursion
                if not (track_class[i].deleted or track_class[i].abandoned):
                    next_index_list.append(self.track_list_next_index[i])



            # 3. deal with observations outside all gates
                
            # now initialize all observations outside of the gate
            for i in obs_outside_index:
                z = z_k[i] + [0, 0]
                x0 = np.matrix(z).T
                kf = dynamic_kf(self.F, self.H, x0, self.P, self.Q, self.R_list[l], self.sensor_para[l]["r0"], quality=self.sensor_para[l]["quality"])
                id_ = len(self.track_list)
                new_track = track_seq(t, id_, kf, self.DeletionThreshold, self.ConfirmationThreshold, self.sensor_num)
                new_track.kf.predict()
                new_track.bb_box_size = [10, 10]  # bb size initialization
                self.track_list.append(new_track)
                next_index_list.append(id_)

        # keep the record
        next_index_list = list(set(next_index_list))
        for i in range(len(self.track_list_next_index)):
            k = self.track_list_next_index[i]
            if self.track_list[k].deleted:
                try:
                    next_index_list.remove(k)
                except:
                    pass
            elif self.track_list[k].confirmed:
                ellips_inputs_k.append(self.track_list[k])
                x_k_pda = self.track_list[k].kf.x_k_k
                index_in_bb = bb_boxes["id"].index(k)
                z_list = bb_boxes["z"][index_in_bb]
                index_bb = self.find_cloest(x_k_pda, z_list)
                if index_bb < 0:
                    bb_output_k.append(self.track_list[k].bb_box_size)
                else:
                    bb_output_k.append(bb_boxes["bb"][index_in_bb][index_bb])
                
        #     swap the track list to the new list.
        self.track_list_next_index = next_index_list

        return ellips_inputs_k, bb_output_k
        
    def find_cloest(self, x_k_pda, z_list):
        # print(z_list)
        if len(z_list) == 0:
            return -1
        dist = (x_k_pda[0,0] - z_list[0][0]) **2 + (x_k_pda[1,0] - z_list[0][1]) **2 
        ind = 0
        for i in range(len(z_list) - 1):
            dist_i = (x_k_pda[0,0] - z_list[i+1][0]) **2 + (x_k_pda[1,0] - z_list[i+1][1]) **2 
            if dist_i < dist:
                dist = dist_i
                ind = i + 1
        return ind
