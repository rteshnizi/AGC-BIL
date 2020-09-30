import json
import numpy as np
import os, sys

agent_id_list = [0, 1, 2]
dm = 20 * np.sqrt(2)
dc = 40
sensor_para_list = []
dt = 0.1
d0 = 9

shape = ["rectangle", [20,20]]  # width, height
sensor_para = {"position": [0, 0, 0], # [x, y, angle]
        "shape": shape,
        "quality": 1.0, 
        "d0": d0, 
        "dc": dc,           # communication range between 2 sensors
        "dm": dm,           # distance for chasing a target
        "v": 1.0,           # scalar for velocity
        "r": 0.5,           # diagonal element in R matrix covariance
        "color": 'r'}
sensor_para_list.append(sensor_para)

shape = ["rectangle", [20,20]]  
sensor_para = {"position": [10, 10, 0], 
        "shape": shape,
        "quality": 1.0,   
        "d0": d0, 
        "dc": dc,
        "dm": dm,
        "v": 1.0,
        "r": 0.5,
        "color": 'g'}
sensor_para_list.append(sensor_para)

shape = ["rectangle", [20, 20]]  
sensor_para = {"position": [-10, -10, 0], 
        "shape": shape,
        "quality": 1.0,  
        "dc": dc,         
        "d0": d0, 
        "dm": dm,
        "v": 1.0,
        "r": 0.5,
        "color": 'peru'}
sensor_para_list.append(sensor_para)

parameters = {}

parameters["sensors"] = sensor_para_list
parameters["dt"] = dt


path = os.getcwd()
filename = os.path.join(path, "data", "sim_parameter.json")
with open(filename, 'w') as outfiles:
    json.dump(parameters, outfiles, indent=4)