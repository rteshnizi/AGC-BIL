from ttl.centralizedAgent import cenAgent
import random
import copy
import csv, json
import os, sys
import numpy as np

class Ttl(object):
	def __init__(self, bil=None):
		self.bil = bil
		# read sensor parameter
		path = os.getcwd()
		filename = os.path.join(path, 'data', 'sim_parameter.json')
		with open(filename) as json_file:
			data = json.load(json_file)

		self.sensor_para_list = data["sensors"]
		self.dt = data["dt"]

		# 2. Sequantial JPDA, centralized way

		self.step_num = data["run_num"]

	def run(self):
		isMoving = False
		isObsDyn = False
		isRotate = False
		isFalseAlarm = False

		print("TTL sim starts")
		self.sim(isMoving, isObsDyn, isRotate, isFalseAlarm)
		print("TTL sim ends, tracking results saved in data/obs.json")

	def sim(self, isMoving, isObsDyn, isRotate, isFalseAlarm):
		time_set = np.linspace(self.dt, self.dt * self.step_num, self.step_num)
		
		# initialize agent in agentRecord
		obs = []
		centralized_fusor = cenAgent(self.sensor_para_list, self.dt, isObsdyn=isObsDyn, isRotate=isRotate, isSimulation=True)

		for t in time_set:

			true_k, noise_k = self.generate_obs(t, isFalseAlarm, isMoving)
			# true_target_set.append(true_k)
			# noise_set.append(noise_k)
			z_k = true_k + noise_k
			# total_z.append(z_k)

			ellips_inputs_k, bb_output_k, deletedTrackList = centralized_fusor.obs_update_callback(t, self.dt, z_k, [])
			
			buffer = {"t": t, "deletedTracks": deletedTrackList, "sensors": [], "tracks": []}

			for track in ellips_inputs_k:
				x = track.kf.x_k_k[0,0]
				y = track.kf.x_k_k[1,0]
				trackObj = {"ID": track.id, "pose": [x, y, 0]}
				# P = track.kf.P_k_k.flatten().tolist()[0]
				# if track.id in trackRecord.keys():
				# 	trackRecord[track.id]["Datap"].append([t, x, y, 0])
				# else:
				# 	trackRecord[track.id] = {"Datap": [[t, x, y, 0]], "trackID": track.id}
				buffer["tracks"].append(trackObj)
				# seq_track_est_k.append([x, y, P])

			# move sensors
			if isMoving:
				centralized_fusor.central_base_policy(ellips_inputs_k)
				centralized_fusor.dynamics()

			# collect sensors data

			for i in range(centralized_fusor.sensor_num):
				x = centralized_fusor.sensor_para_list[i]["position"][0]
				y = centralized_fusor.sensor_para_list[i]["position"][1]
				theta = centralized_fusor.sensor_para_list[i]["position"][2]				
				X, Y = self.RectangleCorners(centralized_fusor.sensor_para_list[i]["position"], centralized_fusor.sensor_para_list[i]["shape"][1])				
				agentObj = {"ID": i, "pose": [x, y, theta], "FoV": [[X, Y]]}
				buffer["sensors"].append(agentObj)
			obs.append(buffer)

		path = os.getcwd()
		filename = os.path.join(path, "data", "obs.json")
		with open(filename, 'w') as outfiles:
			json.dump(obs, outfiles, indent="\t")

	def RotationMatrix(self, theta):
		return np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

	def RectangleCorners(self, pos, shape):
		# given rectangle's center position + angle, return the 4 corner's x and y
		x, y, theta = pos
		width, height = shape
		X, Y = [], []
		vector1 = np.array([width, height]) * 0.5
		vector2 = np.array([-width, height]) * 0.5
		vector3 = np.array([width, -height]) * 0.5
		vector4 = np.array([-width, -height]) * 0.5
		vectorlist = [vector1, vector2, vector3,vector4]
		for vector in vectorlist:
			corner = np.dot(self.RotationMatrix(theta), vector)
			X.append(corner[0,0])
			Y.append(corner[0,1])
		return X, Y

	def generate_obs(self, t, isFalseAlarm, isMoving):

		z_k = []

		if not isMoving:

			x = 20 - 40.0 / 50 * t
			y = 20 - 40.0 / 50 * t
			z_k.append([x, y])

			x = 10 - 30.0 / 50 * t
			y = 20 - 30.0 / 50 * t
			z_k.append([x, y])

			x = - 10 + 30.0 / 50 * t
			y = - 20 + 30.0 / 50 * t
			z_k.append([x, y])

		else:

			beta = np.pi/100

			r = 10

			# circles
			x = 10 + r*np.cos(t*beta + np.pi)
			y = 10 + r*np.sin(t*beta + np.pi)
			target = [x, y]
			# plot_track(target)
			z_k.append(target)

			# circles
			x = -10 + r*np.cos(t*beta)
			y = 10 + r*np.sin(t*beta)
			target = [x, y]
			# plot_track(target)
			z_k.append(target)

			# circles
			x = -10 + r*np.cos(t*beta - np.pi/4)
			y = -10 + r*np.sin(t*beta - np.pi/4)
			target = [x, y]
			# plot_track(target)
			z_k.append(target)

			# circles
			x = 10 + r*np.cos(t*beta + np.pi/2)
			y = -10 + r*np.sin(t*beta + np.pi/2)
			target = [x, y]
			# plot_track(target)
			z_k.append(target)

		# 3. some noises
		noise_k = []
		if isFalseAlarm:
			for i in range(random.randint(1, 5)):
				ran_point = [40* random.random() -20, 40* random.random()-20]
				noise_k.append(ran_point)
		return z_k, noise_k
