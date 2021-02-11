from ttl.centralizedAgent import cenAgent
from ttl.effects import effects
import random
import copy
import csv, json
import os, sys
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
# plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Rectangle, FancyArrowPatch, Polygon
from scipy.stats.distributions import chi2


class Ttl(object):
	def __init__(self, bil=None, scenario=-1, frequency = 0.5):
		self.bil = bil
		# read sensor parameter
		path = os.getcwd()
		filename = os.path.join(path, 'data', 'demo_sensor_parameter.json')
		with open(filename) as json_file:
			data = json.load(json_file)

		self.sensor_para_list = data["sensors"]
		self.time_to_expand = 1.0
		self.dt = data["dt"]
		self.obstacles = data["obstacles"]
		# 2. Sequantial JPDA, centralized way
		self.traj = data["trajectory"]

		self.interpolation(frequency)
		self.step_num = len(self.traj)
		
		# 3. scenario number
		self.scenario = scenario  # available number: 1, 2

		# 4. policy for sensors
		self.policy = {}
		self.policy[0] = [[0, 0]] * self.step_num
		self.policy[2] = [[0, 0]] * self.step_num
		self.policy[1] = [[0, 0]] * self.step_num
		if self.scenario == 1:
			for i in range(16 * self.time_to_expand, 27 * self.time_to_expand):
				self.policy[1][i] = [-1.5 / self.time_to_expand, 3.5 / self.time_to_expand]
		elif self.scenario == 2:
			for i in range(10 * self.time_to_expand, 16 * self.time_to_expand):
				self.policy[1][i] = [-2 / self.time_to_expand, 4.6 / self.time_to_expand]
			for i in range(17 * self.time_to_expand, 26 * self.time_to_expand):
				self.policy[1][i] = [-0.625 / self.time_to_expand, 1.5 / self.time_to_expand]
			for i in range(27 * self.time_to_expand, 30 * self.time_to_expand):
				self.policy[1][i] = [1.66667 / self.time_to_expand, -4 / self.time_to_expand]

	def interpolation(self, frequency):
		self.time_to_expand = int(self.dt / (1 / frequency))
		new_traj = []
		for i in range(len(self.traj) - 1):
			x1 = self.traj[i]
			x2 = self.traj[i+1]
			dx = (x2[0] - x1[0]) / self.time_to_expand
			dy = (x2[1] - x1[1]) / self.time_to_expand
			for j in range(self.time_to_expand):
				new_traj.append([x1[0] + j * dx, x1[1] + j * dy])
		self.traj = new_traj
		self.dt = self.dt / self.time_to_expand

	def run(self):
		isMoving = True
		isObsDyn = False
		isRotate = False
		isFalseAlarm = False

		print("TTL sim starts")
		self.sim(isMoving, isObsDyn, isRotate, isFalseAlarm)
		# print("TTL sim ends, tracking results saved in data/obs.json")

	def sim(self, isMoving, isObsDyn, isRotate, isFalseAlarm):
		arrow_dia = 5
		P_G = 0.9
		gating_size = chi2.ppf(P_G, df = 2)
		time_set = np.linspace(self.dt, self.dt * self.step_num, self.step_num)
		
		# initialize agent in agentRecord
		obs = []
		# for film maker
		est = []
		agentPos = []
		observation_z = []

		centralized_fusor = cenAgent(self.sensor_para_list, self.dt, 
				isObsdyn=isObsDyn, isRotate=isRotate, isSimulation=True)
		centralized_fusor.tracker.ConfirmationThreshold = [3, 4]
		centralized_fusor.tracker.DeletionThreshold = [3, 8]

		for j in range(self.step_num):
			t = time_set[j]

			# true_k, noise_k = self.generate_obs(t, isFalseAlarm, isMoving)

			# true_target_set.append(true_k)
			# noise_set.append(noise_k)

			z_k = [self.traj[j]]
			# total_z.append(z_k)
			observation_z.append(z_k)

			ellips_inputs_k, bb_output_k, deletedTrackList = centralized_fusor.obs_update_callback(t, self.dt, z_k, [])
			
			buffer = {"t": t, "deletedTracks": deletedTrackList, "sensors": [], "tracks": []}
			seq_track_est_k = []
			for track in ellips_inputs_k:
				x = track.kf.x_k_k[0,0]
				y = track.kf.x_k_k[1,0]
				trackObj = {"ID": track.id, "pose": [x, y, 0]}
				P = track.kf.P_k_k.flatten().tolist()[0]
				# if track.id in trackRecord.keys():
				# 	trackRecord[track.id]["Datap"].append([t, x, y, 0])
				# else:
				# 	trackRecord[track.id] = {"Datap": [[t, x, y, 0]], "trackID": track.id}
				buffer["tracks"].append(trackObj)
				seq_track_est_k.append([x, y, P])
			est.append(seq_track_est_k)

			# move sensors
			if isMoving:
				# centralized_fusor.central_base_policy(ellips_inputs_k)
				# centralized_fusor.scenarioPolicy(t, self.scenario)
				centralized_fusor.v_list = [self.policy[0][j], self.policy[1][j], self.policy[2][j]]
				centralized_fusor.dynamics()

			# collect sensors data
			seq_sen_pos_k = []
			for i in range(centralized_fusor.sensor_num):
				x = centralized_fusor.sensor_para_list[i]["position"][0]
				y = centralized_fusor.sensor_para_list[i]["position"][1]
				theta = centralized_fusor.sensor_para_list[i]["position"][2]				
				X, Y = self.RectangleCorners(centralized_fusor.sensor_para_list[i]["position"], centralized_fusor.sensor_para_list[i]["shape"][1])				
				agentObj = {"ID": i, "pose": [x, y, theta], "FoV": [[X, Y]]}
				buffer["sensors"].append(agentObj)
				seq_sen_pos_k.append(centralized_fusor.sensor_para_list[i]["position"])
			obs.append(buffer)

			agentPos.append(copy.deepcopy(seq_sen_pos_k))
		path = os.getcwd()
		filename = os.path.join(path, "data", "obs_"+str(self.scenario)+".json")
		with open(filename, 'w') as outfiles:
			json.dump(obs, outfiles, indent="\t")
		
		# make video of the result
		global ax, fig
		fig = plt.figure()
		#fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
		ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
							xlim=(-5, 205), ylim=(-5, 205))

		real_point, = ax.plot([], [], 'ko', ms=2)
		# obs_list_anim = np.array(obs_list)
		estimationPT, = ax.plot([], [], 'r*', ms=2)

		timer = ax.text(0, 40, '', fontsize = 10)
		# lines stands for neighborings

		def init():

			"""initialize animation"""
			
			real_point.set_data([], [])
			estimationPT.set_data([], [])
			
			# add FoVs
			for i in range(len(self.sensor_para_list)):
				para = self.sensor_para_list[i]
				e = effects.make_rectangle(para["position"][0], para["position"][1], para["position"][2], para)
				ax.add_artist(e)

			# self.demo_connection(sensor_para_list, [], isInitPos = True)
			
			return real_point, estimationPT

		def animate(i):
			"""perform animation step"""
			
			#patches = []
			# ax.patches = []
			for obj in ax.findobj(match = Ellipse):
				obj.remove()
			
			for obj in ax.findobj(match = FancyArrowPatch):
				obj.remove()
			
			for obj in ax.findobj(match = Rectangle):
				
				if obj._width > 1:
					obj.remove()

			# add obstacles
			for obs in self.obstacles:
				xy = np.array(obs["xy"])
				color = obs["color"]
				rect = Polygon(xy, lw = 1, linestyle = '-',
							color=color)

				ax.add_artist(rect)
			# add tracks
			x, y = [], []
			for track in est[i]:
				mu = [track[0], track[1]]
				x.append(mu[0])
				y.append(mu[1])
				P = track[2]
				S = np.matrix(P).reshape(4,4)[0:2, 0:2]
				e = effects.make_ellipse(mu, S, gating_size, 'r')
				ax.add_artist(e)
			estimationPT.set_data(x, y)
			# add observations
			x , y = [], []
			for point in observation_z[i]:
				
				x.append(point[0])
				y.append(point[1])
			real_point.set_data(x, y)
			# add agent pos & FoV
			for j in range(len(agentPos[i])):
				rec_cen = [agentPos[i][j][0], agentPos[i][j][1]]
				# TODO theta will be derived from csv too
				# h = sensor_para_list[j]["shape"][1][1]
				# w = sensor_para_list[j]["shape"][1][0]
				# x, y = left_bot_point(seq_agent_pos[i][j][0], seq_agent_pos[i][j][1], theta, h, w)

				e = effects.make_rectangle(rec_cen[0], rec_cen[1], agentPos[i][j][2], self.sensor_para_list[j])
				ax.add_artist(e)
				
				p2 = effects.vector(agentPos[i][j], arrow_dia)

				e = FancyArrowPatch((agentPos[i][j][0], agentPos[i][j][1]), 
							(p2[0], p2[1]),
							arrowstyle='->',
							linewidth=2,
							color=self.sensor_para_list[j]["color"])
				ax.add_artist(e)
			

			# timer.set_text("t = %s" % (self.dt*i + self.dt))

			return real_point, estimationPT, timer

		ani = animation.FuncAnimation(fig, animate, frames=len(agentPos),
									interval=10, blit=True, init_func=init, repeat = False)
		
		filename = os.path.join(path, "data", 'Jan22_demo_'+str(self.scenario)+'_Visual.mp4')
		ani.save(filename, fps=5)
		plt.close()
		print("TTL sim ends, tracking results saved in", filename)

	def RotationMatrix(self, theta):
		return np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

	def RectangleCorners(self, pos, shape):
		# given rectangle's center position + angle, return the 4 corner's x and y
		x, y, theta = pos
		position = np.array([x, y])
		width, height = shape
		X, Y = [], []
		vector1 = np.array([width, height]) * 0.5
		vector2 = np.array([-width, height]) * 0.5
		vector3 = np.array([width, -height]) * 0.5
		vector4 = np.array([-width, -height]) * 0.5
		vectorlist = [vector1, vector2, vector3,vector4]
		for vector in vectorlist:
			corner = np.dot(self.RotationMatrix(theta), vector) + position
			X.append(corner[0,0])
			Y.append(corner[0,1])
		return X, Y

	def generate_obs(self, t, isFalseAlarm, isMoving):

		z_k = []
		point = [40 - 2.2 * t, 20]
		point = [max(-25, 40 - 1.6 * t), 20]
		z_k.append(point)
		# 3. some noises
		noise_k = []
		# if isFalseAlarm:
		# 	for i in range(random.randint(1, 5)):
		# 		ran_point = [40* random.random() -20, 40* random.random()-20]
		# 		noise_k.append(ran_point)
		return z_k, noise_k
