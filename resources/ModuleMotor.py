
import numpy as np
import time
from model_def import *

class ModuleMotor:

	def __init__(self, state):

		self.state = state
		self.perf = Perf("ModuleMotor")

		# init components
		self.wpg_init()

	def perf(self):

		# performance test
		ti = time.time()
		for i in range(0, 50):
			self.tick()
		perf_report("ModuleMotor", ti)

	def tick(self):

		# performance
		self.perf.tick_i()

		# run components
		self.mpg()
		self.wpg()

		# performance
		self.perf.tick_f()

	def mpg(self):

		# zero kc_m pose
		self.state.kc_m.zeroPose()

		# apply external pushes
		for push in self.state.sig.pushes:
			self.state.kc_m.push(push)

		#x = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', self.state.pars.fovea_HEAD)
		#print x

		# virtual floor
		fovea_BODY = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', self.state.pars.fovea_HEAD)
		intrusion_through_floor_z = np.maximum(self.state.pars.virtual_floor_z - fovea_BODY[2], 0.0)
		push_vec_BODY = np.array([0.0, 0.0, intrusion_through_floor_z * 10.0])
		push_vec = self.state.kc_m.changeFrameRel('BODY', 'HEAD', push_vec_BODY)
		push = Push()
		push.frame = self.state.pars.frame_HEAD
		push.pos = self.state.pars.fovea_HEAD
		push.vec = push_vec
		self.state.kc_m.push(push)

		# recover state
		state = self.state.kc_m.getState()

		# write neck
		self.state.sig.neck_cmd = np.array(state[1])
		#self.state.sig.neck_cmd = np.array([0.0, 0.0, 0.0])

		# write wheels command velocity
		self.state.sig.cmd_vel[0] = state[0][0][0] * self.state.pars.fS_lo
		self.state.sig.cmd_vel[1] = state[0][0][1] * self.state.pars.fS_lo
		self.state.sig.cmd_vel[2] = state[0][1] * self.state.pars.fS_lo

		# add gui command velocity
		self.state.sig.cmd_vel += self.state.sig.cmd_vel_gui

	def wpg_init(self):

		self.wpg_T = 1.0 / self.state.pars.fW
		self.wpg_t = 0.0
		self.wpg_macon = self.state.sig.macon_lo

	def wpg(self):

		# get periodic time
		t = np.mod(self.state.time_t, self.wpg_T)

		# reset
		if t < self.wpg_t:
			self.wpg_macon.fill(0.0)
		self.wpg_t = t

		# max with current contact signal
		c = self.state.sig.macon_lo
		self.wpg_macon = np.maximum(self.wpg_macon, c)

		# nominal whisking range (move to parameters)
		theta_rcp = -np.pi / 3
		theta_min = -np.pi / 6
		theta_max = np.pi / 4
		theta_rng = theta_max - theta_min
		theta_rng2 = theta_rcp - theta_max

		# get theta target for each whisker
		x = np.sin(t * 2 * np.pi * self.state.pars.fW) * 0.5 + 0.5
		theta_cmd = theta_min + x * theta_rng

		# implement RCP
		theta_lim = theta_max + self.wpg_macon * theta_rng2
		theta_cmd = np.minimum(theta_cmd, theta_lim)

		# store
		self.state.sig.theta_cmd = theta_cmd



