
import numpy as np
from kc_model import *



class Focus:

	def __init__(self):

		self.pos = np.array([0.0, 0.0, 0.0])
		self.mag = np.array(0.0)
		self.max = np.array(0.0)

class Push:

	def __init__(self):

		self.is_velocity = True # set to False for a position change
		self.frame = -1
		self.pos = np.array([0.0, 0.0, 0.0])
		self.vec = np.array([0.0, 0.0, 0.0])

	def resolve(self):

		# if is_velocity, convert to position
		if self.is_velocity:
			push = Push()
			push.is_velocity = False
			push.frame = self.frame
			push.pos = self.pos
			push.vec = self.vec * 0.02
			return push
		else:
			return self

class SystemSig:

	def __init__(self, pars):

		# zero input
		z = np.zeros(pars.shape_hi)
		z_2 = np.zeros(pars.shape_hi_2)
		z_lo = np.zeros(pars.shape_lo)

		#  platform outputs
		self.body = np.array([0.0, 0.0, 0.0])
		self.neck = np.array([0.0, 0.0, 0.0])
		self.theta = z.copy()
		self.xy = z_2.copy()
		self.physical = False

		# filtered
		self.theta_f = z.copy()
		self.xy_q = z_2.copy()
		self.xy_f = z_2.copy()
		self.macon = z.copy()

		# downsampled
		self.macon_lo = z_lo.copy()
		self.theta_lo = z_lo.copy()

		# run-time whisker tip positions
		sz = pars.shape_lo[:]
		sz.append(3)
		self.whisker_tip_pos = np.zeros(sz)

		# sc
		self.focus = Focus()

		# action
		self.pushes = []

		# gui inputs
		self.cmd_vel_gui = np.array([0.0, 0.0, 0.0])

		# platform inputs
		self.neck_cmd = np.array([WHISKY_LIFT_INI_RAD, WHISKY_PITCH_INI_RAD, WHISKY_YAW_INI_RAD])
		self.theta_cmd = np.zeros(pars.shape_lo)
		self.cmd_vel = np.array([0.0, 0.0, 0.0])

		# internals outputs
		self.mapval = None

class SystemState:

	def __init__(self):

		self.pars = SystemPars()
		self.sig = SystemSig(self.pars)

		# time
		self.time_n = 0
		self.time_t = 0.0

		# kinematic chains
		self.kc_s = kc_whiskeye()
		self.kc_m = kc_whiskeye()
		self.kc_w = kc_whiskers(self.pars)

		# procedural
		self.cancel = False

	def tick(self):

		# update time
		self.time_n += 1
		self.time_t += 0.02




