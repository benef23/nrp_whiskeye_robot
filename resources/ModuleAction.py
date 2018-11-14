
import numpy as np
from model_def import *
import time

class ActionMull:

	def __init__(self, state):

		self.state = state
		self.t_start = None

	def tick(self, inh):

		# handle start/stop
		if self.t_start is None:
			if inh == 0.0:
				self.start()
		else:
			if inh > 0.0:
				self.stop()

		# default (null) push
		push = Push()

		# if active
		if not self.t_start is None:

			# priority for ongoing action
			pri = self.state.pars.pri_idle

		else:

			# priority at idle
			pri = self.state.pars.pri_idle

		# ok
		return (pri, push)

	def start(self):

		# store start time
		self.t_start = self.state.time_t
		print "mull"

	def stop(self):

		# clear start time
		self.t_start = None
		print "(end) mull"

class ActionOrient:

	def __init__(self, state):

		self.state = state
		self.t_start = None
		self.t_action = state.pars.orient_min_time
		self.dfov = None
		self.dshimmy = None

	def tick(self, inh):

		# handle start/stop
		if self.t_start is None:
			if inh == 0.0:
				self.start()
		else:
			if inh > 0.0:
				self.stop()

		# default (null) push
		push = Push()

		# if active
		if not self.t_start is None:

			# get action time
			t = self.state.time_t - self.t_start
			t_norm = np.minimum(t / self.t_action, 1.0)

			# get action modifiers
			s = np.sin(t_norm * np.pi)
			mod_dfov = 2 * s * s
			s = np.sin(2.0 * t_norm * np.pi)
			mod_shimmy = -s

			#print t_norm, mod_dfov, mod_shimmy

			# if action finished, bid with zero priority to be deselected
			if t >= self.t_action:
				pri = 0.0

			# if action ongoing
			else:

				# send push
				push.frame = self.state.pars.frame_HEAD
				push.pos = self.state.pars.fovea_HEAD
				push.vec = self.dfov * mod_dfov + self.dshimmy * mod_shimmy

				# priority for ongoing action
				pri = self.state.pars.pri_ongoing

		else:

			# priority is based on centroid magnitude
			pri = self.state.sig.focus.max.clip(max=1.0)

		# ok
		return (pri, push)

	def start(self):

		# store start time
		self.t_start = self.state.time_t

		# compute required foveal vector
		dfov = self.state.sig.focus.pos - self.state.pars.fovea_HEAD

		# compute orient distance
		dist = np.sqrt((dfov*dfov).sum())

		# compute orient minimum time
		self.t_action = np.maximum(dist / self.state.pars.orient_max_speed, self.state.pars.orient_min_time)

		# add shimmy
		self.dshimmy = np.array([self.state.pars.orient_shimmy_speed, 0.0, 0.0])

		# report
		print "orient", dfov, dist, self.t_action

		# command completes that vector in given time
		self.dfov = dfov / self.t_action

	def stop(self):

		# clear start time
		self.t_start = None
		print "(end) orient"

class BasalGanglia:

	def __init__(self, state, nchan):

		self.state = state
		self.nchan = nchan
		self.inh = np.ones(nchan)

	def tick(self, pri):

		i_bak = np.argmin(self.inh)
		i = np.argmax(pri)
		self.inh = np.ones(self.nchan)
		self.inh[i] = 0.0

		# report
		if not i == i_bak:
			print i_bak, "->", i

class ModuleAction:

	def __init__(self, state):

		self.state = state
		self.perf = Perf("ModuleAction")

		self.nchan = 2
		self.z = np.zeros(2)

		# init actions
		self.actions = (ActionMull(state), ActionOrient(state))

		# init selection
		self.bg = BasalGanglia(state, self.nchan)

	def perf(self):

		# performance test
		ti = time.time()
		for i in range(0, 50):
			self.tick()
		perf_report("ModuleAction", ti)

	def tick(self):

		# performance
		self.perf.tick_i()

		# collate input
		pris = self.z

		# get bg output
		inh = self.bg.inh
		pushes = [None] * self.nchan

		# run actions
		for i in range(0, len(self.actions)):
			[pris[i], pushes[i]] = self.actions[i].tick(inh[i])

		# run selection
		#print pris
		self.bg.tick(pris)

		# store pushes
		self.state.sig.pushes = pushes

		# performance
		self.perf.tick_f()



