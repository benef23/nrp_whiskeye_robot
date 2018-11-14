
import numpy as np
from scipy.ndimage import gaussian_filter
import math
import matplotlib.pyplot as plt

from model_support import *
import time
import copy

class ModuleSpatial:

	def __init__(self, state):

		self.state = state
		self.perf = Perf("ModuleSpatial")

		# precompute
		self.salience_macon_width_recip = 1.0 / self.state.pars.salience_macon_width

		# create spatial map
		[self.mappos, self.mapval] = self.state.pars.headspace.createMap()

		# create spatial noise
		#self.mapfilt = np.zeros((3, 3, 3))
		#self.mapfilt[1][1][1] = 1.0
		self.mapnoise = np.zeros(self.mapval.shape)

		# precompute spatial noise weighting as a portion of the surface
		# of a sphere centred on the origin of HEAD
		cen = copy.copy(self.state.pars.fovea_HEAD)
		cen[0] -= self.state.pars.sausage_cen_x_offset
		self.sausage_weight = np.zeros(self.mapval.shape)
		shape = self.state.pars.headspace.shape
		for xi in range(0, shape[0]):
			for yi in range(0, shape[1]):
				for zi in range(0, shape[2]):
					d = self.mappos[xi][yi][zi] - cen
					d[2] = 0 # ignore height, we'll do height above floor using run-time robot configuration
					r = np.sqrt((d*d).sum()) # radius of point from sphere centre
					t = (r - self.state.pars.sausage_rad) / self.state.pars.sausage_var_shell
					bias_wall = np.exp(-t*t)
					bias_forward = np.minimum(d[0] / self.state.pars.sausage_rad, 1.0)
					if bias_forward < 0.0:
						bias_forward = 0.0
					mag = bias_wall * bias_forward * self.state.pars.noise_magnitude
					self.sausage_weight[xi][yi][zi] = mag

		# pre-compute exponential "range" in units of map cells
		for i in range(0, 100):
			x = self.state.pars.headspace.step * (i + 1)
			q = x / self.state.pars.salience_macon_width
			z = math.exp(-q*q)
			if z < self.state.pars.salience_discard:
				self.headspace_L = i
				break

		# pre-compute time parameters
		self.salience_decay_lambda = tau2lambda(self.state.pars.salience_decay_tau, self.state.pars.fS_lo)
		self.noise_decay_lambda = tau2lambda(self.state.pars.noise_decay_tau, self.state.pars.fS_lo)

	def remap(self, mapslice):

		return np.transpose(mapslice)

	def perf(self):

		# performance test
		ti = time.time()
		for i in range(0, 50):
			self.tick()
		perf_report("ModuleSpatial", ti)

	def tick(self):

		# performance
		self.perf.tick_i()

		# get floor in HEAD
		f0 = np.array([0.0, 0.0, 0.0])
		f1 = np.array([0.0, 0.0, 1.0])
		f0_HEAD = self.state.kc_m.changeFrameAbs('BODY', 'HEAD', f0)
		f1_HEAD = self.state.kc_m.changeFrameAbs('BODY', 'HEAD', f1)

		# build headspace map of weight by height above floor
		x = np.array([0.0, 0.0, 0.0])
		x_BODY = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', x)
		h0 = x_BODY[2]
		x = np.array([1.0, 0.0, 0.0])
		x_BODY = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', x)
		hx = x_BODY[2] - h0
		x = np.array([0.0, 1.0, 0.0])
		x_BODY = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', x)
		hy = x_BODY[2] - h0
		x = np.array([0.0, 0.0, 1.0])
		x_BODY = self.state.kc_m.changeFrameAbs('HEAD', 'BODY', x)
		hz = x_BODY[2] - h0
		heightAboveFloor = h0 + self.mappos[:,:,:,0]*hx + self.mappos[:,:,:,1]*hy + self.mappos[:,:,:,2]*hz

		# finalise
		heightAboveFloor -= self.state.pars.sausage_heightAboveFloor
		weight_var = heightAboveFloor / self.state.pars.sausage_var_height
		weight = np.exp(-weight_var * weight_var)

		# merge with pre-computed sausage bias
		weight *= self.sausage_weight

		# additive noise
		self.mapnoise += np.random.standard_normal(self.mapval.shape)
		self.mapnoise = gaussian_filter(self.mapnoise, self.state.pars.noise_spatial_sigma)
		self.mapnoise *= self.noise_decay_lambda
		#weighted_noise = self.mapnoise * self.sausage_weight
		#self.mapval += self.mapnoise * self.state.pars.noise_magnitude
		self.mapval += self.mapnoise * weight
		#self.mapval = copy.copy(weight * 5.0)

		# dynamics
		#self.mapval = scipy.ndimage.filters.convolve(self.mapval, self.mapfilt)

		#self.mapval = gaussian_filter(self.mapval, 2.0)
		self.mapval *= self.salience_decay_lambda

		# for each whisker
		for row in self.state.pars.rows:
			for col in self.state.pars.cols:
				macon = self.state.sig.macon_lo[row][col]
				if macon == 0.0:
					continue
				pos = self.state.sig.whisker_tip_pos[row][col]
				self.inject(pos, macon)

		# compute threshold
		thresh = np.max(self.mapval) * 0.2

		# compute summary
		#thresh = 0.2
		over_thresh = (self.mapval - thresh).clip(min=0)
		over_thresh_4d = np.tile(np.expand_dims(over_thresh, 4), (1, 1, 1, 3))
		#q = over_thresh_4d.shape
		self.state.sig.focus.mag = np.sum(over_thresh)
		self.state.sig.focus.max = np.max(over_thresh)
		if self.state.sig.focus.mag > 0:
			self.state.sig.focus.pos = np.sum(over_thresh_4d * self.mappos, axis=(0, 1, 2)) / self.state.sig.focus.mag
		else:
			self.state.sig.focus.pos = np.array([0, 0, 0])

#		print self.state.sig.focus.mag, self.state.sig.focus.max

		# debug (show active rows)
		"""
		mn = []
		for row in self.state.pars.rows:
			mn.append(int(np.mean(self.state.sig.macon_lo[row])*100))
		print mn
		"""

		# publish
		self.state.sig.mapval = self.mapval;

		# performance
		self.perf.tick_f()

	def inject(self, pos, mag):

		# run dynamics
		lam = self.salience_decay_lambda

		# get range of headspace in which inject might have non-negligible effect
		[xii, yii, zii] = self.state.pars.headspace.getRanges(pos, self.headspace_L)

		# create map indexing object for that range
		ind = np.ix_(xii, yii, zii)

		# inject into that range
		mappos = self.mappos[ind]
		dpos = pos - mappos
		x = np.linalg.norm(dpos, axis=3)
		q = x * self.salience_macon_width_recip
		q2 = -q * q
		z = np.exp(q2) * mag
		self.mapval[ind] += z

		"""
		# for each cell in that range
		for xi in xii:
			for yi in yii:
				for zi in zii:

					# compute exponential
					mappos = self.mappos[xi, yi, zi]
					dpos = pos - mappos
					x = np.linalg.norm(dpos)
					q = x / self.state.pars.salience_macon_width
					z = math.exp(-q*q)

					# inject using magnitude
					self.mapval[xi, yi, zi] += z * mag
		"""


