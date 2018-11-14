
import numpy as np
import math
import time

def toString(s):
	return " ".join(str(elm) for elm in s)

def tau2lambda(tau, fS):
	return math.exp(-1.0/(tau * fS))

def create_empty_array_of_shape(shape):
	if shape: return [create_empty_array_of_shape(shape[1:]) for i in xrange(shape[0])]

def perf_report(name, ti):
	tf = time.time()
	print name, "perf/sec:", "%.3f" % (tf - ti)

def stop():
	raise ValueError("stop")

class ArrayFilter:

	def __init__(self, shape, b, a):

		self.shape = shape
		self.shape_prepend = shape[:]
		self.shape_prepend.insert(0, 1)
		self.b = b
		self.a = a
		self.a_unity = False

		if len(a) < len(b) and len(a) == 1:
			self.a = np.array([0.0] * len(b))
			self.a[0] = 1.0
			self.a_unity = True

		if not len(self.a) == len(self.b):
			raise ValueError("bad filter parameters")

		if not self.a[0] == 1.0:
			raise ValueError("bad filter parameters")

		self.N = len(b)
		self.L = len(b) - 1
		if self.L < 1:
			raise ValueError("bad filter parameters")

		shape_hist = shape[:]
		shape_hist.insert(0, self.N)
		self.X = np.zeros(shape_hist)
		self.Y = np.zeros(shape_hist)

		self.ndim = len(self.shape)

		# special case for scalar
		self.scalar = self.ndim == 1 and self.shape[0] == 1
		if self.scalar:
			self.X = np.zeros(self.N)
			self.Y = np.zeros(self.N)
			#self.b = np.zeros(self.N)
			#self.b[self.N-1] = 2.0
			#self.a = np.zeros(self.N)
			#self.a[0] = 1.0

	def run(self, x):

		if self.ndim >= 2:

			# stuff history
			self.X = np.concatenate((np.reshape(x, self.shape_prepend), self.X[0:self.N-1, :, :]))
			self.Y = np.concatenate((np.reshape(x, self.shape_prepend), self.Y[0:self.N-1, :, :]))
			# NB we stuff Y with x, just to do the alignment, and then correct it below

			# compute output
			y = x * self.b[0]
			for i in range(1, self.N):
				y += self.b[i] * self.X[i, :, :]
				y -= self.a[i] * self.Y[i, :, :]

			# correct y stuff
			self.Y[0, :, :] = y

		if self.scalar and False:

			# roll history
			self.X = np.roll(self.X, 1)
			self.Y = np.roll(self.Y, 1)

			# compute output
			y = x * self.b[0]
			y += np.dot(self.b[1:], self.X[1:])
			y -= np.dot(self.a[1:], self.Y[1:])

			# stuff history
			self.X[0] = x
			self.Y[0] = y

		if self.scalar:

			if self.a_unity:

				# stuff history
				self.X = np.concatenate(([x], self.X[0:self.N-1]))

				# compute output
				y = np.dot(self.b, self.X)

			else:

				# stuff history
				self.X = np.concatenate(([x], self.X[0:self.N-1]))
				self.Y = np.concatenate(([x], self.Y[0:self.N-1]))
				# NB we stuff Y with x, just to do the alignment, and then correct it below

				# compute output
				y = np.dot(self.b, self.X)
				y -= np.dot(self.a[1:], self.Y[1:])

				# correct y stuff
				self.Y[0] = y

		# return output
		return y

	def run_simple(self, x):

		# stuff history
		self.X = np.concatenate(([x], self.X[0:self.N-1]))

		# compute output
		y = np.dot(self.b, self.X)

		# return output
		return y

	def run_vector(self, x):

		y = x.copy()
		for i in range(0, len(x)):
			self.X = np.concatenate(([x[i]], self.X[0:self.N-1]))
			y[i] = np.dot(self.b, self.X)
		return y

class Perf:

	def __init__(self, name):

		self.name = name
		self.count = 0
		self.limit = 50
		self.t = None
		self.dt = 0.0
		self.dts = None

		# active? report perf information about all modules?
		self.active = False

	def detail(self):

		self.dts = []

	def tick_i(self):

		self.t = time.time()

	def tick_f(self):

		# active?
		if not self.active:
			return

		t = time.time()
		dt = t - self.t

		if not self.dts is None:
			self.dts.append(dt)

		self.dt += dt
		self.count += 1

		if self.count == self.limit:
			print self.name, "%.3f" % (self.dt)
			self.count = 0
			self.dt = 0.0
			if not self.dts is None:
				print "    ", np.round(np.array(self.dts) * 1000.0), np.max(np.array(self.dts))
				self.dts = []



