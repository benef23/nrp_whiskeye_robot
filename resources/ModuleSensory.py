
import numpy as np
from model_support import *
from kc_model import *
import time

class ModuleSensory:

	def __init__(self, state):

		self.state = state
		self.perf = Perf("ModuleSensory")

		# init components
		self.reaff_filter_init()
		self.macro_filter_init()
		self.contact_init()

	def tick(self):

		# performance
		self.perf.tick_i()

		# run components
		self.theta_filter()
		self.reaff_filter()
		self.macro_filter()
		self.micro_filter()
		self.contact()
		self.downsample()
		self.configure()

		# performance
		self.perf.tick_f()

	def perf_breakdown(self):

		N = 200

		# performance test
		ti = time.time()
		for i in range(0, N):
			self.tick()
		perf_report("ModuleSensory", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.theta_filter()
		perf_report("\ttheta_filter", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.reaff_filter()
		perf_report("\treaff_filter", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.macro_filter()
		perf_report("\tmacro_filter", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.micro_filter()
		perf_report("\tmicro_filter", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.contact()
		perf_report("\tcontact", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.downsample()
		perf_report("\tdownsample", ti)

		# sub perf test
		ti = time.time()
		for i in range(0, N):
			self.configure()
		perf_report("\tconfigure", ti)

	def theta_filter(self):

		# theta filter currently is pass-through
		self.state.sig.theta_f = self.state.sig.theta

	def reaff_filter_init(self):

		self.reaff = create_empty_array_of_shape(self.state.pars.shape_lo_2)

		# filter definition
		for row in self.state.pars.rows:
			for col in self.state.pars.cols:
				for dim in self.state.pars.dims:
					b = self.state.pars.reaff[row, col, dim, :]
					# b is the filter to be applied to the dtheta signal, but
					# we'll be processing theta so let's add a derivative term
					b1 = np.concatenate((b, [0.0]))
					b2 = np.concatenate(([0.0], b))
					b = b1 - b2
					f = ArrayFilter([1], b, [1])
					self.reaff[row][col][dim] = f

	def reaff_filter(self):

		# if the robot is physical
		if self.state.sig.physical:

			# run each filter individually
			for row in self.state.pars.rows:
				for col in self.state.pars.cols:
					f = self.reaff[row][col]
					"""
					theta = self.state.sig.theta_f[:, row, col]
					x = self.state.sig.xy[:, row, col, 0]
					y = self.state.sig.xy[:, row, col, 1]
					self.state.sig.xy_q[:, row, col, 0] = x - f[0].run_vector(theta)
					self.state.sig.xy_q[:, row, col, 1] = y - f[1].run_vector(theta)

					TODO: move to faster code, below
					"""
					for i in range(0, self.state.pars.fS_scale):
						theta = self.state.sig.theta_f[i, row, col]
						x = self.state.sig.xy[i, row, col, 0]
						y = self.state.sig.xy[i, row, col, 1]
						self.state.sig.xy_q[i, row, col, 0] = x - f[0].run_simple(theta)
						self.state.sig.xy_q[i, row, col, 1] = y - f[1].run_simple(theta)

		else:
			self.state.sig.xy_q = self.state.sig.xy

	def macro_filter_init(self):

		# filter definition
		"""
			fS_hi = 500
			sensor_cutoff_freq = 50
			half_sample_rate = fS_hi / 2
			[b, a] = butter(3, sensor_cutoff_freq / half_sample_rate)
		"""
		b = [0.0181, 0.0543, 0.0543, 0.0181]
		a = [1.0000, -1.7600, 1.1829, -0.2781]
		#self.f_x = ArrayFilter(self.state.pars.shape_lo, b, a)
		#self.f_y = ArrayFilter(self.state.pars.shape_lo, b, a)
		self.f_xy = ArrayFilter(self.state.pars.shape_lo_2, b, a)

	def zero_whisker(self, row, col):

		for i in range(0, 10):
			self.state.sig.xy_q[i][row][col][0] = 0.0
			self.state.sig.xy_q[i][row][col][1] = 0.0

	def macro_filter(self):

		"""
		self.zero_whisker(2, 0)
		self.zero_whisker(2, 1)
		self.zero_whisker(2, 2)
		self.zero_whisker(2, 3)
		"""

		self.state.sig.xy_f = self.state.sig.xy_q

		# apply filter
		"""
		for i in range(0, self.state.pars.fS_scale):
			self.state.sig.xy_f[i, :, :, :] = self.f_xy.run(self.state.sig.xy_q[i, :, :, :])
			#self.state.sig.xy_f[i, :, :, 1] = self.f_y.run(self.state.sig.xy_q[i, :, :, 1])
		"""

	def micro_filter(self):

		# not implemented yet, use pass-through
		pass

		"""

%% MICRO FILTER

% highpass filter to remove "lingering" signal that is a
% feature of the microvibrissae

state = [];
fC = 5;
[b, a] = butter(1, fC/(pars.time.fS_hi/2), 'high');
state.b = single(b);
state.a = single(a);
cls = 'dev/whiskerbots/generic/filter';
sys = sys.addprocess('microfilter', cls, pars.time.fS_hi, state);

% expose
sys = sys.expose('microfilter<micro', 'micro');

		"""

	def contact_init(self):

		self.contact_gain = self.state.pars.contact_gain

	def contact(self):

		# extract input
		x = self.state.sig.xy_f[:, :, :, 0];
		y = self.state.sig.xy_f[:, :, :, 1];

		# apply operation
		l = np.sqrt(x*x+y*y)
		c = (l - self.state.pars.contact_thresh) * self.contact_gain
		c = np.clip(c, 0.0, 1.0)

		# store output
		self.state.sig.macon = c

		"""
% lay in threshold and gain
dims = pars.platform.config.dims.macro;
state.macro.threshold = single(repmat(threshold', [1 dims(2)]));
state.macro.gain = single(repmat(gain', [1 dims(2)]));

% micro threshold/gain

%threshold = 0.05;
threshold = 0.1;
gain = 5;
dims = pars.platform.config.dims.micro;
state.micro.threshold = single(repmat(threshold, dims(1), dims(2)));
state.micro.gain = single(repmat(gain, dims(1), dims(2)));

% add process
sys = sys.addprocess('contact', 'dev/whiskerbots/condition/contact', pars.time.fS_hi, state);

% add links
sys = sys.link('macrofilter>macro', 'contact', 0);
sys = sys.link('microfilter>micro', 'contact', 0);


			//	loop through macro
			UINT32 W = platform.getMacroCount();
			for (UINT32 w=0; w<W; w++)
			{
				WFLOAT x = macro[w].x;
				WFLOAT y = macro[w].y;
				WFLOAT l = sqrt(x*x+y*y);
				WFLOAT c = (l - macro_threshold[w]) * macro_gain[w];
				macon_out[w] = constrain(c, 0.0f, 1.0f);
			}

			//	do micro, if attached
			if (input.micro.IS_ATTACHED())
			{
				//	access
				WB_MICRO* micro = (WB_MICRO*) input.micro.getContent();
				WB_CONTACT* micon_out = (WB_CONTACT*) output.micon.getContent();

				//	loop
				W = platform.getMicroCount();
				for (UINT32 w=0; w<W; w++)
				{
					WFLOAT x = micro[w].x;
					WFLOAT y = micro[w].y;
					WFLOAT l = sqrt(x*x+y*y);
					WFLOAT c = (l - micro_threshold[w]) * micro_gain[w];
					micon_out[w] = constrain(c, 0.0f, 1.0f);
				}
			}


		"""

	def downsample(self):

		# downsample by simple averaging
		self.state.sig.theta_lo = np.mean(self.state.sig.theta_f, 0)
		self.state.sig.macon_lo = np.mean(self.state.sig.macon, 0)

		"""



%% DOWNSAMPLE THETA/MACON/MICON

%% anti-aliasing filter

state = [];
[b, a] = butter(3, (pars.time.fS_lo/2)/(pars.time.fS_hi/2));
state.b = single(b);
state.a = single(a);
cls = 'dev/whiskerbots/generic/filter';
sys = sys.addprocess('antialias', cls, pars.time.fS_hi, state);

%% downsample data

cls = 'std/2009/resample/numeric';
sys = sys.addprocess('downsample', cls, pars.time.fS_lo);

sys = sys.link('thetafilter>theta', 'antialias', 0);
sys = sys.link('contact>macon', 'antialias', 0);
sys = sys.link('contact>micon', 'antialias', 0);
sys = sys.link('antialias>theta', 'downsample', 0);
sys = sys.link('antialias>macon', 'downsample', 0);
sys = sys.link('antialias>micon', 'downsample', 0);


		"""

	def configure(self):

		# update kc_s
		self.state.kc_s.setConfig(self.state.sig.neck)

		# update kc_w
		for row in self.state.pars.rows:
			for col in self.state.pars.cols:

				# lay theta (whisker angle) in to kc_w
				theta = self.state.sig.theta_lo[row][col]
				self.state.kc_w[row][col].link[5].angle = theta

				# compute whisker tip location in HEAD
				# NB: at time of writing, WHISKER has zero rotation wrt ROW if
				# theta is zero (i.e. the whisker is straight out), which means
				# the whisker tip is at x=0, y=0, z=l. we could easily add 90
				# degrees to theta, and place the tip at x=l, y=0, z=0. it may
				# or may not be more consistent (and thus helpful) to do this.
				xyz = np.array([0.0, 0.0, self.state.pars.whisker_length[col]]) # whisker tip in WHISKER
				xyz = self.state.kc_w[row][col].changeFrameAbs('WHISKER', 'PARENT', xyz) # whisker tip in HEAD
				self.state.sig.whisker_tip_pos[row][col] = xyz

	def validate(self):

		# validation - these numbers can be put back into the generator to
		# graphically check everything is in the right place
		print "base = ["
		for row in self.state.pars.rows:
			for col in self.state.pars.cols:
				self.state.kc_w[row][col].link[5].angle = 0.0
				xyz = np.array([0.0, 0.0, 0.0])
				xyz = self.state.kc_w[row][col].changeFrameAbs('WHISKER_BASE', 'PARENT', xyz) # whisker tip in HEAD
				print xyz
		print "];"
		print "tip = ["
		for row in self.state.pars.rows:
			for col in self.state.pars.cols:
				self.state.kc_w[row][col].link[5].angle = 0.0
				xyz = np.array([0.0, 0.0, self.state.pars.whisker_length[col]]) # whisker tip in WHISKER
				xyz = self.state.kc_w[row][col].changeFrameAbs('WHISKER', 'PARENT', xyz) # whisker tip in HEAD
				print xyz
		print "];"
		raise ValueError("stop")



		"""

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% MAXYZ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add process
state = [];
state.config = pars.platform.config;
state.macro_chains = wb_get_macro_chains(job.pars.platform);
state.con_lo = 0.05;
state.con_hi = 0.10;
cls = 'dev/whiskerbots/condition/spatial';
sys = sys.addprocess('spatial', cls, pars.time.fS_lo, state);

% link
sys = sys.link('downsample>theta', 'spatial', 0);
sys = sys.link('downsample>macon', 'spatial', 0);

% expose
sys = sys.expose('spatial<h2w', 'h2w');
sys = sys.expose('spatial<w2h', 'w2h');

		"""


