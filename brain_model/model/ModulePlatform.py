
import time
import copy
import numpy as np
from model_support import *

# import ROS
import rospy
import std_msgs
from geometry_msgs.msg import *
from whiskeye_msgs.msg import *

def set_multi_array_dim_from_example(dim, x, labels = None):

	n = len(x.shape)
	for i in range(0, n):
		d = std_msgs.msg.MultiArrayDimension()
		d.size = x.shape[i]
		d.stride = x.strides[i]
		if not labels is None:
			d.label = labels[i]
		dim.append(d)

class ModulePlatform:

	def __init__(self, state):

		self.state = state
		self.t_bak = 0
		topic_root = '/whiskeye'

		"""
		# subscribe
		topic = topic_root + '/head/neck'
		print "subscribing", topic
		self.sub_neck = rospy.Subscriber( topic, std_msgs.msg.Float32MultiArray, self.callback_neck, queue_size=1 )

		# subscribe
		topic = topic_root + '/head/theta'
		print "subscribing", topic
		self.sub_theta = rospy.Subscriber( topic, std_msgs.msg.Float32MultiArray, self.callback_theta, queue_size=1 )

		"""

		# subscribe
		topic = topic_root + '/body/bumper'
		print "subscribing", topic
		self.sub_bumper = rospy.Subscriber( topic, std_msgs.msg.Bool, self.callback_bumper, queue_size=1, tcp_nodelay=True )

		# subscribe
		topic = topic_root + '/head/bridge_u'
		print "subscribing", topic
		self.sub_bridge_u = rospy.Subscriber( topic, bridge_u, self.callback_bridge_u, queue_size=1, tcp_nodelay=True )

		# subscribe
		topic = topic_root + '/gui/cmd_vel'
		print "subscribing", topic
		self.sub_cmd_vel = rospy.Subscriber( topic, Twist, self.callback_cmd_vel, queue_size=1, tcp_nodelay=True )

		# default input data
		self.input_neck = None #np.radians(np.array([-50.0, 50.0, 0.0]))
		self.input_theta = np.zeros(self.state.pars.shape_hi)
		self.input_xy = np.zeros(self.state.pars.shape_hi_2)
		self.input_physical = False

		# publish
		topic = topic_root + '/head/neck_cmd'
		print "publishing", topic
		self.pub_neck_cmd = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = topic_root + '/body/cmd_vel'
		print "publishing", topic
		self.pub_cmd_vel = rospy.Publisher(topic, Twist, queue_size=0)

		# publish
		topic = topic_root + '/head/theta_cmd'
		print "publishing", topic
		self.pub_theta_cmd = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = self.state.pars.topic_root + "/mapval"
		print "publishing", topic
		self.pub_mapval = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = self.state.pars.topic_root + "/xy"
		print "publishing", topic
		self.pub_xy = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = self.state.pars.topic_root + "/xy_q"
		print "publishing", topic
		self.pub_xy_q = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = self.state.pars.topic_root + "/theta"
		print "publishing", topic
		self.pub_theta = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

		# publish
		topic = self.state.pars.topic_root + "/theta_f"
		print "publishing", topic
		self.pub_theta_f = rospy.Publisher(topic, std_msgs.msg.Float32MultiArray, queue_size=0)

	def tick(self):

		# wait for input
		while self.input_neck is None:
			if rospy.is_shutdown():
				raise ValueError("ROS shutdown")
			time.sleep(0.001)

		# propagate inputs into sig
		self.state.sig.neck = self.input_neck
		self.state.sig.theta = self.input_theta
		self.state.sig.xy = self.input_xy
		self.state.sig.physical = self.input_physical

		# sync with clock and clear connection with local buffers
		self.input_neck = None

		# publish
		msg = Twist()
		msg.linear.x = self.state.sig.cmd_vel[0]
		msg.linear.y = self.state.sig.cmd_vel[1]
		msg.angular.z = self.state.sig.cmd_vel[2]
		self.pub_cmd_vel.publish(msg)

		# publish
		msg = std_msgs.msg.Float32MultiArray()
		set_multi_array_dim_from_example(msg.layout.dim, self.state.sig.neck_cmd, ['dof'])
		msg.data = self.state.sig.neck_cmd.flatten().tolist()
		self.pub_neck_cmd.publish(msg)

		# publish
		msg = std_msgs.msg.Float32MultiArray()
		set_multi_array_dim_from_example(msg.layout.dim, self.state.sig.theta_cmd, ['row', 'col'])
		msg.data = self.state.sig.theta_cmd.flatten().tolist()
		self.pub_theta_cmd.publish(msg)

		# publish
		if not self.state.sig.mapval is None:
			msg = std_msgs.msg.Float32MultiArray()
			set_multi_array_dim_from_example(msg.layout.dim, self.state.sig.mapval, ['x', 'y', 'z'])
			msg.data = self.state.sig.mapval.flatten().tolist()
			self.pub_mapval.publish(msg)

		# publish debugs
		if True:
			# republish inputs for matlab

			# xy
			msg = std_msgs.msg.Float32MultiArray()
			msg.data = self.state.sig.xy.flatten().tolist()
			self.pub_xy.publish(msg)
			# xy_q
			msg = std_msgs.msg.Float32MultiArray()
			msg.data = self.state.sig.xy_q.flatten().tolist()
			self.pub_xy_q.publish(msg)
			# theta
			msg = std_msgs.msg.Float32MultiArray()
			msg.data = self.state.sig.theta.flatten().tolist()
			self.pub_theta.publish(msg)
			# theta_f
			msg = std_msgs.msg.Float32MultiArray()
			msg.data = self.state.sig.theta_f.flatten().tolist()
			self.pub_theta_f.publish(msg)

	def callback_bumper(self, msg):

		if msg.data:
			print "detected bumper press"
			self.state.cancel = True

	def callback_bridge_u(self, msg):

		"""
		t = time.time()
		dt = t - self.t_bak
		print np.round(dt * 1000)
		self.t_bak = t
		"""

		x = np.asarray(msg.theta.data)
		x = np.reshape(x, self.state.pars.shape_hi)
		self.input_theta = x

		x = np.asarray(msg.xy.data)
		x = np.reshape(x, self.state.pars.shape_hi_2)
		self.input_xy = x

		x = np.asarray(msg.neck.data)
		x = np.reshape(x, (self.state.pars.fS_scale, 3))
		self.input_neck = np.mean(x, axis=0)

		self.input_physical = msg.physical.data

	def callback_cmd_vel(self, msg):

		self.state.sig.cmd_vel_gui = [msg.linear.x, msg.linear.y, msg.angular.z]




