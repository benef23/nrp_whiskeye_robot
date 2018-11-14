#!/usr/bin/python

import numpy as np
import math
import matplotlib.pyplot as plt
import time

from model_def import *

# import ROS
import rospy
import std_msgs

class Monitor:

	def __init__(self):

		# get pars
		self.pars = SystemPars()

		# prepare figure
		self.slice_fig = plt.figure()
		mapval = np.zeros(self.pars.headspace.shape)
		mapslice = mapval[0, :, :]
		mapslice = self.remap(mapslice)
		extent = self.pars.headspace.ylim + self.pars.headspace.zlim
		self.slice_image = plt.imshow(mapslice, vmin=0.0, vmax=1.0, extent=extent, origin='lower')
		self.slice_title = plt.title("...")
		plt.gca().invert_xaxis()
		plt.xlabel('y')
		plt.ylabel('z')

		# finalise figures
		plt.ion()
		plt.show()

		# init node
		rospy.init_node('monitor')

		# state
		self.tick = 0
		self.mapval = None

		# subscribe
		topic = self.pars.topic_root + '/tick'
		print "subscribing", topic
		self.sub_tick = rospy.Subscriber( topic, std_msgs.msg.Int32, self.callback_tick, queue_size=1 )

		# subscribe
		topic = self.pars.topic_root + '/mapval'
		print "subscribing", topic
		self.sub_mapval = rospy.Subscriber( topic, std_msgs.msg.Float32MultiArray, self.callback_mapval, queue_size=1 )

	def remap(self, mapslice):

		return np.transpose(mapslice)

	def callback_mapval(self, msg):

		# recover
		self.mapval = np.array(msg.data).reshape(self.pars.headspace.shape)

	def callback_tick(self, msg):

		# recover
		self.tick = msg.data

	def do_events(self):

		# wait for input
		if self.mapval is None:
			time.sleep(0.01)

		else:

			# debug (show boundary)
			"""
			self.mapval[:,0,:] = 0
			self.mapval[:,:,0] = 1
			"""

			# show slice of map
			mapslice = self.mapval[self.pars.headspace.xi_cen, :, :]
			self.slice_image.set_data(self.remap(mapslice))
			self.slice_title.set_text("sample " + str(self.tick))

			# iter
			self.mapval = None

			# update figure
			self.slice_fig.canvas.draw()

		# flush event queue
		self.slice_fig.canvas.flush_events()

mon = Monitor()

while not rospy.is_shutdown():
	mon.do_events()



