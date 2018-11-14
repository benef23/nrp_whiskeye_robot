#!/usr/bin/python

from model_def import *
from ModulePlatform import *
from ModuleSensory import *
from ModuleSpatial import *
from ModuleAction import *
from ModuleMotor import *

import cProfile

# import ROS
import rospy
import std_msgs

def main():

	# get state
	state = SystemState()

	# create modules
	m_sensory = ModuleSensory(state)
	m_spatial = ModuleSpatial(state)
	m_action = ModuleAction(state)
	m_motor = ModuleMotor(state)

	# performance
	if False:
		print "---- perf"
		m_sensory.perf()
		m_spatial.perf()
		m_action.perf()
		m_motor.perf()
		return

	# validation
	#m_sensory.validate()

	# init node
	rospy.init_node('model')

	# publish
	topic = state.pars.topic_root + "/tick"
	print "publishing", topic
	pub_tick = rospy.Publisher(topic, std_msgs.msg.Int32, queue_size=0)

	# create modules
	m_platform = ModulePlatform(state)

	# for each tick
#	for n in range(0, 100):
	while not state.cancel:

		# publish tick
		pub_tick.publish(state.time_n)

		# tick modules
		m_platform.tick()
		#if n == 200:
		#	m_sensory.perf_breakdown()
		#	break
		m_sensory.tick()
		m_spatial.tick()
		m_action.tick()
		m_motor.tick()

		# tick state
		state.tick()

	#print dir(state.sig)

#cProfile.run('main()', sort='cumtime')
main()



