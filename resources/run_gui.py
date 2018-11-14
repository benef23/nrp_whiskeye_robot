#!/usr/bin/env python

import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf

GObject.threads_init()
Gdk.threads_init()

import rospy
from geometry_msgs.msg import Twist

import math
import numpy
import copy
import time
import threading
import sys

#INTERPTYPE = GdkPixbuf.InterpType.NEAREST
INTERPTYPE = GdkPixbuf.InterpType.BILINEAR

################################################################

def fmt(x, f):
	s = ""
	x = bytearray(x)
	for i in range(0, len(x)):
		if not i == 0:
			s = s + ", "
		s = s + f.format(x[i])
	return s

def hex2(x):
	return "{0:#04x}".format(x)

def hex4(x):
	return "{0:#06x}".format(x)

def hex8(x):
	return "{0:#010x}".format(x)

def flt3(x):
	return "{0:.3f}".format(x)

def error(msg):
	print(msg)
	sys.exit(0)

################################################################

class whiskeye_gui:

	def on_window_main_destroy(self, object, data=None):
		print("quit with cancel")
		Gtk.main_quit()

	def on_window_key_release_event(self, object, data=None):
		self.move("x")

	def on_window_key_press_event(self, object, data=None):
		k = Gdk.keyval_name(data.keyval)
		if k == "Up":
			self.move("f")
		if k == "Down":
			self.move("b")
		if k == "Left":
			self.move("l")
		if k == "Right":
			self.move("r")

	def on_gtk_quit_activate(self, menuitem, data=None):
		print("quit from menu")
		Gtk.main_quit()

	def btn_move(self, object, data):
		self.move(data)

	def move(self, data):
		msg = Twist()
		if data == "f":
			msg.linear.x = self.speed_linear
		if data == "b":
			msg.linear.x = -self.speed_linear
		if data == "l":
			msg.angular.z = self.speed_angular
		if data == "r":
			msg.angular.z = -self.speed_angular
		else:
			pass # leave at zero
		self.msg_cmd_vel = msg

	def update_ui(self):

		# publish
		self.pub_cmd_vel.publish(self.msg_cmd_vel)

		return True

	def __init__(self):

		# report
		print("initialising...")
		print(sys.version)

		# initialise
		self.builder = Gtk.Builder()
		self.builder.add_from_file("gui/gui.glade")

		# get references
		self.window_main = self.builder.get_object("window_main")

		# buttons
		self.builder.get_object("btn_f").connect("pressed", self.btn_move, "f")
		self.builder.get_object("btn_b").connect("pressed", self.btn_move, "b")
		self.builder.get_object("btn_l").connect("pressed", self.btn_move, "l")
		self.builder.get_object("btn_r").connect("pressed", self.btn_move, "r")
		self.builder.get_object("btn_f").connect("released", self.btn_move, "x")
		self.builder.get_object("btn_b").connect("released", self.btn_move, "x")
		self.builder.get_object("btn_l").connect("released", self.btn_move, "x")
		self.builder.get_object("btn_r").connect("released", self.btn_move, "x")

		# options
		#self.opt.robot_is_phys = None

		# handle args
		"""
		for arg in sys.argv[1:]:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "robot":
				self.opt.robot_name = val
			elif key == "rob":
				self.opt.robot_is_phys = True
			elif key == "sim":
				self.opt.robot_is_phys = False
			elif key == "uncompressed":
				self.opt.uncompressed = True
			else:
				error("argument not recognised \"" + arg + "\"")

		# infer rob/sim
		if self.opt.robot_is_phys is None:
			if self.opt.robot_name[:3] == "rob":
				self.opt.robot_is_phys = True
				print("inferred that robot server is a physical robot from its name")
			else:
				self.opt.robot_is_phys = False
				print("inferred that robot server is a simulated robot from its name")
		"""

		# parameters
		self.speed_linear = 0.5
		self.speed_angular = 1.0

		# default data
		self.msg_cmd_vel = Twist()

		# connect events
		self.window_main.connect("destroy", self.on_window_main_destroy)
		self.window_main.connect("key-press-event", self.on_window_key_press_event)
		self.window_main.connect("key-release-event", self.on_window_key_release_event)

		# colors
		#self.window_main.modify_bg(Gtk.StateFlags.NORMAL, Gdk.Color(30000, 25000, 25000))

		# load images
		#self.image_caml.set_from_file("../../share/media/test_image_320.png")

		# show main window
		self.window_main.show()

		# subscribe
		topic_root = "/whiskeye"
		"""
		self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
				platform_sensors, self.callback_platform_sensors)
		"""

		# publish
		self.pub_cmd_vel = rospy.Publisher(topic_root + "/gui/cmd_vel", Twist, queue_size=0)

		# start update timer
		GLib.timeout_add(100, self.update_ui)

if __name__ == "__main__":
	rospy.init_node("whiskeye_gui", anonymous=True)
	main = whiskeye_gui()
	Gtk.main()



