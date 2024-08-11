#!/usr/bin/env python

##**************************************************************************
 #                     cocontraction_per_trial.py                          *
 #                           -------------------                           *
 # copyright            : (C) 2024 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                             	   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/


# This script plots the cocontraction value of each trajectory trial

import time
import argparse

import rospy

import sys
import numpy
import matplotlib.pyplot

import threading

import matplotlib.lines
import matplotlib.animation

import neuromechanics_control.msg
from rosgraph_msgs.msg import Clock
from dynamic_reconfigure.server import Server
from neuromechanics_control.cfg import MaeDynParametersConfig


class CocontractionPlotAnimation(matplotlib.animation.TimedAnimation):

	#Callback for cocontraction topic
	def CocontractionCallback(self, data):
		if self.first_cocontraction:
			for joint, value in self.cocontraction_values.items():
				for index, name in enumerate(data.names):
					if data.names[index] == joint:
						self.aux_cocontraction_index.append(index)
			self.first_cocontraction = False

		if self.init_cocontraction:
			self.start = rospy.get_time()
			self.init_cocontraction = False
		now = rospy.get_time()

		if now - self.start < self.duration_trial:
			counter2 = 0
			for joint, value in self.cocontraction_values.items():
				self.cocontraction_values[joint] = data.data[self.aux_cocontraction_index[counter2]]
				counter2 += 1

			t_step_cocontraction = 0
			for joint in self.joint_list:
				t_step_cocontraction += self.cocontraction_values[joint]
			t_step_cocontraction = t_step_cocontraction * self.inverse_num_joints
			self.total_calls += 1.0
			self.cocontraction = self.cocontraction + t_step_cocontraction

		else:
			if self.total_calls != 0.0:
				self.cocontraction = self.cocontraction / self.total_calls
			self.trial_cocontraction.append(self.cocontraction)
			if (self.cocontraction > self.cocontraction_max):
				self.cocontraction_max = self.cocontraction
			self.trial_number.append(self.total_trials)
			self.total_trials += 1

			self.total_calls = 0.0
			self.cocontraction = 0
			self.init_cocontraction = True

		if (now - self.start > self.duration_trial):
			self.init_cocontraction = True

	#Callback for simulation time
	def callback(self, clock):
		self.sim_time = clock.clock

	# Callback to dynamically change parameters
	def param_callback(self, config, level):
		self.display = config["Display"]
		return config

	def new_figure(self):
		# Create the figure, the axis and the animation
		self.figure = matplotlib.pyplot.figure()
		self.axis = self.figure.add_subplot(1, 1, 1)
		self.axis.set_title(self.figureName)

		self.axis.set_xlabel('Trials (n)')
		self.axis.set_ylabel('Cocontraction')
		self.line = matplotlib.pyplot.Line2D([], [], color='red', marker='.', markersize=3,linestyle='solid', label = self.cocontractionTopic)

		self.axis.add_line(self.line)

		self.axis.set_ylim(0, self.cocontraction_max + 0.1)
		self.axis.set_xlim(1, self.total_trials + 1)

		matplotlib.animation.TimedAnimation.__init__(self, self.figure, interval=1./self.refreshRate*1000.0, repeat=False, blit=False)

	def __init__(self):
		rospy.init_node('MaePlot', log_level=rospy.INFO)

		#Retrieve RosLaunch parameters
		self.cocontractionTopic = rospy.get_param("~cocontraction_topic")
		self.joint_list = rospy.get_param("~joint_list")
		self.figureName = rospy.get_param("~figure_name")
		self.refreshRate = rospy.get_param("~refresh_rate")
		self.duration_trial = rospy.get_param("~duration_trial")

		#Get global parameter use_sim_time and reference_time
		self.use_sim_time = rospy.get_param("use_sim_time")
		self.ref_time = rospy.get_param("reference_time")

		self.init_cocontraction = True
		self.start = 0.0

		self.total_calls = 0.0
		self.total_trials = 0

		self.trial_cocontraction = []
		self.trial_number = []

		self.cocontraction_values = {}
		self.cocontraction = 0
		self.cocontraction_max = 0

		self.num_joints = len(self.joint_list)
		self.inverse_num_joints = 1.0 / self.num_joints

		self.first_cocontraction = True
		self.aux_cocontraction_index = []

		for joint in self.joint_list:
			self.cocontraction_values[joint] = 0

		# Define a lock for callback synchronization
		self.lock = threading.Lock()

		self.display = False

		#Simulation time in case use_sim_time True
		self.duration = rospy.Duration(0)
		self.checkpoint_time = rospy.Time(0)
		self.first = True
		self.sim_time = rospy.Time(0)

		#Subscribing to the topics
		rospy.Subscriber(self.cocontractionTopic, neuromechanics_control.msg.AnalogCompact, self.CocontractionCallback, queue_size=None)

	def _draw_frame(self, framedata):
		self.lock.acquire()
		xdata, ydata = self.line.get_data()
		xdata = numpy.append(xdata, self.trial_number)
		ydata = numpy.append(ydata, self.trial_cocontraction)

		self.trial_number = []
		self.trial_cocontraction = []
		self.lock.release()

		self.line.set_data(xdata,ydata)
		self._drawn_artists = [self.line]

		self.axis.set_ylim(0.0, self.cocontraction_max)
		self.axis.set_xlim(1, self.total_trials + 1)

		return self._drawn_artists

	def new_frame_seq(self):
		return iter(xrange(sys.maxint))

	def _init_draw(self):
		lines = [self.line]
		for l in lines:
			l.set_data([], [])

	def close_display(self):
		self.display = False

Animation = CocontractionPlotAnimation()
srv = Server(MaeDynParametersConfig, Animation.param_callback)
while not rospy.is_shutdown():
	if Animation.display:
		Animation.new_figure()
		matplotlib.pyplot.show()
		Animation.close_display()
	rospy.sleep(1)
