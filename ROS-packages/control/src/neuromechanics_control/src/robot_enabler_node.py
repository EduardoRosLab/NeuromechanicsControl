#!/usr/bin/env python

##**************************************************************************
 #                           robot_enabler_node.py                         *
 #                           -------------------                           *
 # copyright            : (C) 2018 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                              *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This node has the purpose of enabling the robot before start.
# Once the robot is enabled it publishes on topic /robot_enabled so the other
# nodes know they can start their work.

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
import numpy as np
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import (
	JointCommand,
)


class Robot_Enabler(object):

	def __init__(self, baxter_arm):
		# Publisher to advertise once the robot is enabled
		self.pub = rospy.Publisher("/robot_enabled", Bool, queue_size=10, latch=True)
		self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
		# Publisher to put baxter's left arm in 0 position
		if baxter_arm == 'left':
			self.pub_position = rospy.Publisher('robot/limb/left/joint_command', JointCommand, queue_size = 10)
		if baxter_arm == 'right':
			self.pub_position = rospy.Publisher('robot/limb/right/joint_command', JointCommand, queue_size = 10)

		# Baxter interface to have access to the state of the robot
		self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
		# Baxter limb to have access to the robot's position
		self.limb = baxter_interface.Limb(baxter_arm)

		# Dictionary to store the robot's left arm position
		if baxter_arm == 'left':
			self.angles = {'left_s0': 100.0, 'left_s1': 100.0, 'left_e0': 100.0, 'left_e1': 100.0, 'left_w0': 100.0, 'left_w1': 100.0, 'left_w2': 100.0}
		if baxter_arm == 'right':
			self.angles = {'right_s0': 100.0, 'right_s1': 100.0, 'right_e0': 100.0, 'right_e1': 100.0, 'right_w0': 100.0, 'right_w1': 100.0, 'right_w2': 100.0}

		# Variable to store baxter state
		self.enabled = False

		# Message components to put arm in 0 position (mode 1 = position control mode)
		self.mode = 1
		self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		if baxter_arm == 'left':
			self.names = ['left_s0','left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
		if baxter_arm == 'right':
			self.names = ['right_s0','right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

		self.torque = []

		self.rate = rospy.Rate(500)


	# Function to get baxter's left arm position and store the position of every joint in a dictionary
	def get_angles(self):
		angles_random = self.limb.joint_angles()
		for k, v in angles_random.items():
	    		for x, y in self.angles.items():
		    		if k == x:
			    		self.angles[x] = v


	# Function to put baxter's left arm in 0 position. It publishes a message to do so and keeps publishing it
	# until the arm hasn't reached the 0 position (with a 0.008 angle tolerance)
	def zero_position(self):
		self.pub_position.publish(self.mode, self.position, self.names)
		reached = False
		while not reached:
			self.get_angles()
			count = 0
			for x in self.angles:
				if abs(self.angles[x]) > 0.008:
					reached = False
					count += 1
			if count == 0:
				reached = True
			self.pub_position.publish(self.mode, self.position, self.names)
			self.rate.sleep()


	# Function to enable the robot, put it in zero position,
	# and advertise that the robot is ready to start
	def robot_enabler(self):
		r = rospy.Rate(10) # 10hz
		while not self.rs.state().enabled:
			self.rs.enable()
		enabled = True
		published = False

		# Set maximum speed
		self.limb.set_joint_position_speed(1.0)

		#Set Baxter's joint state publish rate to 500 Hz
		self._pub_rate.publish(500)
		rospy.loginfo("Baxter joint state publish rate 500Hz")

		# while not published:
		# 	if (self.pub.get_num_connections() > 0):
		# 		self.pub.publish(enabled)
		# 		published = True
		# 	r.sleep()

	def set_speed(self):
		self.limb.set_joint_position_speed(1.0)


	# Function to send zero torque to Baxter
	def zero_torque(self):
		self.mode = 3
		self.torque = [0.0] * len(self.names)
		self.pub_position.publish(self.mode, self.torque, self.names)

		a = 0
		r = rospy.Rate(100) # 100hz

		# while a<100:
		# 	self.pub_position.publish(self.mode, self.torque, self.names)
		# 	rospy.loginfo("initial command ")
		# 	a+=1
		# 	r.sleep()

def main():
	rospy.init_node('robot_enabler', anonymous=True, disable_signals = True)
	baxter_arm = rospy.get_param("~limb")
	enabler = Robot_Enabler(baxter_arm)
	r = rospy.Rate(10) # 10hz

	use_sim_time = rospy.get_param("/use_sim_time")
	if use_sim_time:
        # ext_clock = ExternalClock()
        # clock_subscriber = rospy.Subscriber("/clock", Clock, ext_clock.ClockCallback)
		time = rospy.get_rostime().to_sec()
		while not time > 0.0:
			time = rospy.get_rostime().to_sec()
			r.sleep()
	enabler.robot_enabler()
	enabler.set_speed()
	while not rospy.is_shutdown():
			enabler.set_speed()

	rospy.signal_shutdown("enabling done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
