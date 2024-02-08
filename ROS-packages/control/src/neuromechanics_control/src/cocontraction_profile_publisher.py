#!/usr/bin/env python

#!/usr/bin/env python

##**************************************************************************
 #                       cocontraction_profile_publisher.py                *
 #                           -------------------                           *
 # copyright            : (C) 2023 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                                   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/


import numpy as np

import numpy as np
import os

import rospy
from std_msgs.msg import Float64

from neuromechanics_control.msg import AnalogCompactDelay
from neuromechanics_control.msg import AnalogCompact

from rosgraph_msgs.msg import Clock

from dynamic_reconfigure.server import Server
from neuromechanics_control.cfg import CocontractionProfileDynParametersConfig

class CocontractionPublisher():
    def __init__(self, cocontraction, joint_list, output_topic, sampling_frequency, reconfig_server):

        self.cocontraction = cocontraction

        self.sampling_frequency = sampling_frequency
        self.time_step = 1.0 / self.sampling_frequency

        # self.publisher = rospy.Publisher(output_topic, AnalogCompactDelay, queue_size = 10)
        self.publisher = rospy.Publisher(output_topic, AnalogCompact, queue_size = 10)


        # self.cocontraction_msg = AnalogCompactDelay()
        self.cocontraction_msg = AnalogCompact()

        self.cocontraction_msg.data = [self.cocontraction] * len(joint_list)
        self.cocontraction_msg.names = joint_list

        self.dyn_srv = reconfig_server


    def publishCocontraction(self, time):
        self.cocontraction = self.dyn_srv.config["cocontraction"]

        for i in range(len(self.cocontraction_msg.data)):
            self.cocontraction_msg.data[i] = self.cocontraction
        t = round(self.sampling_frequency * time.to_sec()) * self.time_step
        t = round(t, 3)
        time_stamp = rospy.Time(t)

        self.cocontraction_msg.header.stamp = time_stamp
        self.publisher.publish(self.cocontraction_msg)

    # def dynamicCallback(self, config, level):
    #     self.cocontraction = config["cocontraction"]

    def clean_shutdown(self):
        print("\nExiting cocontraction publisher")


class ExternalClock():
    def __init__(self):
        self.received_time = rospy.Time(0.0)
        self.first_received = False

    def ClockCallback(self, data):
        self.first_received = True
        if self.received_time < data.clock:
            self.received_time = data.clock

    def GetLastConfirmed(self):
        return self.received_time

    def FirstReveived(self):
        return self.first_received



def main():
    # Initialize ROS node
    print("Initializing Cocontraction Publisher node... ")
    rospy.init_node("cocontraction_publisher_node", anonymous=True)

    cocontraction = rospy.get_param("~cocontraction")
    output_topic = rospy.get_param("~output_topic")
    joint_list = rospy.get_param("~joint_list")
    sampling_frequency = rospy.get_param("~sampling_frequency")

    use_sim_time = rospy.get_param("/use_sim_time")

    rate = rospy.Rate(sampling_frequency)

    dynamic_cfg_srv = Server(CocontractionProfileDynParametersConfig, lambda config, level: config)

    cocontraction_profile = CocontractionPublisher(cocontraction, joint_list, output_topic, sampling_frequency, dynamic_cfg_srv)

    rospy.on_shutdown(cocontraction_profile.clean_shutdown)


    # If simulated time: Subscribe to master clock
    if use_sim_time:
        ext_clock = ExternalClock()
        clock_subscriber = rospy.Subscriber("/clock", Clock, ext_clock.ClockCallback)
        current_time = rospy.Time(0.0)


    while not rospy.is_shutdown():
        if use_sim_time:
            new_time = ext_clock.GetLastConfirmed()
            if new_time > current_time:
                current_time = new_time
                cocontraction_profile.publishCocontraction(current_time)
        else:
            time = rospy.get_rostime()
            cocontraction_profile.publishCocontraction(time)
            rate.sleep()




if __name__ == '__main__':
    main()
