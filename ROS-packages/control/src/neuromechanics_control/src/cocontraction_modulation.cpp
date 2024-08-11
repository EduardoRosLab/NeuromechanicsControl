/***************************************************************************
 *                      cocontraction_modulation.cpp                       *
 *                           -------------------                           *
 * copyright            : (C) 2024 by Ignacio Abadía                       *
 * email                : iabadia@ugr.es			                             *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node computes the cocontraction level based on the input motion


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <neuromechanics_control/AnalogCompact.h>
#include "log4cxx/logger.h"
#include "neuromechanics_control/ExternalClock.h"
#include "neuromechanics_control/ROSCocontractionModulation.h"

#include <cstring>
#include <ctime>
#include <cmath>
#include <signal.h>

static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}



int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "cocontraction_modulation");
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic_pos_plus, input_topic_pos_minus, input_topic_vel_plus, input_topic_vel_minus, output_topic, output_topic_mean;
	std::vector<std::string> joint_list, joint_list_mean;
	std::vector<double> error_threshold, w_compliance_robustness, max_cocontraction;
	double sampling_frequency;

	bool use_sim_time;

	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	ros::Time current_time(0.0);

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic_pos_plus", input_topic_pos_plus);
	private_node_handle_.getParam("input_topic_pos_minus", input_topic_pos_minus);
	private_node_handle_.getParam("input_topic_pos_plus", input_topic_vel_plus);
	private_node_handle_.getParam("input_topic_pos_minus", input_topic_vel_minus);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("joint_list_mean", joint_list_mean);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("output_topic_mean", output_topic_mean);
	private_node_handle_.getParam("error_threshold", error_threshold);
	private_node_handle_.getParam("w_compliance_robustness", w_compliance_robustness);
	private_node_handle_.getParam("max_cocontraction", max_cocontraction);

	nh.getParam("use_sim_time", use_sim_time);

	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	double time_step = 1.0 / sampling_frequency;

	if (use_sim_time){
		ROS_DEBUG("Error estimator node: Subscribing to topic /clock");
		clock_subscriber = nh.subscribe("/clock", 1000, &ExternalClock::ClockCallback, &ext_clock);
	}

	// Create the publisher
	ros::Publisher output_publisher = nh.advertise<neuromechanics_control::AnalogCompact>(output_topic, 1.0);
	ROSCocontractionModulation objCocontractionModulation(input_topic_pos_plus,
			input_topic_pos_minus,
			input_topic_vel_plus,
			input_topic_vel_minus,
			output_topic,
			output_topic_mean,
			joint_list,
			joint_list_mean,
			error_threshold,
			w_compliance_robustness,
			max_cocontraction);

	ROS_INFO("Cocontraction modulation node initialized: writing to topic %s with frequency %f", output_topic.c_str(), sampling_frequency);

	ros::Rate rate(sampling_frequency);

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.001));
		if (use_sim_time){
			ros::Time new_time = ext_clock.GetLastConfirmedTime();
			if (new_time > current_time){
				current_time = new_time;
				ROS_DEBUG("Updating cocontraction at time %f", current_time.toSec());
				objCocontractionModulation.UpdateCocontraction(current_time);
				rate.sleep();
			}
		}
		else{
			// current_time = ros::Time::now();
			// Ensure a 2ms time step between the error samples
			current_time =  ros::Time(round(sampling_frequency * ros::Time::now().toSec()) * time_step);
			ROS_DEBUG("Updating cocontraction at time %f", current_time.toSec());
			objCocontractionModulation.UpdateCocontraction(current_time);
			rate.sleep();
		}
	}

	ROS_INFO("Ending cocontraction modulation node");

	ros::shutdown();
	return 0;
} // end main()
