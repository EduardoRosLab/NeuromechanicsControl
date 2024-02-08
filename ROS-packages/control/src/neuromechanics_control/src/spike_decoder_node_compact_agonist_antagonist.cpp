/***************************************************************************
 *                           spike_decoder_node.cpp                        *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Jesus Garrido                        *
 * email                : jesusgarrido@ugr.es                              *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This is the spike decoder node. It translates spiking activity to analogue
// signals.

#include <neuromechanics_control/AnalogCompact_AgonistAntagonist.h>
#include <neuromechanics_control/ROSSpikeDecoderCompact_AgonistAntagonist.h>
#include <ros/ros.h>
#include "log4cxx/logger.h"

#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>

#include "neuromechanics_control/ExternalClock.h"

static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}


int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "spike_decoder", ros::init_options::NoSigintHandler);
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic, output_topic;
	std::vector<int> min_neuron_index_pos, max_neuron_index_pos, min_neuron_index_neg, max_neuron_index_neg;
	std::vector<double> delta_spike_pos, delta_spike_neg, torque_max_pos, torque_max_neg;
	std::vector<std::string> joint_list;
	double sampling_frequency, tau_time_constant, min_agonist, max_agonist, min_antagonist, max_antagonist;
	bool use_sim_time;
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	ros::Time current_time;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("min_neuron_index_list_pos", min_neuron_index_pos);
	private_node_handle_.getParam("max_neuron_index_list_pos", max_neuron_index_pos);
	private_node_handle_.getParam("min_neuron_index_list_neg", min_neuron_index_neg);
	private_node_handle_.getParam("max_neuron_index_list_neg", max_neuron_index_neg);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("tau_time_constant", tau_time_constant);
	private_node_handle_.getParam("delta_spike_list_pos", delta_spike_pos);
	private_node_handle_.getParam("delta_spike_list_neg", delta_spike_neg);
	private_node_handle_.getParam("torque_max_pos", torque_max_pos);
	private_node_handle_.getParam("torque_max_neg", torque_max_neg);
	private_node_handle_.getParam("min_agonist", min_agonist);
	private_node_handle_.getParam("max_agonist", max_agonist);
	private_node_handle_.getParam("min_antagonist", min_antagonist);
	private_node_handle_.getParam("max_antagonist", max_antagonist);


	nh.getParam("use_sim_time", use_sim_time);

	// Create the subscriber
  ros::CallbackQueue CallbackQueue;
  nh.setCallbackQueue(&CallbackQueue);

	// Create the publisher
	ros::Publisher output_publisher = nh.advertise<neuromechanics_control::AnalogCompact_AgonistAntagonist>(output_topic, 1.0);

	ROSSpikeDecoderCompact_AgonistAntagonist objSpikeDecoder(
			input_topic,
			min_neuron_index_pos,
			max_neuron_index_pos,
			min_neuron_index_neg,
			max_neuron_index_neg,
			tau_time_constant,
			delta_spike_pos,
			delta_spike_neg
	);

	ROS_INFO("Spike Decoder node initialized: reading from topic %s and writing to topic %s", input_topic.c_str(), output_topic.c_str());

	ros::Rate rate(sampling_frequency);

	if (use_sim_time){
		clock_subscriber = nh.subscribe("/clock", 1000, &ExternalClock::ClockCallback, &ext_clock);
	}

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.001));
		if (use_sim_time){
			current_time = ext_clock.GetLastConfirmedTime();
		}
		else{
			current_time = ros::Time::now();
		}

		// Update the analog values and store result
		objSpikeDecoder.UpdateDecoder(current_time.toSec());
		std::vector<double> analog_value_agonist = objSpikeDecoder.GetAgonistValue();
		std::vector<double> analog_value_antagonist = objSpikeDecoder.GetAntagonistValue();


		// Normalize the analog value to the maximum torque and keep it within the bounds
		for (unsigned int i=0; i<analog_value_agonist.size(); ++i){
			analog_value_agonist[i] = analog_value_agonist[i] / torque_max_pos[i];
			if (analog_value_agonist[i] < min_agonist){
				analog_value_agonist[i] = min_agonist;
			}
			else if (analog_value_agonist[i] > max_agonist){
				analog_value_agonist[i] = max_agonist;
			}
			analog_value_antagonist[i] = analog_value_antagonist[i] / torque_max_neg[i];
			if (analog_value_antagonist[i] < min_antagonist){
				analog_value_antagonist[i] = min_antagonist;
			}
			else if (analog_value_antagonist[i] > max_antagonist){
				analog_value_antagonist[i] = max_antagonist;
			}
		}


		std::ostringstream oss;
    	std::copy(analog_value_agonist.begin(), analog_value_agonist.end(), std::ostream_iterator<double>(oss, ","));
		ROS_DEBUG("Updating spike decoder agonist activity at time %f. Value returned: %s", current_time.toSec(), oss.str().c_str());
		std::ostringstream oss2;
			std::copy(analog_value_antagonist.begin(), analog_value_antagonist.end(), std::ostream_iterator<double>(oss2, ","));
		ROS_DEBUG("Updating spike decoder antgonist activity at time %f. Value returned: %s", current_time.toSec(), oss2.str().c_str());

		neuromechanics_control::AnalogCompact_AgonistAntagonist msg;
		msg.agonist = analog_value_agonist;
		msg.antagonist = analog_value_antagonist;
		msg.names = joint_list;
		msg.header.stamp = current_time;
		output_publisher.publish(msg);
		rate.sleep();
	}

	ROS_INFO("Ending Spike Decoder node");

	ros::shutdown();
	return 0;
} // end main()
