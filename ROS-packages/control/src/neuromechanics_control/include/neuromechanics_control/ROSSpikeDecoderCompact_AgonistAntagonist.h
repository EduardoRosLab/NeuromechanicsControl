/***************************************************************************
 *                           ROSSpikeDecoderCompact_AgonistAntagonist.h                             *
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

#ifndef ROSSPIKEDECODERCOMPACT_AGONISTANTAGONIST_H_
#define ROSSPIKEDECODERCOMPACT_AGONISTANTAGONIST_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <neuromechanics_control/Spike.h>
#include <neuromechanics_control/Spike_group.h>



class spike_comparison {
public:
	bool operator() (const neuromechanics_control::Spike lsp, const neuromechanics_control::Spike rsp) const{
		if (lsp.time>rsp.time) {
			return true;
		} else if (lsp.time<rsp.time) {
			return false;
		} else {
			return (lsp.neuron_index > rsp.neuron_index);
		}
	}
};

/*
 * This class defines a spike-to-analog decoder.
 */
class ROSSpikeDecoderCompact_AgonistAntagonist {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Input spike activity suscriber
	ros::Subscriber subscriber;

	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of spike activity not processed yet
	std::priority_queue<neuromechanics_control::Spike, std::vector<neuromechanics_control::Spike>, spike_comparison > activity_queue;

	// Indexes of the neurons
	std::vector<unsigned int> min_neuron_index_pos;

	std::vector<unsigned int> max_neuron_index_pos;

	std::vector<unsigned int> min_neuron_index_neg;

	std::vector<unsigned int> max_neuron_index_neg;

	// Tau time constant
	double tau_time_constant;

	// Increment with each spike
	std::vector<double> spike_increment_pos;

	std::vector<double> spike_increment_neg;

	// Value of the agonist and antagonist output variables
	std::vector<double> output_var_agonist;
	std::vector<double> output_var_antagonist;


	// Last time when the decoder has been updated
	double last_time;

	// Callback function for reading input activity
	void SpikeCallback(const neuromechanics_control::Spike_group::ConstPtr& msg);

	// Callback function for reading input activity
	//void SpikeCallback(const neuromechanics_control::Spike::ConstPtr& msg);

public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	ROSSpikeDecoderCompact_AgonistAntagonist(std::string input_topic_name,
			std::vector<int> min_neuron_index_pos,
			std::vector<int> max_neuron_index_pos,
			std::vector<int> min_neuron_index_neg,
			std::vector<int> max_neuron_index_neg,
			double tau_time_constant,
			std::vector<double> spike_increment_pos,
			std::vector<double> spike_increment_neg);

	/*
	 * Update the output variables with the current time and the output activity
	 */
	void UpdateDecoder(double end_time);

	/*
	 * Return the output variables
	 */
	std::vector<double> GetAgonistValue();
	std::vector<double> GetAntagonistValue();

	/*
	 * Destructor of the class
	 */
	virtual ~ROSSpikeDecoderCompact_AgonistAntagonist();

};

#endif /* ROSSPIKEDECODERCOMPACT_AGONISTANTAGONIST_H_ */
