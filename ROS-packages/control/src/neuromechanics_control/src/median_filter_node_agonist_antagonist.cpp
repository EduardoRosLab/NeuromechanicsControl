/***************************************************************************
 *                           median_filter_node.cpp          		           *
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

// This node applies a mean filter to the analogue input data from a specified
// input topic. The number of samples used for the mean filter is also specified


#include <neuromechanics_control/AnalogCompact.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <neuromechanics_control/AnalogCompact_AgonistAntagonist.h>



static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}

/*
 * Create a class
 */
class MedianFilter {
public:
	ros::Subscriber clock_subscriber;

private:
	ros::Publisher joint_state_publisher;
	std::vector<std::string> joint_list;

	neuromechanics_control::AnalogCompact_AgonistAntagonist msgState;


	int num_joints;
	int num_samples;
	int median_center;
	double ** input_samples_agonist;
	double ** input_samples_antagonist;
	double ** output_ordered_samples_agonist;
	double ** output_ordered_samples_antagonist;


	std::vector<double> output_samples_agonist;
	std::vector<double> output_samples_antagonist;

	bool use_sim_time;

	int FindJointIndex(std::vector<std::string> strvector, std::string name){
		std::vector<std::string>::iterator first = strvector.begin();
		std::vector<std::string>::iterator last = strvector.end();
		unsigned int index = 0;
		bool found = false;

		while (first!=last && !found) {
			if (*first==name)
				found = true;
			else {
				++first;
				++index;
			}
		}

		if (found) {
			return index;
		} else {
			return -1;
		}
	};

public:
	MedianFilter (int num_samples, std::vector<std::string> & joint_list,
		std::string output_topic, bool sim_time): num_samples(num_samples),
		joint_list(joint_list), use_sim_time(sim_time) {

		ros::NodeHandle nh;
		this->joint_state_publisher = nh.advertise<neuromechanics_control::AnalogCompact_AgonistAntagonist>(output_topic, 1);
		ROS_DEBUG("Joint State MedianFilter: Writing state to topic %s",output_topic.c_str());

		this->msgState.names = this->joint_list;
		this->msgState.agonist.resize(this->joint_list.size());
		this->msgState.antagonist.resize(this->joint_list.size());
		this->msgState.header.stamp = ros::Time(0);

		if (this->num_samples < 3){
			this->num_samples = 3;
		}
		this->median_center = this->num_samples/2;

		this->num_joints = this->joint_list.size();
		this->input_samples_agonist = (double **) new double * [this->num_joints];
		this->input_samples_antagonist = (double **) new double * [this->num_joints];
		this->output_ordered_samples_agonist = (double **) new double * [this->num_joints];
		this->output_ordered_samples_antagonist = (double **) new double * [this->num_joints];

		for (int i=0; i< this->num_joints; i++){
				this->input_samples_agonist[i]=new double[this->num_samples]();
				this->input_samples_antagonist[i]=new double[this->num_samples]();
				this->output_ordered_samples_agonist[i]=new double[this->num_samples]();
				this->output_ordered_samples_antagonist[i]=new double[this->num_samples]();
		}
		this->output_samples_agonist = std::vector<double> (this->num_joints, 0);
		this->output_samples_antagonist = std::vector<double> (this->num_joints, 0);

		return;
	};

	void EventCallback(const neuromechanics_control::AnalogCompact_AgonistAntagonist::ConstPtr& msg){
		this->Filter(msg->agonist, msg->antagonist);
		this->msgState.header.stamp = msg->header.stamp;
		this->PublishState();

		return;
	};

	void Filter(std::vector<double> new_value_agonist, std::vector<double> new_value_antagonist){

		for (int z=0; z<this->num_joints; z++){
			//input_samples
			float mean_agonist=0.0;
			float mean_antagonist=0.0;
			for(int i=this->num_samples-2; i>=0; i--){
					//input_samples
					this->input_samples_agonist[z][i+1]=this->input_samples_agonist[z][i];
					mean_agonist+=this->input_samples_agonist[z][i+1];
					this->input_samples_antagonist[z][i+1]=this->input_samples_antagonist[z][i];
					mean_antagonist+=this->input_samples_antagonist[z][i+1];
			}
			//new input sample
			this->input_samples_agonist[z][0]=new_value_agonist[z];
			this->input_samples_antagonist[z][0]=new_value_antagonist[z];
			mean_agonist+=this->input_samples_agonist[z][0];
			mean_antagonist+=this->input_samples_antagonist[z][0];
			this->output_samples_agonist[z]=mean_agonist/this->num_samples;
			this->output_samples_antagonist[z]=mean_antagonist/this->num_samples;
		}
	};

	void PublishState(){
		this->msgState.agonist = this->output_samples_agonist;
		this->msgState.antagonist = this->output_samples_antagonist;
		this->joint_state_publisher.publish(msgState);
		return;
	};
};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "median_filter", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.

	std::string input_topic, output_topic, clock_topic;
	std::vector<std::string> joint_list;
	bool sim_time;
	double checking_frequency, sampling_frequency;
	int num_samples;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("num_samples", num_samples);

	nh.getParam("use_sim_time", sim_time);

	MedianFilter objFilter = MedianFilter(num_samples, joint_list, output_topic, sim_time);

	ros::Rate rate(1000);
	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);
	ros::Subscriber sub = nh.subscribe(input_topic, 1000, &MedianFilter::EventCallback, &objFilter);

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.001));
		rate.sleep();
	}
	ROS_INFO("Ending Median Filter node");
	ros::shutdown();
	return 0;
} // end main()
