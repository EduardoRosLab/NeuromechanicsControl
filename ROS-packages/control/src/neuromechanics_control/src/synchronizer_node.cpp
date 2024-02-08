/***************************************************************************
 *                          synchronizer_node.cpp                          *
 *                           -------------------                           *
 * copyright            : (C) 2020 by Jesus Garrido, Francisco Naveros, 	 *
 *														Ignacio Abadía  														 *
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

// This is the synchronizer node. It controls that all nodes follow the same
// time evolution when running on simulation time.
// It waits for all nodes to have finished the current time step. Once they are
// all done, it tells them to execute the next time step, and so on.

#include "ros/ros.h"
#include "ros/console.h"
#include <ros/callback_queue.h>
#include "rosgraph_msgs/Clock.h"
#include <ros/subscribe_options.h>
#include "std_msgs/Bool.h"
// #include <baxter_core_msgs/JointCommand.h>

// #include <neuromechanics_control/ExternalClock.h>

#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <stdlib.h>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <neuromechanics_control/SynchronizerDynParams.h>


static bool stop_simulation;


void rosShutdownHandler(int sig)
{
	stop_simulation = true;
}


class ExternalClock{
private:
	ros::Time received_time;

	bool first_received;

	int counter;
public:
	ExternalClock(): received_time(0.0), first_received(false), counter(0) {}

	void ClockCallback (const rosgraph_msgs::Clock& msg){
		this->counter+=1;
		if (this->received_time<msg.clock){
			this->received_time = msg.clock;
		}
		return;
	}

	ros::Time GetLastConfirmedTime(){
		return this->received_time;
	}

	bool FirstReceived(){
		return this->first_received;
	}

	int GetCounter(){
		return this->counter;
	}

	void ResetCounter(){
		this->counter = 0;
	}
};


int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "synchronizer", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
   		ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;
	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string synchronizing_topic, master_clock_topic;
	ros::Subscriber synchronizing_subscriber;

	// std::vector<ExternalClock *> clock_objects;
	ExternalClock external_clock = ExternalClock();

	unsigned int num_topics;
	double checking_frequency, step_time;
	bool use_sim_time = false;
	ros::Publisher time_publisher;

	// struct timespec startt, endt;

	stop_simulation = false;

	// Number of external nodes to synchronize
	int number_of_nodes;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("clock_topic", synchronizing_topic);
	private_node_handle_.getParam("number_of_nodes", number_of_nodes);
	private_node_handle_.getParam("checking_frequency", checking_frequency);
	private_node_handle_.getParam("step_time", step_time);

	// Get global parameter use_sim_time
	nh.getParam("use_sim_time", use_sim_time);
	nh.getParam("master_clock_topic", master_clock_topic);

	// Synchronizer node will shutdown when running in real time
	if (!use_sim_time){
		ROS_WARN("Synchronizer: Simulated time is not enabled. Synchronizer_node will shutdown. If you want to enable simulated time set use_sim_time variable to true.");
		ros::shutdown();
		return 0;
	}

	// Set callback queue
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	// Include the private node handle in the callback queue for the dynamic reconfigure server
	private_node_handle_.setCallbackQueue(&CallbackQueue);

	// Dynamic parameters to control synchronizer node behavior
	// SynchronizerDynParams controller(&nh, &private_node_handle_);


	// num_topics = number_of_nodes;
	// synchronizing_subscribers.resize(num_topics);
	// clock_objects.resize(num_topics);

	// Create the subscribers and objects for every clock topic of the nodes that need to be synchronized
	// for (unsigned int i=0; i<num_topics; ++i){
	// 	clock_objects[i] = new ExternalClock();
	// 	ROS_DEBUG("Synchronizer: Subscribing to clock topic %s",synchronizing_topics[i].c_str());
	// 	synchronizing_subscribers[i] = nh.subscribe(synchronizing_topics[i], 10, &ExternalClock::ClockCallback, clock_objects[i]);
	// }

	synchronizing_subscriber = nh.subscribe(synchronizing_topic, 1000, &ExternalClock::ClockCallback, &external_clock);

	// Publisher to advertise synchronizer node clock signal. This is the clock signal that all the nodes will follow
	time_publisher  = nh.advertise<rosgraph_msgs::Clock>(master_clock_topic, 100);

	ROS_INFO("Synchronizer node initialized.");

	ros::WallRate rate(checking_frequency);
	ros::WallRate init_rate(1.0);

	ros::Time next_time (0.0), last_sent_time(0.0);
	bool all_first_received = false;

	// Message to advertise current synchronized time
	rosgraph_msgs::Clock current_time;

	// Publish start simulation time 0.0
	current_time.clock = ros::Time(0.0);
	ROS_DEBUG("Synchronizer: Publishing simulation time %f", current_time.clock.toSec());
	time_publisher.publish(current_time);

	// Wait until all nodes have published 0.0 time and the synchronizer has received it
	// The synchronizer node keeps publishing 0.0 time until all nodes are synchronized
	while(external_clock.GetCounter() != number_of_nodes){
		// ROS_INFO("FIRST COUNTER = %i", external_clock.GetCounter());
		CallbackQueue.callAvailable(ros::WallDuration(0.01));
		ROS_DEBUG("Synchronizer: Publishing simulation time %f", current_time.clock.toSec());
		time_publisher.publish(current_time);
	}
	external_clock.ResetCounter();

	ROS_DEBUG("Synchronizer: Node synchronized");

	// Time stamp for statistical purposes
	ros::WallTime start = ros::WallTime::now();

	// Synchronizer loop
	while (!stop_simulation){
		CallbackQueue.callAvailable(ros::WallDuration(0.004));

		// Dynamic parameters control. If paused button is true or the simulation has reached the time stamp
		// specified by the user ---> STOP THE SIMULATION
		// while (controller.GetPaused() || (next_time.toSec()>=controller.GetTimeStamp() && controller.GetStopTS())){
		// 	CallbackQueue.callAvailable(ros::WallDuration(0.004));
		// }

		// Check if all nodes have finished the current time step
		unsigned int finished_nodes = external_clock.GetCounter();
		// for (unsigned int i=0; i<num_topics; ++i){
		// 	if (clock_objects[i]->GetLastConfirmedTime()>=next_time){
		// 		finished_nodes++;
		// 	}
		// }
		ROS_DEBUG("Finished nodes %i",finished_nodes);
		// When all nodes have finished the current time step, the synchronizer publishes the next time step
		if (finished_nodes == number_of_nodes){
			next_time += ros::Duration(step_time);
			ROS_DEBUG("Synchronizer: All nodes finished. Sending new time stamp %f",next_time.toSec());
			current_time.clock = next_time;
			time_publisher.publish(current_time);
			ROS_DEBUG("Synchronizer: Publishing next time %f", next_time.toSec());
			external_clock.ResetCounter();
		}
		// else if (last_sent_time != current_time){
		// 	time_publisher.publish(current_time);
		// 	ROS_DEBUG("Synchronizer: Publishing last time %f", next_time.toSec());
		// 	last_sent_time = current_time;
		// }
		// rate.sleep();
	}

	ROS_INFO("Ending Synchronizer node");

	// Time statistics
	ros::Time sim_time = current_time.clock;
	ros::WallTime end = ros::WallTime::now();
	ros::WallDuration total = end - start;
	ROS_INFO("Elapsed time : %f", total.toSec());
	ROS_INFO("Simulated Time : %f", sim_time.toSec());
	ROS_INFO("Simulation ratio: %f", sim_time.toSec() / total.toSec());

	// Shutdown node
	ros::shutdown();

	return 0;
} // end main()
