/***************************************************************************
 *                           Q_TrajectoryController.h                  *
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

#ifndef Q_TRAJECTORYCONTROLLER_H_
#define Q_TRAJECTORYCONTROLLER_H_

#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include "neuromechanics_control/ExternalClock.h"


class Q_TrajectoryGenerator;

/*
 * This class defines a Poisson generator with tanh function.
 */
class Q_TrajectoryController {

private:
	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// ROS CallbackQueue
	ros::CallbackQueue CallbackQueue;

	// Desired position publishers
	ros::Publisher desired_position_pub;

	// Desired velocity publishers
	ros::Publisher desired_velocity_pub;

	// Joint command publisher
	ros::Publisher joint_command_pub;

	// Control signal publisher
	ros::Publisher control_signal_pub;

	// Joint state subscriber
	ros::Subscriber joint_state_sub;

	//Subscriber and external clock for synchronizer clock
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	//Simulation time active
	bool use_sim_time;

	// Length of each trial
	double trial_length;

	// Number of trials
	unsigned int total_number_of_trials, number_of_trials;

	// In-learning signal
	bool in_learning;

	// Sending and checking rate
	ros::Rate rate;

	// Trajectory generator
	Q_TrajectoryGenerator * trajectory_generator;

	// Trial init time
	ros::Time trial_init_time;

	// List of joints as indicated in desired_* topics and sin vectors
	std::vector<std::string> joint_list;

	double sampling_frequency;

	double time_step;

	// List of baxter joint states as indicated in joint_list
	//std::vector<double> baxter_last_state;

	// Move to starting point function
	void MoveToStartingPoint(const bool & StopMovement);

	// Perform a learning trial
	void LearningTrial(const bool & StopMovement);

	// Auxiliary function to find a string in a vector. It returns -1 if not found, otherwise the index.
	int FindJointIndex(std::vector<std::string> strvector, std::string name);

	// Callback function for Baxter state
	//void BaxterStateCallback(const sensor_msgs::JointState::ConstPtr& msg);


public:

	/*
	 * This function initializes a trajectory controller.
	 */
	Q_TrajectoryController(std::vector<std::string> joint_list,
			std::string desired_position_topic,
			std::string desired_velocity_topic,
			std::string control_topic,
			std::string joint_command_topic,
			Q_TrajectoryGenerator * trajectory_generator,
			unsigned int total_number_of_trials,
			double trial_length,
			double update_frequency,
			bool sim_time);

	/*
	 * This function executes a controller step. It returns
	 * true if the simulation is not ended yet and false otherwise.
	 */
	bool NextControlStep(const bool & StopMovement);

	/*
	 * Destructor of the class
	 */
	virtual ~Q_TrajectoryController();

};

#endif /* Q_TraJECTORYCONTROLLER_H_ */
