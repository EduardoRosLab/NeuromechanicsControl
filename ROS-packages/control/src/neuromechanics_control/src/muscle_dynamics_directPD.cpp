/***************************************************************************
 *                          muscle_dynamics_withPD.cpp	                   *
 *                           -------------------                           *
 * copyright            : (C) 2022 by Ignacio Abadia                       *
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


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <neuromechanics_control/AnalogCompact.h>
#include <neuromechanics_control/MuscleDynamicsCocontraction_DirectPD.h>
#include "log4cxx/logger.h"
#include "neuromechanics_control/ExternalClock.h"


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
	ros::init(argc, argv, "muscle_dynamics");
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_agonist_antagonist_topic,  current_position_topic, current_velocity_topic, desired_position_topic, desired_velocity_topic,
							output_torque_topic, cocontraction_input_topic, clock_topic, output_agonist_antagonist_control_topic,
							output_reflex_contribution_topic, output_elastic_passive_control_topic, output_viscous_passive_control_topic;
	std::vector<std::string> joint_list;
	std::vector<double> muscle_gain, muscle_gain_reflex, muscle_stiffness_gain, muscle_tonic_stiffness, damping, cocontraction, resting_position, max_torque, max_deviation;
	double sampling_frequency, delay;

	bool use_sim_time;

	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	ros::Time current_time(0.0);

	stop_node = false;

	std::vector<double> kp_gain, kd_gain;
	std::vector<double> PD_min_output, PD_max_output;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("PD_kp_gain", kp_gain);
	private_node_handle_.getParam("PD_kd_gain", kd_gain);
	private_node_handle_.getParam("PD_min_output", PD_min_output);
	private_node_handle_.getParam("PD_max_output", PD_max_output);
	private_node_handle_.getParam("current_position_topic", current_position_topic);
	private_node_handle_.getParam("current_velocity_topic", current_velocity_topic);
	private_node_handle_.getParam("desired_position_topic", desired_position_topic);
	private_node_handle_.getParam("desired_velocity_topic", desired_velocity_topic);
	private_node_handle_.getParam("output_torque_topic", output_torque_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("muscle_gain", muscle_gain);
	private_node_handle_.getParam("muscle_gain_reflex", muscle_gain_reflex);
	private_node_handle_.getParam("muscle_stiffness_gain", muscle_stiffness_gain);
	private_node_handle_.getParam("muscle_tonic_stiffness", muscle_tonic_stiffness);
	private_node_handle_.getParam("damping", damping);
	private_node_handle_.getParam("cocontraction", cocontraction);
	private_node_handle_.getParam("cocontraction_input_topic", cocontraction_input_topic);
	private_node_handle_.getParam("resting_position", resting_position);
	private_node_handle_.getParam("max_torque", max_torque);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("clock_topic", clock_topic);
	// private_node_handle_.getParam("node_name", node_name);
	private_node_handle_.getParam("delay", delay);
	private_node_handle_.getParam("max_deviation", max_deviation);
	private_node_handle_.getParam("output_agonist_antagonist_control_topic", output_agonist_antagonist_control_topic);
	private_node_handle_.getParam("output_reflex_contribution_topic", output_reflex_contribution_topic);
	private_node_handle_.getParam("output_elastic_passive_control_topic", output_elastic_passive_control_topic);
	private_node_handle_.getParam("output_viscous_passive_control_topic", output_viscous_passive_control_topic);


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
	ros::Publisher output_publisher = nh.advertise<neuromechanics_control::AnalogCompact>(output_torque_topic, 1.0);
	MuscleDynamicsCocontraction_DirectPD objMuscleDynamics(kp_gain,
																								kd_gain,
																								PD_min_output,
																								PD_max_output,
																							 	current_position_topic,
																								current_velocity_topic,
																								desired_position_topic,
																								desired_velocity_topic,
																								output_torque_topic,
																								joint_list,
																								muscle_gain,
																								muscle_gain_reflex,
																								muscle_stiffness_gain,
																								muscle_tonic_stiffness,
																								damping,
																								cocontraction,
																								cocontraction_input_topic,
																								resting_position,
																								max_torque,
																								sampling_frequency,
																								delay,
																								max_deviation,
																								output_agonist_antagonist_control_topic,
																								output_reflex_contribution_topic,
																								output_elastic_passive_control_topic,
																								output_viscous_passive_control_topic);

	ROS_INFO("Muscle dynamics node initialized: writing to topic %s with frequency %f", output_torque_topic.c_str(), sampling_frequency);

	ros::Rate rate(sampling_frequency);

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.001));
		if (use_sim_time){
			ros::Time new_time = ext_clock.GetLastConfirmedTime();
			if (new_time > current_time){
				current_time = new_time;
				ROS_DEBUG("Updating muscle dynamics torque at time %f", current_time.toSec());
				objMuscleDynamics.UpdateTorque(current_time);
				rate.sleep();
				//ROS_INFO("Error node Current time: %f ", current_time.toSec());
			}
		}
		else{
			// current_time = ros::Time::now();
			// Ensure a 2ms time step between the error samples
			current_time =  ros::Time(round(sampling_frequency * ros::Time::now().toSec()) * time_step);
			ROS_DEBUG("Updating muscle dynamics torque at time %f", current_time.toSec());
			objMuscleDynamics.UpdateTorque(current_time);
			rate.sleep();
		}
		//ROS_INFO("Error node Current time 2: %f ", current_time.toSec());
	}

	ROS_INFO("Ending error estimator node");

	ros::shutdown();
	return 0;
} // end main()
