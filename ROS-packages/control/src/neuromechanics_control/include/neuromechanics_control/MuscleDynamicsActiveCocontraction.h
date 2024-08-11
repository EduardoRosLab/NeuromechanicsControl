/***************************************************************************
 *                        MuscleDynamicsActiveCocontraction.h              *
 *                           -------------------                           *
 * copyright            : (C) 2024 by Ignacio Abadia                       *
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

#ifndef MUSCLEDYNAMICSACTIVECOCONTRACTION_H_
#define MUSCLEDYNAMICSACTIVECOCONTRACTION_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <neuromechanics_control/AnalogCompact.h>
#include <neuromechanics_control/AnalogCompact_AgonistAntagonist.h>



class analog_comparison {
public:
	bool operator() (const neuromechanics_control::AnalogCompact lsp, const neuromechanics_control::AnalogCompact rsp) const{
		if (lsp.header.stamp>rsp.header.stamp) {
			return true;
		} else {
			return false;
		}
	}
};

/*
 * This class defines a spike-to-analog decoder.
 */
class MuscleDynamicsActiveCocontraction {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Current position subscriber
	ros::Subscriber subscriber_current_position;

	// Current velocity subscriber
	ros::Subscriber subscriber_current_velocity;

	// Desired position subscriber
	ros::Subscriber subscriber_desired_position;

	// Agonist-antagonist subscriber
	ros::Subscriber subscriber_agonist_antagonist;

	// Cocontraction subscriber
	ros::Subscriber subscriber_cocontraction;

	// Output torque publisher
	ros::Publisher publisher_torque;

	ros::Publisher publisher_agonist_antagonist_control, publisher_reflex_contribution, publisher_elastic_passive_control, publisher_viscous_passive_control;

	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of current position analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_current_position;

	// Queue of current velocity analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_current_velocity;

	// Queue of desired position analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_desired_position;

	// Queue of agonist analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_agonist_signal;

	// Queue of antagonist analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_antagonist_signal;

	// Joint list
	std::vector<std::string> joint_list;

	// Muscle gain
	std::vector<double> alpha;

	// Muscle stiffness gain
	std::vector<double> beta;

	// Muscle tonic stiffness
	std::vector<double> gamma;

	// Muscle damping
	std::vector<double> delta;

	// Cocontraction
	std::vector<double> cocontraction, baseline_cocontraction;

	// Joint resting position
	std::vector<double> resting_position;

	// Joint max torque
	std::vector<double> max_torque;

	// Current values
	std::vector<double> current_position, current_velocity, desired_position, agonist_signal, antagonist_signal;

	// Value of the output variable (positive and negative)
	std::vector<double> output_torque;

	// Max position deviation
	std::vector<double> max_deviation;

	// Reflex gain & cocontraction weight
	std::vector<double> reflex_gain, cocontraction_weight;

	std::vector<double> agonist_antagonist_control, reflex_contribution, elastic_passive_control, viscous_passive_control;


	// Sensorial delay to match desired & current position
	double delay;

	// Last time when the decoder has been updated
	double last_time_current_pos, last_time_current_vel, last_time_desired_pos, last_time_input_control, last_time_agonist, last_time_antagonist;

	// Last cocontraction value used
	std::vector<double> last_cocontraction;

	double sampling_frequency;

	// Callback function for reading input activity
	void CurrentPositionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void CurrentVelocityCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void DesiredPositionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void AgonistAntagonistCallback(const neuromechanics_control::AnalogCompact_AgonistAntagonist::ConstPtr& msg);

	// Callback function for reading input activity
	void CocontractionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Clean the input value queue and retrieve the last value
	void CleanQueue(std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > & queue,
			std::vector<double> & updateVar,
			double & lastTime,
			double end_time);

	// Find the index of name in vector strvector. -1 if not found
	int FindJointIndex(std::vector<std::string> strvector, std::string name);

public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	MuscleDynamicsActiveCocontraction(std::string input_agonist_antagonist_topic,
		std::string current_position_topic,
		std::string current_velocity_topic,
		std::string desired_position_topic,
		std::string output_torque_topic,
		std::vector<std::string> joint_list,
		std::vector<double> muscle_gain,
		std::vector<double> muscle_stiffness_gain,
		std::vector<double> muscle_tonic_stiffness,
		std::vector<double> damping,
		std::vector<double> cocontraction,
		std::string cocontraction_input_topic,
		std::vector<double> resting_position,
		std::vector<double> max_torque,
		double sampling_frequency,
		double delay,
		std::vector<double> max_deviation,
		std::string output_agonist_antagonist_control_topic,
		std::string output_reflex_contribution_topic,
		std::string output_elastic_passive_control_topic,
		std::string output_viscous_passive_control_topic);

	/*
	 * Update the output variables with the current time and the output activity and send the output message
	 */
	void UpdateTorque(ros::Time current_time);

	/*
	 * Destructor of the class
	 */
	virtual ~MuscleDynamicsActiveCocontraction();

};

#endif /* MUSCLEDYNAMICSACTIVECOCONTRACTION_H_ */
