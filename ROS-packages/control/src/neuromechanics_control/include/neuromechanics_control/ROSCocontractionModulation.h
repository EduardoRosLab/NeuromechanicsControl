/***************************************************************************
 *                      ROSCocontractionModulation.h                       *
 *                           -------------------                           *
 * copyright            : (C) 2024 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es 			                             *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef ROSCOCONTRACTIONMODULATION_H_
#define ROSCOCONTRACTIONMODULATION_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <neuromechanics_control/AnalogCompact.h>


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
class ROSCocontractionModulation {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Desired position subscriber
	ros::Subscriber subscriber_pos_plus;

	// Actual position subscriber
	ros::Subscriber subscriber_pos_minus;

	// Desired velocity subscriber
	ros::Subscriber subscriber_vel_plus;

	// Actual velocity subscriber
	ros::Subscriber subscriber_vel_minus;

	// Output publisher positive and negative error
	ros::Publisher publisher;

	ros::Publisher publisher_mean;


	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of desired position analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_pos_plus;

	// Queue of actual position analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_pos_minus;

	// Queue of desired velocity analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_vel_plus;

	// Queue of actual velocity analog signals not processed yet
	std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > activity_queue_vel_minus;

	// Position error gain
	std::vector<double> error_threshold;

	// Compliance-Robustness weighted trade-off 
	std::vector<double> w_compliance_robustness;

	// Position error gain
	std::vector<double> max_output_var;

	std::vector<double> last_error_pos;


	// Joint list
	std::vector<std::string> joint_list;

	std::vector<std::string> joint_list_mean;

	// Current values
	std::vector<double> position_plus, position_minus, velocity_plus, velocity_minus;

	// Value of the output variable (positive and negative)
	std::vector<double> output_var;

	std::vector<double> output_var_mean;

	// Last time when the decoder has been updated
	double last_time_pos_plus, last_time_pos_minus, last_time_vel_plus, last_time_vel_minus;;

	// Callback function for reading input activity
	void PositionPlusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void PositionMinusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void VelocityPlusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

	// Callback function for reading input activity
	void VelocityMinusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg);

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
	ROSCocontractionModulation(std::string pos_plus_topic_name,
			std::string pos_minus_topic_name,
			std::string vel_plus_topic_name,
			std::string vel_minus_topic_name,
			std::string output_topic_name,
			std::string output_topic_mean_name,
			std::vector<std::string> joint_list,
			std::vector<std::string> joint_list_mean,
			std::vector<double> error_threshold,
			std::vector<double> w_compliance_robustness,
			std::vector<double> max_cocontraction);

	/*
	 * Update the output variables with the current time and the output activity and send the output message
	 */
	void UpdateCocontraction(ros::Time current_time);

	/*
	 * Destructor of the class
	 */
	virtual ~ROSCocontractionModulation();

};

#endif /* ROSCOCONTRACTIONMODULATION_H_ */
