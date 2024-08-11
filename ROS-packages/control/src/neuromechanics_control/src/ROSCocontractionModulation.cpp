/***************************************************************************
 *                           ROSCocontractionModulation.cpp                *
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

// This node computes the cocontraction leve comparing the current and desired position

#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>

#include "neuromechanics_control/ROSCocontractionModulation.h"
#include "neuromechanics_control/AnalogCompact.h"

void ROSCocontractionModulation::PositionPlusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the msg should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_pos_plus){
		ROS_WARN("Cocontraction modulation: Positive position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_pos_plus.push(newMessage);
	}
}

void ROSCocontractionModulation::PositionMinusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the msg should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_pos_minus){
		ROS_WARN("Cocontraction modulation: Actual position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_pos_minus.push(newMessage);
	}
}

void ROSCocontractionModulation::VelocityPlusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_vel_plus){
		ROS_WARN("Cocontraction modulation: Desired velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_vel_plus.push(newMessage);
	}
}

void ROSCocontractionModulation::VelocityMinusCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_vel_minus){
		ROS_WARN("Cocontraction modulation: Actual velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_vel_minus.push(newMessage);
	}
}

int ROSCocontractionModulation::FindJointIndex(std::vector<std::string> strvector, std::string name){
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

ROSCocontractionModulation::ROSCocontractionModulation(
	std::string pos_plus_topic_name,
	std::string pos_minus_topic_name,
	std::string vel_plus_topic_name,
	std::string vel_minus_topic_name,
	std::string output_topic_name,
	std::string output_topic_mean_name,
	std::vector<std::string> joint_list,
	std::vector<std::string> joint_list_mean,
	std::vector<double> error_threshold,
	std::vector<double> w_compliance_robustness,
	std::vector<double> max_cocontraction):
				NodeHandler(),
				CallbackQueue(),
				activity_queue_pos_plus(),
				activity_queue_pos_minus(),
				joint_list(joint_list),
				joint_list_mean(joint_list_mean),
				last_time_pos_plus(0.0),
				last_time_pos_minus(0.0),
				last_time_vel_plus(0.0),
				last_time_vel_minus(0.0),
				error_threshold(error_threshold),
				w_compliance_robustness(w_compliance_robustness),
				max_output_var(max_cocontraction){

	this->position_plus = std::vector<double> (joint_list.size(), 0.0);
	this->position_minus = std::vector<double> (joint_list.size(), 0.0);
	this->velocity_plus = std::vector<double> (joint_list.size(), 0.0);
	this->velocity_minus = std::vector<double> (joint_list.size(), 0.0);
	this->output_var = std::vector<double> (joint_list.size(), 0.0);
	this->output_var_mean =  std::vector<double> (1, 0.0);

	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);
	this->subscriber_pos_plus = this->NodeHandler.subscribe(pos_plus_topic_name, 10.0, &ROSCocontractionModulation::PositionPlusCallback, this);
	this->subscriber_pos_minus = this->NodeHandler.subscribe(pos_minus_topic_name, 10.0, &ROSCocontractionModulation::PositionMinusCallback, this);
	this->subscriber_vel_plus = this->NodeHandler.subscribe(vel_plus_topic_name, 10.0, &ROSCocontractionModulation::VelocityPlusCallback, this);
	this->subscriber_vel_minus = this->NodeHandler.subscribe(vel_minus_topic_name, 10.0, &ROSCocontractionModulation::VelocityMinusCallback, this);

	this->publisher = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_topic_name, 1.0);
	this->publisher_mean = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_topic_mean_name, 1.0);

}

ROSCocontractionModulation::~ROSCocontractionModulation() {
	// TODO Auto-generated destructor stub
}

void ROSCocontractionModulation::UpdateCocontraction(ros::Time current_time){

	// Process all the spikes in the queue
	this->CallbackQueue.callAvailable();

	double end_time = current_time.toSec();

	this->CleanQueue(this->activity_queue_pos_plus, this->position_plus, this->last_time_pos_plus, end_time);
	this->CleanQueue(this->activity_queue_pos_minus, this->position_minus, this->last_time_pos_minus, end_time);
	this->CleanQueue(this->activity_queue_vel_plus, this->velocity_plus, this->last_time_vel_plus, end_time);
	this->CleanQueue(this->activity_queue_vel_minus, this->velocity_minus, this->last_time_vel_minus, end_time);

	std::vector<double> ErrorPos(this->joint_list.size());
	// std::vector<double> ErrorVel(this->joint_list.size());

	for (unsigned int i=0; i<this->joint_list.size(); ++i){
		ErrorPos[i] = std::abs(this->position_plus[i] - this->position_minus[i]);
		// ErrorVel[i] = std::abs(this->velocity_plus[i] - this->velocity_minus[i]);
		if (ErrorPos[i] < this->error_threshold[i]){
			this->output_var[i] = 0.0;
		}
		else{
			this->output_var[i] = 20*pow(this->w_compliance_robustness[i],3) * (ErrorPos[i])/(exp((-2*this->w_compliance_robustness[i])));
		}
		if (this->output_var[i] < 0.0){
			this->output_var[i] = 0.0;
		}
		if (this->output_var[i] > this->max_output_var[i]){
			this->output_var[i] = this->max_output_var[i];
		}
		this->output_var_mean[0] += this->output_var[i];
	}
	this->output_var_mean[0] = this->output_var_mean[0] / this->joint_list.size();
	// Create the message and publish it
	neuromechanics_control::AnalogCompact newMsg;
	newMsg.header.stamp = current_time;
	newMsg.names = this->joint_list;
	newMsg.data = this->output_var;
	this->publisher.publish(newMsg);

	neuromechanics_control::AnalogCompact newMsg_mean;
	newMsg.header.stamp = current_time;
	newMsg.names = this->joint_list_mean;
	newMsg.data = this->output_var_mean;
	this->publisher_mean.publish(newMsg);
	this->output_var_mean[0] = 0.0;

	return;
}

void ROSCocontractionModulation::CleanQueue(std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > & queue,
		std::vector<double> & updateVar,
		double & lastTime,
		double end_time){
	neuromechanics_control::AnalogCompact top_value;
	// Clean the queue (just in case any spike has to be discarded in the queue)
	if (!queue.empty()){
		top_value= queue.top();
		while (!(queue.empty()) && top_value.header.stamp.toSec()<lastTime){
			queue.pop();
			ROS_WARN("Cocontraction modulation: Discarded analog value from queue with time %f. Current time: %f", top_value.header.stamp.toSec(), ros::Time::now().toSec());
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
	}

	if (!queue.empty()){
		while (!(queue.empty()) && top_value.header.stamp.toSec()<=end_time){
			for (unsigned int i=0; i<top_value.names.size(); ++i){
				int index = this->FindJointIndex(this->joint_list, top_value.names[i]);

				if (index!=-1) {
					updateVar[index] = top_value.data[i];
				}
			}
			lastTime = top_value.header.stamp.toSec();
			queue.pop();
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
	}
}
