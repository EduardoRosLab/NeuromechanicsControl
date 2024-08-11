/***************************************************************************
 *                  MuscleDynamicsCocontraction_WithPD_Ablation.cpp        *
 *                           -------------------                           *
 * copyright            : (C) 2024 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es  			                           *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>

#include "neuromechanics_control/MuscleDynamicsCocontraction_WithPD_Ablation.h"
#include "neuromechanics_control/AnalogCompact.h"
#include "neuromechanics_control/AnalogCompact_AgonistAntagonist.h"


void MuscleDynamicsCocontraction_WithPD_Ablation::CurrentPositionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the signal should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_current_pos){
		ROS_WARN("Muscle Dynamics: Current position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_current_position.push(newMessage);
	}
}

void MuscleDynamicsCocontraction_WithPD_Ablation::CurrentVelocityCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the signal should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_current_vel){
		ROS_WARN("Muscle Dynamics: Current velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_current_velocity.push(newMessage);
	}
}

void MuscleDynamicsCocontraction_WithPD_Ablation::DesiredPositionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the signal should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_desired_pos){
		ROS_WARN("Muscle Dynamics: Desired position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_desired_position.push(newMessage);
	}
}

void MuscleDynamicsCocontraction_WithPD_Ablation::DesiredVelocityCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	// Check if the signal should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_desired_vel){
		ROS_WARN("Muscle Dynamics: Desired velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		neuromechanics_control::AnalogCompact newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		this->activity_queue_desired_velocity.push(newMessage);
	}
}

// Callback for cocontraction
void MuscleDynamicsCocontraction_WithPD_Ablation::CocontractionCallback(const neuromechanics_control::AnalogCompact::ConstPtr& msg){
	if (msg->data[0] != this->last_cocontraction){
		for (unsigned int i=0; i<this->joint_list.size(); ++i){
			this->cocontraction[i] = this->baseline_cocontraction[i] * msg->data[0];
			// if (this->cocontraction[i] > 1.0){
			// 	this->cocontraction[i] = 1.0;
			// }
			this->reflex_gain[i] = this->cocontraction_weight[i] * this->cocontraction[i];
			ROS_INFO("NEW REFLEX GAIN = %f", this->reflex_gain[i]);
		}
		this->last_cocontraction = msg->data[0];
	}
}

int MuscleDynamicsCocontraction_WithPD_Ablation::FindJointIndex(std::vector<std::string> strvector, std::string name){
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


MuscleDynamicsCocontraction_WithPD_Ablation::MuscleDynamicsCocontraction_WithPD_Ablation(
	std::vector<double> kp_gain,
	std::vector<double> kd_gain,
	double PD_min_output,
	double PD_max_output,
	std::string current_position_topic,
	std::string current_velocity_topic,
	std::string desired_position_topic,
	std::string desired_velocity_topic,
	std::string output_torque_topic,
	std::vector<std::string> joint_list,
	std::vector<double> muscle_gain,
	std::vector<double> muscle_gain_reflex,
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
	std::string output_viscous_passive_control_topic):
				NodeHandler(),
				CallbackQueue(),
				joint_list(joint_list),
				alpha(muscle_gain),
				alpha_reflex(muscle_gain_reflex),
				beta(muscle_stiffness_gain),
				gamma(muscle_tonic_stiffness),
				delta(damping),
				cocontraction(cocontraction),
				baseline_cocontraction(cocontraction),
				resting_position(resting_position),
				max_torque(max_torque),
				sampling_frequency(sampling_frequency),
				delay(delay),
				last_time_current_pos(0.0),
				last_time_current_vel(0.0),
				last_time_desired_pos(0.0),
				last_time_input_control(0.0),
				last_time_agonist(0.0),
				last_time_antagonist(0.0),
				max_deviation(max_deviation),
				kp_gain(kp_gain),
				kd_gain(kd_gain),
				PD_min_output(PD_min_output),
				PD_max_output(PD_max_output)
{

	this->current_position = std::vector<double> (joint_list.size(), 0.0);
	this->current_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->desired_position = std::vector<double> (joint_list.size(), 0.0);
	this->desired_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->pd_output = std::vector<double> (joint_list.size(), 0.0);
	this->output_torque = std::vector<double> (joint_list.size(), 0.0);

	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);
	this->subscriber_current_position = this->NodeHandler.subscribe(current_position_topic, 10.0, &MuscleDynamicsCocontraction_WithPD_Ablation::CurrentPositionCallback, this);
	this->subscriber_current_velocity = this->NodeHandler.subscribe(current_velocity_topic, 10.0, &MuscleDynamicsCocontraction_WithPD_Ablation::CurrentVelocityCallback, this);
	this->subscriber_desired_position = this->NodeHandler.subscribe(desired_position_topic, 10.0, &MuscleDynamicsCocontraction_WithPD_Ablation::DesiredPositionCallback, this);
	this->subscriber_desired_velocity = this->NodeHandler.subscribe(desired_velocity_topic, 10.0, &MuscleDynamicsCocontraction_WithPD_Ablation::DesiredVelocityCallback, this);

	this->subscriber_cocontraction = this->NodeHandler.subscribe(cocontraction_input_topic, 10.0, &MuscleDynamicsCocontraction_WithPD_Ablation::CocontractionCallback, this);

	this->publisher_torque = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_torque_topic, 1.0);

	this->publisher_agonist_antagonist_control = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_agonist_antagonist_control_topic, 1.0);
	this->publisher_reflex_contribution = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_reflex_contribution_topic, 1.0);

	this->publisher_elastic_passive_control = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_elastic_passive_control_topic, 1.0);
	this->publisher_viscous_passive_control = this->NodeHandler.advertise<neuromechanics_control::AnalogCompact>(output_viscous_passive_control_topic, 1.0);


	this->last_cocontraction = 1.0;

	this->cocontraction_weight = std::vector<double> (joint_list.size(), 0.0);
	this->reflex_gain = std::vector<double> (joint_list.size(), 0.0);

	// Normalize stiffness gain (k) for the joint range of motion and initialize reflex gain
	for (unsigned int i=0; i<this->joint_list.size(); ++i){
		// this->cocontraction_weight[i] = 1.0/(this->max_position[i] - this->min_position[i]);
		this->cocontraction_weight[i] = 1.0/(this->max_deviation[i]);
		this->reflex_gain[i] = this->cocontraction_weight[i] * this->cocontraction[i];
	}


	this->agonist_antagonist_control = std::vector<double> (joint_list.size(), 0.0);
	this->reflex_contribution = std::vector<double> (joint_list.size(), 0.0);
	this->elastic_passive_control = std::vector<double> (joint_list.size(), 0.0);
	this->viscous_passive_control = std::vector<double> (joint_list.size(), 0.0);

}

MuscleDynamicsCocontraction_WithPD_Ablation::~MuscleDynamicsCocontraction_WithPD_Ablation() {
	// TODO Auto-generated destructor stub
}

void MuscleDynamicsCocontraction_WithPD_Ablation::UpdateTorque(ros::Time current_time){

	// Process all the input signals in the queue
	this->CallbackQueue.callAvailable();

	double end_time = current_time.toSec();

	this->CleanQueue(this->activity_queue_current_position, this->current_position, this->last_time_current_pos, end_time);
	this->CleanQueue(this->activity_queue_current_velocity, this->current_velocity, this->last_time_current_vel, end_time);
	this->CleanQueue(this->activity_queue_desired_position, this->desired_position, this->last_time_desired_pos, end_time - this->delay);
	this->CleanQueue(this->activity_queue_desired_velocity, this->desired_velocity, this->last_time_desired_vel, end_time - this->delay);



	// MUSCLE DYNAMICS WITH PD CONTROL
	// // // // // // // // // // // // //
	// // // // // // // // // // // // //
	for (unsigned int i=0; i<this->joint_list.size(); ++i){

		// PD control for the active component
		this->pd_output[i] = this->kp_gain[i]*(this->desired_position[i] - this->current_position[i]) + this->kd_gain[i]*(this->desired_velocity[i] - this->current_velocity[i]);
		if (this->pd_output[i] > this->PD_max_output){
				this->pd_output[i] = this->PD_max_output;
		}
		else if (this->pd_output[i] < this->PD_min_output){
				this->pd_output[i] = this->PD_min_output;
		}

		this->agonist_antagonist_control[i] = this->alpha[i]*(this->pd_output[i]);


		this->reflex_contribution[i] = this->reflex_gain[i]*(this->desired_position[i] - this->current_position[i]);

		if (this->reflex_contribution[i] > 1.0){
			this->reflex_contribution[i] = 1.0;
		}
		else if (this->reflex_contribution[i] < -1.0){
			this->reflex_contribution[i] = -1.0;
		}
		this->reflex_contribution[i] *= this->alpha_reflex[i];

		this->elastic_passive_control[i] = this->beta[i]*(2*this->cocontraction[i] + this->gamma[i])*(this->resting_position[i] - this->current_position[i]);
		this->viscous_passive_control[i] = this->delta[i]*this->current_velocity[i];

		this->output_torque[i] = this->agonist_antagonist_control[i] + this->reflex_contribution[i] + this->elastic_passive_control[i] + this->viscous_passive_control[i];


		// this->output_torque[i] = this->alpha[i]*(this->agonist_signal[i] - this->antagonist_signal[i]) + this->beta[i]*this->gamma[i]*(this->resting_position[i] - this->current_position[i]) + this->beta[i]*(this->agonist_signal[i] + this->antagonist_signal[i] + this->cocontraction[i])*(this->desired_position[i] - this->current_position[i]) + this->delta[i]*this->current_velocity[i];
		if (this->output_torque[i] > this->max_torque[i]){
			this->output_torque[i] = this->max_torque[i];
		}
		else if (this->output_torque[i] < -1.0*this->max_torque[i]){
			this->output_torque[i] = -1.0*this->max_torque[i];
		}

	}
	 // // // // // // // // // // // // //
	 // // // // // // // // // // // // //


	// Create the message and publish it
	neuromechanics_control::AnalogCompact msgTorque, msgAgonistAntagonist, msgReflex, msgElastic, msgViscous;
	msgTorque.header.stamp = msgAgonistAntagonist.header.stamp = msgReflex.header.stamp = msgElastic.header.stamp = msgViscous.header.stamp = current_time;
	msgTorque.names = msgAgonistAntagonist.names = msgReflex.names = msgElastic.names = msgViscous.names = this->joint_list;
	msgTorque.data = this->output_torque;
	this->publisher_torque.publish(msgTorque);

	msgAgonistAntagonist.data = this->agonist_antagonist_control;
	msgReflex.data = this->reflex_contribution;
	msgElastic.data = this->elastic_passive_control;
	msgViscous.data = this->viscous_passive_control;
	this->publisher_agonist_antagonist_control.publish(msgAgonistAntagonist);
	this->publisher_reflex_contribution.publish(msgReflex);
	this->publisher_elastic_passive_control.publish(msgElastic);
	this->publisher_viscous_passive_control.publish(msgViscous);

	return;
}

void MuscleDynamicsCocontraction_WithPD_Ablation::CleanQueue(std::priority_queue<neuromechanics_control::AnalogCompact, std::vector<neuromechanics_control::AnalogCompact>, analog_comparison > & queue,
		std::vector<double> & updateVar,
		double & lastTime,
		double end_time){
	neuromechanics_control::AnalogCompact top_value;
	// Clean the queue (just in case any spike has to be discarded in the queue)
	if (!queue.empty()){
		top_value= queue.top();
		while (!(queue.empty()) && top_value.header.stamp.toSec()<lastTime){
			queue.pop();
			ROS_WARN("Muscle Dynamics: Discarded analog value from queue with time %f. Current time: %f", top_value.header.stamp.toSec(), ros::Time::now().toSec());
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
	}

	// ROS_INFO("QUEUE VALUES, end time = %f", end_time);
	if (!queue.empty()){
		while (!(queue.empty()) && top_value.header.stamp.toSec()<=end_time){
			// ROS_INFO("message time = %f", top_value.header.stamp.toSec());
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
				// ROS_INFO("next message time = %f", top_value.header.stamp.toSec());
			}
		}
	}
}
