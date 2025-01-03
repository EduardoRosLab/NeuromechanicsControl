/***************************************************************************
 *                           Lethargic_LIFTimeDrivenModel_1_1.cpp          *
 *                           -------------------                           *
 * copyright            : (C) 2018 by Francisco Naveros                    *
 * email                : fnaveros@ugr.es                                  *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "../../include/neuron_model/Lethargic_LIFTimeDrivenModel_1_1.h"
#include "../../include/neuron_model/VectorNeuronState.h"
#include "../../include/neuron_model/CurrentSynapseModel.h"


#include <iostream>
#include <cmath>
#include <string>

#include "../../include/openmp/openmp.h"

#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"


void Lethargic_LIFTimeDrivenModel_1_1::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline=1L;
		skip_comments(fh,Currentline);
		if(fscanf(fh,"%f",&this->eexc)==1){
			skip_comments(fh,Currentline);

			if(fscanf(fh,"%f",&this->erest)==1){
				skip_comments(fh,Currentline);

				if(fscanf(fh,"%f",&this->vthr)==1){
					skip_comments(fh,Currentline);

					if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
						//Conversion from F to nF
						this->cm*=1e9;
						inv_cm=1.0f/cm;
						skip_comments(fh,Currentline);

						if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
							inv_texc=1.0f/texc;
							skip_comments(fh,Currentline);

							if (fscanf(fh, "%f", &this->tref) == 1 && this->tref >= 0.0f){
								skip_comments(fh,Currentline);

								if (fscanf(fh, "%f", &this->grest) == 1 && this->grest > 0.0f){
									//Conversion from S to nS
									this->grest*=1e9;

									skip_comments(fh,Currentline);

									this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

								} else {
									throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_GREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							} else {
								throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_TREF, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						} else {
							throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					} else {
						throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				} else {
					throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_VTHR, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			} else {
				throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_EREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		} else {
			throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters((erest+vthr)/2,(erest+vthr)/2,0);
		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);

	}else{
		throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_2_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a second scale.
Lethargic_LIFTimeDrivenModel_1_1::Lethargic_LIFTimeDrivenModel_1_1(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, SecondScale), eexc(0), erest(0), vthr(0), cm(0), texc(0),
		tref(0), grest(0){
}

Lethargic_LIFTimeDrivenModel_1_1::~Lethargic_LIFTimeDrivenModel_1_1(void)
{
	delete this->integration_required;
}

void Lethargic_LIFTimeDrivenModel_1_1::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * Lethargic_LIFTimeDrivenModel_1_1::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * Lethargic_LIFTimeDrivenModel_1_1::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), inter->GetWeight());
	
	//The integration is required after increment a conductance.
	if (this->integration_required[inter->GetTargetNeuronModelIndex()] == false){
		this->integration_required[inter->GetTargetNeuronModelIndex()] = true;
		this->SolveDifferentialEquation(inter->GetTargetNeuronModelIndex(), time);
	}

	return 0;
}


bool Lethargic_LIFTimeDrivenModel_1_1::UpdateState(int index, double CurrentTime){

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;


	this->integrationMethod->NextDifferentialEquationValues(this->integration_required, CurrentTime);

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType Lethargic_LIFTimeDrivenModel_1_1::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType Lethargic_LIFTimeDrivenModel_1_1::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & Lethargic_LIFTimeDrivenModel_1_1::PrintInfo(ostream & out){
	out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	out << "\tExc. Reversal Potential: " << this->eexc << "V\tResting potential: " << this->erest << "V" << endl;

	out << "\tFiring threshold: " << this->vthr << "V\tMembrane capacitance: " << this->cm << "F\tExcitatory Time Constant: " << this->texc << "s" << endl;

	out << "\tRefractory Period: " << this->tref << "s\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}



void Lethargic_LIFTimeDrivenModel_1_1::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float initialization[] = {erest,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);

	//Initialize the integration required vector.
	this->integration_required = new bool[N_neurons];
}


//void Lethargic_LIFTimeDrivenModel_1_1::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
//	if (NeuronState[0] < this->erest){
//		NeuronState[0] = this->erest;
//    }
//	if (NeuronState[0] > this->vthr){
//		NeuronState[0] = this->erest;
//		State->NewFiredSpike(index);
//		this->integrationMethod->resetState(index);
//this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
//this->State->NInternalSpikeIndexs++;
//	}
//}
//
//void Lethargic_LIFTimeDrivenModel_1_1::SolveDifferentialEquation(int index, double CurrentTime){
//	float * NeuronState = State->GetStateVariableAt(index);
//	float elapsed_time = CurrentTime - State->GetLastUpdateTime(index);
//
//	//since the conductance is zero, the membrane potential equation can be solved analitically.
//	NeuronState[0] = this->erest - (this->erest - NeuronState[0])*expf(-elapsed_time*this->grest*this->inv_cm);
//}
//
//void Lethargic_LIFTimeDrivenModel_1_1::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
//	if(this->GetVectorNeuronState()->GetLastSpikeTime(index)>this->tref){
//		AuxNeuronState[0]=(NeuronState[1] * (this->eexc - NeuronState[0]) + grest * (this->erest - NeuronState[0]))*this->inv_cm;
//	}else if((this->GetVectorNeuronState()->GetLastSpikeTime(index)+elapsed_time)>this->tref){
//		float fraction=(this->GetVectorNeuronState()->GetLastSpikeTime(index)+elapsed_time-this->tref)/elapsed_time;
//		AuxNeuronState[0]=fraction*(NeuronState[1] * (this->eexc - NeuronState[0]) + grest * (this->erest - NeuronState[0]))*this->inv_cm;
//	}else{
//		AuxNeuronState[0]=0;
//	}
//}
//
//void Lethargic_LIFTimeDrivenModel_1_1::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
//	float limit=1e-9;
//	float * Conductance_values=this->Get_conductance_exponential_values(elapsed_time_index);
//
//	if(NeuronState[N_DifferentialNeuronState]<limit){
//		NeuronState[N_DifferentialNeuronState]=0.0f;
//	}else{
//		NeuronState[N_DifferentialNeuronState]*= Conductance_values[0];
//	}
//}


void Lethargic_LIFTimeDrivenModel_1_1::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, expf(-elapsed_time*this->inv_texc));
}


bool Lethargic_LIFTimeDrivenModel_1_1::CheckSynapseType(Interconnection * connection){
	int Type = connection->GetType();
	if (Type<N_TimeDependentNeuronState && Type >= 0){
		NeuronModel * model = connection->GetSource()->GetNeuronModel();
		//Synapse types that process input spikes
		if (Type < N_TimeDependentNeuronState && model->GetModelOutputActivityType() == OUTPUT_SPIKE)
			return true;
		else{
			cout << "Synapses type " << Type << " of neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " must receive spikes. The source model generates currents." << endl;
			return false;
		}
		//Synapse types that process input current
	}
	else{
		cout << "Neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " does not support input synapses of type " << Type << ". Just defined " << N_TimeDependentNeuronState << " synapses types." << endl;
		return false;
	}
}



void Lethargic_LIFTimeDrivenModel_1_1::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<Lethargic_LIFTimeDrivenModel_1_1> *) new Euler<Lethargic_LIFTimeDrivenModel_1_1>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<Lethargic_LIFTimeDrivenModel_1_1> *) new RK2<Lethargic_LIFTimeDrivenModel_1_1>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<Lethargic_LIFTimeDrivenModel_1_1> *) new RK4<Lethargic_LIFTimeDrivenModel_1_1>(this);
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<Lethargic_LIFTimeDrivenModel_1_1> *) new Bifixed_Euler<Lethargic_LIFTimeDrivenModel_1_1>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<Lethargic_LIFTimeDrivenModel_1_1> *) new Bifixed_RK2<Lethargic_LIFTimeDrivenModel_1_1>(this);

		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<Lethargic_LIFTimeDrivenModel_1_1> *) new Bifixed_RK4<Lethargic_LIFTimeDrivenModel_1_1>(this);
		}
		else{
			throw EDLUTFileException(TASK_INTEGRATION_METHOD_TYPE, ERROR_INTEGRATION_METHOD_TYPE, REPAIR_INTEGRATION_METHOD_TYPE, *Currentline, fileName.c_str(), true);
		}
	}
	else{
		throw EDLUTFileException(TASK_INTEGRATION_METHOD_TYPE, ERROR_INTEGRATION_METHOD_READ, REPAIR_INTEGRATION_METHOD_READ, *Currentline, fileName.c_str(), true);
	}

	//We load the integration method parameter.
	integrationMethod->loadParameter((TimeDrivenNeuronModel *)this, fh, Currentline, fileName.c_str());
}

