/***************************************************************************
 *                           AdExTimeDrivenModel_1_2_GPU.cu                *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Francisco Naveros                    *
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

#include "../../include/neuron_model/AdExTimeDrivenModel_GPU.h"
#include "../../include/neuron_model/AdExTimeDrivenModel_GPU2.h"
#include "../../include/neuron_model/VectorNeuronState.h"
#include "../../include/neuron_model/VectorNeuronState_GPU.h"

#include <iostream>
#include <cmath>
#include <string>

#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"

#include "../../include/cudaError.h"
//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

void AdExTimeDrivenModel_GPU::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline = 1L;
		skip_comments(fh, Currentline);
		if (fscanf(fh, "%f", &this->a) == 1){
			skip_comments(fh, Currentline);
			if (fscanf(fh, "%f", &this->b) == 1){
				skip_comments(fh, Currentline);
				if (fscanf(fh, "%f", &this->TSF) == 1 && this->TSF > 0.0){
					skip_comments(fh, Currentline);
					if (fscanf(fh, "%f", &this->VT) == 1){
						skip_comments(fh, Currentline);
						if (fscanf(fh, "%f", &this->tauw) == 1 && this->tauw > 0.0f){
							skip_comments(fh, Currentline);
							if (fscanf(fh, "%f", &this->eexc) == 1){
								skip_comments(fh, Currentline);
								if (fscanf(fh, "%f", &this->einh) == 1){
									skip_comments(fh, Currentline);
									if (fscanf(fh, "%f", &this->Ereset) == 1){
										skip_comments(fh, Currentline);
										if (fscanf(fh, "%f", &this->Eleak) == 1){
											skip_comments(fh, Currentline);
											if (fscanf(fh, "%f", &this->gleak) == 1 && this->gleak > 0.0f){
												skip_comments(fh, Currentline);
												if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
													skip_comments(fh, Currentline);
													if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
														skip_comments(fh, Currentline);
														if (fscanf(fh, "%f", &this->tinh) == 1 && this->tinh > 0.0f){
															skip_comments(fh, Currentline);

															this->State = (VectorNeuronState_GPU *) new VectorNeuronState_GPU(N_NeuronStateVariables);

														}else {
															throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
														}
													}else {
														throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
													}
												}else {
													throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
												}
											}else {
												throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_GLEAK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
											}
										}else {
											throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_ELEAK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_ERESET, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TAUW, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_VT, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TSF, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_B, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_A, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

  		//INTEGRATION METHOD
		this->integrationMethod_GPU=LoadIntegrationMethod_GPU::loadIntegrationMethod_GPU((TimeDrivenNeuronModel_GPU *)this, this->GetModelID(), fh, &Currentline, N_NeuronStateVariables, N_DifferentialNeuronState, N_TimeDependentNeuronState);

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod_GPU->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

AdExTimeDrivenModel_GPU::AdExTimeDrivenModel_GPU(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel_GPU(NeuronTypeID, NeuronModelID, MilisecondScale){
		//a=1.0f;  //nS
		//b=9.0f;  //pA
		//TSF=2.0f;  //mV
		//VT=-50.0f;//mV
		//tauw=50.0f ; //ms   
		//eexc=0.0f;//mV
		//einh=-80.0f;//mV
		//Ereset=-80.0f;//mV
		//Eleak=-65.0f;//mV
		//gleak=10;//nS
		//cm=110.0f;//pF
		//texc=5.0f;//ms
		//tinh=10.0f;//ms
}

AdExTimeDrivenModel_GPU::~AdExTimeDrivenModel_GPU(void){
	DeleteClassGPU2();
}

void AdExTimeDrivenModel_GPU::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * AdExTimeDrivenModel_GPU::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * AdExTimeDrivenModel_GPU::ProcessInputSpike(Interconnection * inter, double time){
	this->State_GPU->AuxStateCPU[inter->GetType()*State_GPU->GetSizeState() + inter->GetTargetNeuronModelIndex()] += inter->GetWeight();

	return 0;
}


__global__ void AdExTimeDrivenModel_GPU_UpdateState(AdExTimeDrivenModel_GPU2 ** NeuronModel_GPU2, double CurrentTime){
	(*NeuronModel_GPU2)->UpdateState(CurrentTime);
}

		
bool AdExTimeDrivenModel_GPU::UpdateState(int index, double CurrentTime){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;

	//----------------------------------------------
	if(prop.canMapHostMemory){
		AdExTimeDrivenModel_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
	}else{
		HANDLE_ERROR(cudaMemcpy(state->AuxStateGPU,state->AuxStateCPU,this->N_TimeDependentNeuronState*state->SizeStates*sizeof(float),cudaMemcpyHostToDevice));
		AdExTimeDrivenModel_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
		HANDLE_ERROR(cudaMemcpy(state->InternalSpikeCPU,state->InternalSpikeGPU,state->SizeStates*sizeof(bool),cudaMemcpyDeviceToHost));
	}


	if(this->GetVectorNeuronState()->Get_Is_Monitored()){
		HANDLE_ERROR(cudaMemcpy(state->VectorNeuronStates,state->VectorNeuronStates_GPU,state->GetNumberOfVariables()*state->SizeStates*sizeof(float),cudaMemcpyDeviceToHost));
		HANDLE_ERROR(cudaMemcpy(state->LastUpdate,state->LastUpdateGPU,state->SizeStates*sizeof(double),cudaMemcpyDeviceToHost));
		HANDLE_ERROR(cudaMemcpy(state->LastSpikeTime,state->LastSpikeTimeGPU,state->SizeStates*sizeof(double),cudaMemcpyDeviceToHost));
	}
 

	HANDLE_ERROR(cudaEventRecord(stop, 0)); 
	HANDLE_ERROR(cudaEventSynchronize(stop));


	memset(state->AuxStateCPU,0,N_TimeDependentNeuronState*state->SizeStates*sizeof(float));

	return false;

}


enum NeuronModelOutputActivityType AdExTimeDrivenModel_GPU::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType AdExTimeDrivenModel_GPU::GetModelInputActivityType(){
	return INPUT_SPIKE;
}

ostream & AdExTimeDrivenModel_GPU::PrintInfo(ostream & out){
	return out;
}	


void AdExTimeDrivenModel_GPU::InitializeStates(int N_neurons, int OpenMPQueueIndex){

	//Select the correnpondent device. 
	HANDLE_ERROR(cudaSetDevice(GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));  
	HANDLE_ERROR(cudaEventCreate(&stop));
	HANDLE_ERROR(cudaGetDeviceProperties( &prop, GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));

	this->State_GPU = (VectorNeuronState_GPU *) this->State;
	
	//Initialize neural state variables.
	
	float initialization[] = {Eleak,0.0f,0.0f,0.0f};
	State_GPU->InitializeStatesGPU(N_neurons, initialization, N_TimeDependentNeuronState, prop);

	//INITIALIZE CLASS IN GPU
	this->InitializeClassGPU2(N_neurons);


	InitializeVectorNeuronState_GPU2();
}



__global__ void AdExTimeDrivenModel_GPU_InitializeClassGPU2(AdExTimeDrivenModel_GPU2 ** NeuronModel_GPU2, float a, float b, float TSF, 
		float VT, float tauw, float eexc, float einh, float Ereset, float Eleak, float gleak, float cm, float texc, float tinh,  
		char const* integrationName, int N_neurons, void ** Buffer_GPU){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)=new AdExTimeDrivenModel_GPU2(a, b, TSF, VT, tauw, eexc, einh, Ereset, Eleak, gleak, cm, texc, tinh, 
			integrationName, N_neurons, Buffer_GPU);
	}
}

void AdExTimeDrivenModel_GPU::InitializeClassGPU2(int N_neurons){
	cudaMalloc(&NeuronModel_GPU2, sizeof(AdExTimeDrivenModel_GPU **));
	
	char * integrationNameGPU;
	cudaMalloc((void **)&integrationNameGPU,32*4);
	HANDLE_ERROR(cudaMemcpy(integrationNameGPU,integrationMethod_GPU->GetType(),32*4,cudaMemcpyHostToDevice));

	this->N_thread = 128;
	this->N_block=prop.multiProcessorCount*16;
	if((N_neurons+N_thread-1)/N_thread < N_block){
		N_block = (N_neurons+N_thread-1)/N_thread;
	}
	int Total_N_thread=N_thread*N_block;

	integrationMethod_GPU->InitializeMemoryGPU(N_neurons, Total_N_thread);

	AdExTimeDrivenModel_GPU_InitializeClassGPU2<<<1,1>>>(NeuronModel_GPU2, a, b, TSF, VT, tauw, eexc, einh, Ereset, Eleak, gleak, cm, 
		texc, tinh, integrationNameGPU, N_neurons, integrationMethod_GPU->Buffer_GPU);

	cudaFree(integrationNameGPU);
}



__global__ void initializeVectorNeuronState_GPU2(AdExTimeDrivenModel_GPU2 ** NeuronModel_GPU2, int NumberOfVariables, float * InitialStateGPU, float * AuxStateGPU, float * StateGPU, double * LastUpdateGPU, double * LastSpikeTimeGPU, bool * InternalSpikeGPU, int SizeStates){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)->InitializeVectorNeuronState_GPU2(NumberOfVariables, InitialStateGPU, AuxStateGPU, StateGPU, LastUpdateGPU, LastSpikeTimeGPU, InternalSpikeGPU, SizeStates);
	}
}

void AdExTimeDrivenModel_GPU::InitializeVectorNeuronState_GPU2(){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;
	initializeVectorNeuronState_GPU2<<<1,1>>>(NeuronModel_GPU2, state->NumberOfVariables, state->InitialStateGPU, state->AuxStateGPU, state->VectorNeuronStates_GPU, state->LastUpdateGPU, state->LastSpikeTimeGPU, state->InternalSpikeGPU, state->SizeStates);
}


__global__ void DeleteClass_GPU2(AdExTimeDrivenModel_GPU2 ** NeuronModel_GPU2){
	if(blockIdx.x==0 && threadIdx.x==0){
		delete (*NeuronModel_GPU2); 
	}
}


void AdExTimeDrivenModel_GPU::DeleteClassGPU2(){
    DeleteClass_GPU2<<<1,1>>>(NeuronModel_GPU2);
    cudaFree(NeuronModel_GPU2);
}


bool AdExTimeDrivenModel_GPU::CheckSynapseType(Interconnection * connection){
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