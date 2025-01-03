/***************************************************************************
 *                           EgidioGranuleCell_TimeDriven_GPU.cu           *
 *                           -------------------                           *
 * copyright            : (C) 2013 by Francisco Naveros                    *
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

#include "../../include/neuron_model/EgidioGranuleCell_TimeDriven_GPU.h"
#include "../../include/neuron_model/EgidioGranuleCell_TimeDriven_GPU2.h"
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

void EgidioGranuleCell_TimeDriven_GPU::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		gMAXNa_f=0.013f;
		gMAXNa_r=0.0005f;
		gMAXNa_p=0.0002f;
		gMAXK_V=0.003f;
		gMAXK_A=0.004f;
		gMAXK_IR=0.0009f;
		gMAXK_Ca=0.004f;
		gMAXCa=0.00046f;
		gMAXK_sl=0.00035f;
		this->State = (VectorNeuronState_GPU *) new VectorNeuronState_GPU(N_NeuronStateVariables);

  		//INTEGRATION METHOD
		this->integrationMethod_GPU = LoadIntegrationMethod_GPU::loadIntegrationMethod_GPU((TimeDrivenNeuronModel_GPU *)this, this->GetModelID(), fh, &Currentline, N_NeuronStateVariables, N_DifferentialNeuronState, N_TimeDependentNeuronState);

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod_GPU->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_EGIDIO_GRANULE_CELL_TIME_DRIVEN_GPU_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}


EgidioGranuleCell_TimeDriven_GPU::EgidioGranuleCell_TimeDriven_GPU(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel_GPU(NeuronTypeID, NeuronModelID, MilisecondScale), gMAXNa_f(0.0f), gMAXNa_r(0.0f), gMAXNa_p(0.0f), gMAXK_V(0.0f), gMAXK_A(0.0f), gMAXK_IR(0.0f), gMAXK_Ca(0.0f),
		gMAXCa(0.0f), gMAXK_sl(0.0f), gLkg1(5.68e-5f), gLkg2(2.17e-5f), VNa(87.39f), VK(-84.69f), VLkg1(-58.0f), VLkg2(-65.0f), V0_xK_Ai(-46.7f),
		K_xK_Ai(-19.8f), V0_yK_Ai(-78.8f), K_yK_Ai(8.4f), V0_xK_sli(-30.0f), B_xK_sli(6.0f), F(96485.309f), A(1e-04f), d(0.2f), betaCa(1.5f),
		Ca0(1e-04f), R(8.3134f), cao(2.0f), Cm(1.0e-3f), temper(30.0f), Q10_20 ( pow(3,((temper-20.0f)/10.0f))), Q10_22 ( pow(3,((temper-22.0f)/10.0f))),
		Q10_30 ( pow(3,((temper-30.0f)/10.0f))), Q10_6_3 ( pow(3,((temper-6.3f)/10.0f))),	I_inj_abs(0.0F)/*I_inj_abs(9.0e-12f)*/,
		I_inj(-I_inj_abs*1000.0f/299.26058e-8f), eexc(0.0f), einh(-80.0f), texc(0.5f), tinh(10.0f), vthr(0.0f){
}

EgidioGranuleCell_TimeDriven_GPU::~EgidioGranuleCell_TimeDriven_GPU(void){
	DeleteClassGPU2();
}

void EgidioGranuleCell_TimeDriven_GPU::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}


VectorNeuronState * EgidioGranuleCell_TimeDriven_GPU::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * EgidioGranuleCell_TimeDriven_GPU::ProcessInputSpike(Interconnection * inter, double time){
	this->State_GPU->AuxStateCPU[inter->GetType()*State_GPU->GetSizeState() + inter->GetTargetNeuronModelIndex()] += 1e-9f*inter->GetWeight();

	return 0;
}



__global__ void EgidioGranuleCell_TimeDriven_GPU_UpdateState(EgidioGranuleCell_TimeDriven_GPU2 ** NeuronModel_GPU2, double CurrentTime){
	(*NeuronModel_GPU2)->UpdateState(CurrentTime);
}
		
bool EgidioGranuleCell_TimeDriven_GPU::UpdateState(int index, double CurrentTime){
	
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;

	if(prop.canMapHostMemory){
		EgidioGranuleCell_TimeDriven_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
	}else{
		HANDLE_ERROR(cudaMemcpy(state->AuxStateGPU,state->AuxStateCPU,this->N_TimeDependentNeuronState*state->SizeStates*sizeof(float),cudaMemcpyHostToDevice));
		EgidioGranuleCell_TimeDriven_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
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


enum NeuronModelOutputActivityType EgidioGranuleCell_TimeDriven_GPU::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType EgidioGranuleCell_TimeDriven_GPU::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & EgidioGranuleCell_TimeDriven_GPU::PrintInfo(ostream & out){
	return out;
}	


void EgidioGranuleCell_TimeDriven_GPU::InitializeStates(int N_neurons, int OpenMPQueueIndex){

	//Select the correnpondent device. 
	HANDLE_ERROR(cudaSetDevice(GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));  
	HANDLE_ERROR(cudaEventCreate(&stop));
	HANDLE_ERROR(cudaGetDeviceProperties( &prop, GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));

	this->State_GPU = (VectorNeuronState_GPU *) this->State;

	//Initial State
	float V=-80.0f;
	float xNa_f=0.00047309535f;
	float yNa_f=1.0f;
	float xNa_r=0.00013423511f;
	float yNa_r=0.96227829f;
	float xNa_p=0.00050020111f;
	float xK_V=0.010183001f;
	float xK_A=0.15685486f;
	float yK_A=0.53565367f;
	float xK_IR=0.37337035f;
	float xK_Ca=0.00012384122f;
	float xCa=0.0021951104f;
	float yCa=0.89509747f;
	float xK_sl=0.00024031171f;
	float Ca=Ca0;
	float gexc=0.0f;
	float ginh=0.0f;

	//Initialize neural state variables.
	float initialization[] = {V,xNa_f,yNa_f,xNa_r,yNa_r,xNa_p,xK_V,xK_A,yK_A,xK_IR,xK_Ca,xCa,yCa,xK_sl,Ca,gexc,ginh};
	State_GPU->InitializeStatesGPU(N_neurons, initialization, N_TimeDependentNeuronState, prop);

	//INITIALIZE CLASS IN GPU
	this->InitializeClassGPU2(N_neurons);

	InitializeVectorNeuronState_GPU2();
}


__global__ void EgidioGranuleCell_TimeDriven_GPU2_InitializeClassGPU2(EgidioGranuleCell_TimeDriven_GPU2 ** NeuronModel_GPU2,  
		float gMAXNa_f, float gMAXNa_r, float gMAXNa_p, float gMAXK_V,
		float gMAXK_A,float gMAXK_IR,float gMAXK_Ca,float gMAXCa,float gMAXK_sl, char const* integrationName, int N_neurons, void ** Buffer_GPU){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2) = new EgidioGranuleCell_TimeDriven_GPU2(gMAXNa_f, gMAXNa_r, gMAXNa_p, gMAXK_V,
			gMAXK_A,gMAXK_IR,gMAXK_Ca,gMAXCa,gMAXK_sl,integrationName, N_neurons, Buffer_GPU);
	}
}


void EgidioGranuleCell_TimeDriven_GPU::InitializeClassGPU2(int N_neurons){
	cudaMalloc(&NeuronModel_GPU2, sizeof(EgidioGranuleCell_TimeDriven_GPU2 **));
	
	char * integrationNameGPU;
	cudaMalloc((void **)&integrationNameGPU,32*4);
	HANDLE_ERROR(cudaMemcpy(integrationNameGPU,integrationMethod_GPU->GetType(),32*4,cudaMemcpyHostToDevice));
	
	this->N_thread = 128;
	this->N_block=prop.multiProcessorCount*4;
	if((N_neurons+N_thread-1)/N_thread < N_block){
		N_block = (N_neurons+N_thread-1)/N_thread;
	}
	int Total_N_thread=N_thread*N_block;

	integrationMethod_GPU->InitializeMemoryGPU(N_neurons, Total_N_thread);

	
	EgidioGranuleCell_TimeDriven_GPU2_InitializeClassGPU2<<<1,1>>>(NeuronModel_GPU2, gMAXNa_f, gMAXNa_r, gMAXNa_p, gMAXK_V,
			gMAXK_A,gMAXK_IR,gMAXK_Ca,gMAXCa,gMAXK_sl,integrationNameGPU, N_neurons, integrationMethod_GPU->Buffer_GPU);

	cudaFree(integrationNameGPU);
}


__global__ void initializeVectorNeuronState_GPU2(EgidioGranuleCell_TimeDriven_GPU2 ** NeuronModel_GPU2, int NumberOfVariables, float * InitialStateGPU, float * AuxStateGPU, float * StateGPU, double * LastUpdateGPU, double * LastSpikeTimeGPU, bool * InternalSpikeGPU, int SizeStates){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)->InitializeVectorNeuronState_GPU2(NumberOfVariables, InitialStateGPU, AuxStateGPU, StateGPU, LastUpdateGPU, LastSpikeTimeGPU, InternalSpikeGPU, SizeStates);
	}
}

void EgidioGranuleCell_TimeDriven_GPU::InitializeVectorNeuronState_GPU2(){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;
	initializeVectorNeuronState_GPU2<<<1,1>>>(NeuronModel_GPU2, state->NumberOfVariables, state->InitialStateGPU, state->AuxStateGPU, state->VectorNeuronStates_GPU, state->LastUpdateGPU, state->LastSpikeTimeGPU, state->InternalSpikeGPU, state->SizeStates);
}


__global__ void DeleteClass_GPU2(EgidioGranuleCell_TimeDriven_GPU2 ** NeuronModel_GPU2){
	if(blockIdx.x==0 && threadIdx.x==0){
		delete (*NeuronModel_GPU2); 
	}
}


void EgidioGranuleCell_TimeDriven_GPU::DeleteClassGPU2(){
    DeleteClass_GPU2<<<1,1>>>(NeuronModel_GPU2);
    cudaFree(NeuronModel_GPU2);
}


bool EgidioGranuleCell_TimeDriven_GPU::CheckSynapseType(Interconnection * connection){
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