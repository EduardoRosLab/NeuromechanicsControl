/***************************************************************************
 *                           Lethargic_LIFTimeDrivenModel_1_1.h            *
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

#ifndef LETHARGIC_LIFTIMEDRIVENMODEL_1_1_H_
#define LETHARGIC_LIFTIMEDRIVENMODEL_1_1_H_

/*!
 * \file Lethargic_LIFTimeDrivenModel_1_1.h
 *
 * \author Francisco Naveros
 * \date December 2018
 *
 * This file declares a class which abstracts a Leaky Integrate-And-Fire neuron model with one 
 * differential equation and one time dependent equations (conductances).
 */

#include "./TimeDrivenNeuronModel.h"
#include "./VectorNeuronState.h"

#include <string>

using namespace std;

class InputSpike;
//class VectorNeuronState;
class Interconnection;



/*!
 * \class Lethargic_LIFTimeDrivenModel_1_1
 *
 * \brief Leaky Integrate-And-Fire Time-Driven neuron model with a membrane potential and
 * one excitatory conductances.
 *
 * This class abstracts the behavior of a neuron in a time-driven spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 *
 * \author Francisco Naveros
 * \date December 2018
 */
class Lethargic_LIFTimeDrivenModel_1_1 : public TimeDrivenNeuronModel {
	protected:

		/*!
		 * \brief Excitatory reversal potential in V units
		 */
		float eexc;

		/*!
		 * \brief Resting potential in V units
		 */
		float erest;

		/*!
		 * \brief Resting conductance in S units
		 */
		float grest;

		/*!
		 * \brief Inverse membrane capacitance units
		 */
		float inv_cm;

		/*!
		 * \brief Membrane capacitance in F units
		 */
		float cm;

		/*!
		 * \brief Firing threshold in V units
		 */
		float vthr;

		/*!
		 * \brief AMPA receptor time constant in s units
		 */
		float texc;
		float inv_texc;

		/*!
		 * \brief Refractory periodin s units
		 */
		float tref;



		bool * integration_required;



		/*!
		 * \brief It loads the neuron model description.
		 *
		 * It loads the neuron type description from the file .cfg.
		 *
		 * \param ConfigFile Name of the neuron description file (*.cfg).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		void LoadNeuronModel(string ConfigFile) throw (EDLUTFileException);

	public:

		/*!
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=2;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=1;

		/*!
		 * \brief Number of state variables which are calculate with a time dependent equation for each cell.
		*/
		static const int N_TimeDependentNeuronState=1;


		/*!
		 * \brief Default constructor with parameters.
		 *
		 * It generates a new neuron model object without being initialized.
		 *
		 * \param NeuronTypeID Neuron model identificator.
		 * \param NeuronModelID Neuron model configuration file.
		 */
		Lethargic_LIFTimeDrivenModel_1_1(string NeuronTypeID, string NeuronModelID);


		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		virtual ~Lethargic_LIFTimeDrivenModel_1_1();


		/*!
		 * \brief It loads the neuron model description and tables (if necessary).
		 *
		 * It loads the neuron model description and tables (if necessary).
 		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		virtual void LoadNeuronModel() throw (EDLUTFileException);


		/*!
		 * \brief It return the Neuron Model VectorNeuronState 
		 *
		 * It return the Neuron Model VectorNeuronState 
		 *
		 */
		virtual VectorNeuronState * InitializeState();


		/*!
		 * \brief It processes a propagated spike (input spike in the cell).
		 *
		 * It processes a propagated spike (input spike in the cell).
		 *
		 * \note This function doesn't generate the next propagated spike. It must be externally done.
		 *
		 * \param inter the interconection which propagate the spike
		 * \param time the time of the spike.
		 *
		 * \return A new internal spike if someone is predicted. 0 if none is predicted.
		 */
		virtual InternalSpike * ProcessInputSpike(Interconnection * inter, double time);


		/*!
		 * \brief Update the neuron state variables.
		 *
		 * It updates the neuron state variables.
		 *
		 * \param index The cell index inside the VectorNeuronState. if index=-1, updating all cell.
		 * \param CurrentTime Current time.
		 *
		 * \return True if an output spike have been fired. False in other case.
		 */
		virtual bool UpdateState(int index, double CurrentTime);


		/*!
		 * \brief It gets the neuron output activity type (spikes or currents).
		 *
		 * It gets the neuron output activity type (spikes or currents).
		 *
		 * \return The neuron output activity type (spikes or currents).
		 */
		enum NeuronModelOutputActivityType GetModelOutputActivityType();

		/*!
		 * \brief It gets the neuron input activity types (spikes and/or currents or none).
		 *
		 * It gets the neuron input activity types (spikes and/or currents or none).
		 *
		 * \return The neuron input activity types (spikes and/or currents or none).
		 */
		enum NeuronModelInputActivityType GetModelInputActivityType();


		/*!
		 * \brief It prints the time-driven model info.
		 *
		 * It prints the current time-driven model characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out);


		/*!
		 * \brief It initialice VectorNeuronState.
		 *
		 * It initialice VectorNeuronState.
		 *
		 * \param N_neurons cell number inside the VectorNeuronState.
		 * \param OpenMPQueueIndex openmp index
		 */
		virtual void InitializeStates(int N_neurons, int OpenMPQueueIndex);


		/*!
		 * \brief It evaluates if a neuron must spike.
		 *
		 * It evaluates if a neuron must spike.
		 *
		 * \param previous_V previous membrane potential
		 * \param NeuronState neuron state variables.
		 * \param index Neuron index inside the neuron model.
		 * \param elapsedTimeInNeuronModelScale integration method step.
		 * \return It returns if a neuron must spike.
		 */

		inline void EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
			if (NeuronState[0] < this->erest){
				NeuronState[0] = this->erest;
			}
			if (NeuronState[0] > this->vthr){
				NeuronState[0] = this->erest;
				State->NewFiredSpike(index);
				this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
				this->State->NInternalSpikeIndexs++;
			}
		}

		inline void SolveDifferentialEquation(int index, double CurrentTime){
			float * NeuronState = State->GetStateVariableAt(index);
			float elapsed_time = CurrentTime - State->GetLastUpdateTime(index);

			//since the conductance is zero, the membrane potential equation can be solved analitically.
			NeuronState[0] = this->erest - (this->erest - NeuronState[0])*expf(-elapsed_time*this->grest*this->inv_cm);
		}


		/*!
		 * \brief It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where differential equations are evaluated.
		 * \param AuxNeuronState results of the differential equations evaluation.
		 * \param index Neuron index inside the VectorNeuronState
		 */
		inline void EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
			if (this->GetVectorNeuronState()->GetLastSpikeTime(index)>this->tref){
				AuxNeuronState[0] = (NeuronState[1] * (this->eexc - NeuronState[0]) + grest * (this->erest - NeuronState[0]))*this->inv_cm;
			}
			else if ((this->GetVectorNeuronState()->GetLastSpikeTime(index) + elapsed_time)>this->tref){
				float fraction = (this->GetVectorNeuronState()->GetLastSpikeTime(index) + elapsed_time - this->tref) / elapsed_time;
				AuxNeuronState[0] = fraction*(NeuronState[1] * (this->eexc - NeuronState[0]) + grest * (this->erest - NeuronState[0]))*this->inv_cm;
			}
			else{
				AuxNeuronState[0] = 0;
			}
		}

		/*!
		 * \brief It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where time dependent equations are evaluated.
		 * \param elapsed_time integration time step.
		 * \param elapsed_time_index index inside the conductance_exp_values array.
		 */
		inline void EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
			float limit = 1e-9;
			float * Conductance_values = this->Get_conductance_exponential_values(elapsed_time_index);

			if (NeuronState[N_DifferentialNeuronState]<limit){
				NeuronState[N_DifferentialNeuronState] = 0.0f;
				this->integration_required[index] = false;
			}
			else{
				NeuronState[N_DifferentialNeuronState] *= Conductance_values[0];
			}
		}


		/*!
		 * \brief It Checks if the neuron model has this connection type.
		 *
		 * It Checks if the neuron model has this connection type.
		 *
		 * \param Type input connection type.
		 *
		 * \return If the neuron model supports this connection type
		 */
		virtual bool CheckSynapseType(Interconnection * connection);


		/*!
		 * \brief It calculates the conductace exponential value for an elapsed time.
		 *
		 * It calculates the conductace exponential value for an elapsed time.
		 *
		 * \param index elapsed time index .
		 * \param elapses_time elapsed time.
		 */
		void Calculate_conductance_exp_values(int index, float elapsed_time);

		/*!
		* \brief It calculates the number of electrical coupling synapses.
		*
		* It calculates the number for electrical coupling synapses.
		*
		* \param inter synapse that arrive to a neuron.
		*/
		void CalculateElectricalCouplingSynapseNumber(Interconnection * inter){};

		/*!
		* \brief It allocate memory for electrical coupling synapse dependencies.
		*
		* It allocate memory for electrical coupling synapse dependencies.
		*/
		void InitializeElectricalCouplingSynapseDependencies(){};

		/*!
		* \brief It calculates the dependencies for electrical coupling synapses.
		*
		* It calculates the dependencies for electrical coupling synapses.
		*
		* \param inter synapse that arrive to a neuron.
		*/
		void CalculateElectricalCouplingSynapseDependencies(Interconnection * inter){};


		/*!
		* \brief It loads the integration method from the neuron model configuration file
		*
		* It loads the integration method from the neuron model configuration file
		*
		* \param fileName neuron model configuration file name
		* \fh neuron model configuration file
		* \Currentline current line inside the file
		*/
		void loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException);

};

#endif /* LETHARGIC_LIFTIMEDRIVENMODEL_1_1_H_ */
