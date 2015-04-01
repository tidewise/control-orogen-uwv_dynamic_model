/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ForceApplier.hpp"

using namespace uwv_motion_model;

ForceApplier::ForceApplier(std::string const& name)
    : ForceApplierBase(name)
{
}

ForceApplier::ForceApplier(std::string const& name, RTT::ExecutionEngine* engine)
    : ForceApplierBase(name, engine)
{
}

ForceApplier::~ForceApplier()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ForceApplier.hpp for more detailed
// documentation about them.

bool ForceApplier::configureHook()
{
    if (! ForceApplierBase::configureHook())
        return false;

    TCM		= _TCM.get();

    int numberOfThrusters	= TCM.cols();

    treatOutput			= _treatOutput.get();

	thrusterNames	= _names.get();
	if(thrusterNames.size() != numberOfThrusters && thrusterNames.size() != 0){
		exception(WRONG_SIZE_OF_NAMES);
		return false;
	}

	CoeffThruster	= _thruster_coefficients.get() ;
	if(	CoeffThruster.size() != numberOfThrusters){
		exception(WRONG_SIZE_OF_THRUSTER_COEFFICIENTS);
		return false;
   	}

	controlModes	= _control_modes.get();
	if(controlModes.size() == 0){
		//resize the controlModes to number of Thrusters
		controlModes.resize(numberOfThrusters);
		//set undefined controlModes to RAW
		for(int i = 0; i < numberOfThrusters; i++){
			controlModes[i] = base::JointState::RAW;
		}
   	}
	else if(controlModes.size() != numberOfThrusters){
		exception(WRONG_SIZE_OF_CONTROLMODES);
		return false;
   	}


   // The three first elements are the forces and the last three are the torques applied in the AUV in body-frame
   forcesOut.resize(6);
   std::vector<std::string> forces_names;
   forces_names.push_back("Surge");
   forces_names.push_back("Sway");
   forces_names.push_back("Heave");
   forces_names.push_back("Roll");
   forces_names.push_back("Pitch");
   forces_names.push_back("Yaw");
   forcesOut.names = forces_names;

   newForceSample = false;

   return true;
}
bool ForceApplier::startHook()
{
    if (! ForceApplierBase::startHook())
        return false;
    return true;
}
void ForceApplier::updateHook()
{
    ForceApplierBase::updateHook();

    base::VectorXd	forces;

   if (_thruster_samples.read(thrusterForces) == RTT::NewData)
   {
	// Check if the inputs are OK
	if(checkControlInput(thrusterForces))
	{
		base::VectorXd forces;

		// Calculate the force applied by each thruster
		calcThrustersForces(thrusterForces, forces);
		// Apply the TCM and get the forces and torques applied in the AUV in body-frame
		calcOutput(forcesOut, forces);

		forcesOut.time = thrusterForces.time;
		// If the task is used as a subclasses, 'newForceSample' to be used in the main task to inform a new force sample is available.
		//	Set it as false in main task after using the data.
		newForceSample = true;

		// In case any treatment of the data need to be done in the main task before write as output.
		if(!treatOutput)
		{
			_forces.write(forcesOut);
			_thruster_forces.write(thrusterForces);
		}
	}
   }


}
void ForceApplier::errorHook()
{
    ForceApplierBase::errorHook();
}
void ForceApplier::stopHook()
{
    ForceApplierBase::stopHook();
}
void ForceApplier::cleanupHook()
{
    ForceApplierBase::cleanupHook();
}

// Calculate the force applied by each thruster, based on the thruster's coefficient.
void ForceApplier::calcThrustersForces(base::samples::Joints &forcesThruster, base::VectorXd &forces)
{

	int numberOfThrusters = forcesThruster.size();
	forces.resize(numberOfThrusters);

	for (int i=0; i<numberOfThrusters; i++)
	{
		double input;

		input = forcesThruster.elements[i].getField(controlModes[i]);

		// For the case where the control signal is a PWM signal, present in raw
		if(controlModes[i] == base::JointState::RAW)
			input *= _thrusterVoltage.get();

		// To avoid minimal change of values
		double factor = 0.01;
		if( input >= -factor && input <= factor && controlModes[i] != base::JointState::EFFORT)
			forcesThruster.elements[i].effort = 0.0;

		// In case the thruster's driver is able the estimate the forces of each thruster directly and send it in the effort field,
		//	there is no need to estimate the forces using the coefficients.
		else if(controlModes[i] != base::JointState::EFFORT)
		{
			// A common way to estimate the force applied by a thruster according the main literature. F = Coeff*V|V| or F = Coeff*speed|speed|
			if(input > 0)
				forcesThruster.elements[i].effort = CoeffThruster[i].positive * input*fabs(input);
			else
				forcesThruster.elements[i].effort = CoeffThruster[i].negative * input*fabs(input);
		}
		// The vector of forces for each thruster
		forces[i] = forcesThruster.elements[i].effort;
	}
}

// Calculate the forces and torques applied in the AUV, based in the TCM.
void ForceApplier::calcOutput(base::samples::Joints &cmd_out, base::VectorXd &forces)
{
	base::Vector6d outCmd = TCM * forces;
	for(int i; i<6; i++)
	{
		cmd_out.elements[i].effort = outCmd[i];
	}
}

// Verify the expected input signal
bool ForceApplier::checkControlInput(base::samples::Joints &forcesThruster)
{
	std::string textElement;
	bool checkError	= false;
	bool checkMode	= false;

	int numberOfThrusters = TCM.cols();

	if(forcesThruster.elements.size() != numberOfThrusters)
	{
		RTT::log(RTT::Error) << "The input has not "<< numberOfThrusters <<
				" thruster as predicted "<< RTT::endlog();
		exception(UNEXPECTED_THRUSTER_INPUT);
		return false;
	}

	for (int i=0; i<numberOfThrusters; i++)
	{	// Verify if the input mode is a valid input or a nan
		 switch(controlModes[i])
		{
		 	 case base::JointState::EFFORT:
		 		 if (!forcesThruster.elements[i].hasEffort())
		 		 {
					textElement = "effort";
					checkError = true;
		 		 }
		 		 break;
			 case base::JointState::SPEED:
				 if (!forcesThruster.elements[i].hasSpeed())
				 {
					textElement = "speed";
					checkError = true;
				 }
				 break;
			 case base::JointState::RAW:
				 if (!forcesThruster.elements[i].hasRaw())
				 {
					textElement = "speed";
					checkError = true;
				 }
				 break;
			 case base::JointState::POSITION:
				 checkMode = true;
				 if (!forcesThruster.elements[i].hasPosition())
				 {
					textElement = "position";
					checkError = true;
				 }
				 break;
			 case base::JointState::ACCELERATION:
				 checkMode = true;
				 if (!forcesThruster.elements[i].hasAcceleration())
				 {
					textElement = "acceleration";
					checkError = true;
				 }
				 break;
		}

		// In case of any input value is NAN or not the write control mode
		if(checkError || checkMode)
		{
			// Define how to call the problematic thruster
			std::string textThruster;
			if(thrusterNames.size() == numberOfThrusters)
				textThruster = thrusterNames[i];
			else
			{
				std::stringstream number;
				number << i;
				textThruster = number.str();
			}
			// In case of any input value is NAN
			if(checkError)
				RTT::log(RTT::Error) << "The field "<< textElement << " of thruster "<<
									textThruster << " was not set." << RTT::endlog();
			// In case the control mode is unfamiliar
			if(checkMode)
				RTT::log(RTT::Error) << "The field "<< textElement << " of thruster "<<
					textThruster << " is not a usual control input." << RTT::endlog();
			exception(UNSET_THRUSTER_INPUT);
			return false;
		}
	}
	// In case of all inputs have valid values
	return true;
}
