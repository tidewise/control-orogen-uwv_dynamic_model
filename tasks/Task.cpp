/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace uwv_motion_model;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    gModelParameters = _model_parameters.get();

    int controlOrder 	= gModelParameters.ctrl_order;
    int simPerCycle 	= gModelParameters.sim_per_cycle;
    double samplingTime = TaskContext::getPeriod();

    // Creating the motion model object
    gMotionModel.reset(new uwv_dynamic_model::DynamicModel(controlOrder, samplingTime, simPerCycle));
    gMotionModel->initParameters(gModelParameters);

    // Updating the samplingTime at the component property
    gModelParameters.samplingtime = samplingTime;
    _model_parameters.set(gModelParameters);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::commands::Joints controlInput;
    base::samples::RigidBodyState states;
    SecondaryStates secondaryStates;
    ControlMode controlMode = _control_mode.get();
    static bool firstRun = true;

    // Updating control input
    if (_cmd_in.readNewest(controlInput) == RTT::NewData)
    {
    	// Checking if the control input was properly set
    	if(checkInput(controlInput))
    	{
    		if(state() != SIMULATING)
    			state(SIMULATING);

    		// Sending control input
    		switch(controlMode)
    		{
    		case PWM:
    			gMotionModel->sendPWMCommands(controlInput);
    			break;
    		case RPM:
    			gMotionModel->sendRPMCommands(controlInput);
    			break;
    		case EFFORT:
    			gMotionModel->sendEffortCommands(controlInput);
    			break;
    		}

    		// Getting updated states
    		gMotionModel->getPosition(states.position);
    		gMotionModel->getQuatOrienration(states.orientation);
    		gMotionModel->getLinearVelocity(states.velocity);
    		gMotionModel->getAngularVelocity(states.angular_velocity);
    		gMotionModel->getLinearAcceleration(secondaryStates.linearAcceleration.acceleration);
    		gMotionModel->getAngularAcceleration(secondaryStates.angularAcceleration.acceleration);
    		gMotionModel->getEfforts(secondaryStates.efforts.values);

    		// Transforming from euler to axis-angle representation
    		eulerToAxisAngle(states.angular_velocity);

    		// Setting the sample time
    		states.time 							 = controlInput.time;
    		secondaryStates.angularAcceleration.time = controlInput.time;
    		secondaryStates.linearAcceleration.time  = controlInput.time;
    		secondaryStates.efforts.time 			 = controlInput.time;

    		// Setting source and target frame names
    		states.sourceFrame 			= _source_frame.get();
    		states.targetFrame 			= _target_frame.get();

    		// Calculating the covariance matrix
    		setUncertainty(states);

    		// Writing the updated states
    		_cmd_out.write(states);
    		_secondary_states.write(secondaryStates);
    	}
    }
    else if(firstRun)
    {
    	firstRun = false;
    	states.initUnknown();
    	_cmd_out.write(states);
    }
}
bool Task::checkInput(base::samples::Joints &controlInput)
{
	ControlMode controlMode = _control_mode.get();
	bool inputError = false;

	// Checks if the controlInput size is correct and then if
	// the correspondent field is set
	switch(controlMode)
	{
	case PWM:
		if(controlInput.size() != gModelParameters.ctrl_order)
		{
			inputError = true;
			exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if (!controlInput.elements[i].hasRaw())
				{
					inputError = true;
					if(state() != RAW_FIELD_UNSET)
						error(RAW_FIELD_UNSET);
				}
			}
		}
		break;

	case RPM:
		if(controlInput.size() != gModelParameters.ctrl_order)
		{
			inputError = true;
			exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if (!controlInput.elements[i].hasSpeed())
				{
					inputError = true;
					if(state() != SPEED_FIELD_UNSET)
						error(SPEED_FIELD_UNSET);
				}
			}
		}
		break;

	case EFFORT:
		if(controlInput.size() != 6)
		{
			inputError = true;
			exception(WRONG_SIZE_OF_CONTROL_ELEMENTS);
		}
		else
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if (!controlInput.elements[i].hasEffort())
				{
					inputError = true;
					if(state() != EFFORT_FIELD_UNSET)
						error(EFFORT_FIELD_UNSET);
				}
			}
		}
		break;
	}

	// If everything is fine, inputError = FALSE and the method returns a TRUE value
	return !inputError;
}

void Task::eulerToAxisAngle(base::Vector3d &states)
{
    Eigen::AngleAxisd axisAngle = Eigen::AngleAxisd(Eigen::AngleAxisd(states(2), Eigen::Vector3d::UnitZ()) *
						    	  Eigen::AngleAxisd(states(1), Eigen::Vector3d::UnitY()) *
								  Eigen::AngleAxisd(states(0), Eigen::Vector3d::UnitX()));

    states = axisAngle.angle() * axisAngle.axis();
}

void Task::setUncertainty(base::samples::RigidBodyState &states)
{
	base::Vector6d velocityUncertainty = _velocity_uncertainty.get();
	static base::Vector6d positionUncertainty = Eigen::VectorXd::Zero(6);
	double samplingTime = TaskContext::getPeriod();


	for(int i = 0; i < 3; i++)
	{
		states.cov_velocity(i,i) = velocityUncertainty[i];
		states.cov_angular_velocity(i,i) = velocityUncertainty[i+3];

		// Euler integration of the velocity uncertainty in order to
		// calculate the position uncertainty
		positionUncertainty[i] += samplingTime*velocityUncertainty[i];
		positionUncertainty[i+3] += samplingTime*velocityUncertainty[i+3];

		states.cov_position(i,i) = positionUncertainty[i];
		states.cov_orientation(i,i) = positionUncertainty[i+3];
	}
}

bool Task::setNewParameters(void)
{
	uwv_dynamic_model::Parameters uwvParameters = _model_parameters.get();
	return gMotionModel->setUWVParameters(uwvParameters);
}

void Task::resetStates(void)
{
	gMotionModel->resetStates();
}

void Task::errorHook()
{
    TaskBase::errorHook();

    base::commands::Joints newControlInput;

    if (_cmd_in.readNewest(newControlInput) == RTT::NewData)
    {
    	// If there's no input error anymore, the system is recovered to operational state
    	if(checkInput(newControlInput))
    		recover();
    }
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
