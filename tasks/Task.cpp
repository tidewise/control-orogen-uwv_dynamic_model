/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace uwv_dynamic_model;

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

    model_parameters = _model_parameters.get();

    // Creating the motion model object
    model_simulation.reset(new ModelSimulation(setSimulator(), TaskContext::getPeriod(), _sim_per_cycle.get(), 0));
    model_simulation->setUWVParameters(model_parameters);

    last_control_input = base::Time::fromSeconds(0);
    states.initUnknown();

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

    base::LinearAngular6DCommand control_input;

    // Updating control input
    if (_cmd_in.readNewest(control_input) == RTT::NewData)
    {
        // Checking if the control input was properly set
        if(checkInput(control_input))
        {
            // Setting the control input timestamp if it's not set
            if(control_input.time == base::Time::fromSeconds(0))
            {
                control_input.time = base::Time::now();
                if(state() != INPUT_TIMESTAMP_NOT_SET)
                    state(INPUT_TIMESTAMP_NOT_SET);
            }
            else if(state() != SIMULATING)
                state(SIMULATING);
            // Ignore the new sample if it has the same time stamp as the previous one

            double sampling_time = (control_input.time - last_control_input).toSeconds();
            // Getting new samplingTime
            if(last_control_input == base::Time::fromSeconds(0))
                last_control_input = control_input.time;
            else if ((control_input.time - last_control_input).toSeconds() > 0)
            {
                // Updating the samplingTime at the component property
                model_simulation->setSamplingTime(sampling_time);
                last_control_input = control_input.time;
            }

            // Sending control input
            // Getting updated states
            base::Vector6d control;
            control.head(3) = control_input.linear;
            control.tail(3) = control_input.angular;
            states = toRBS(model_simulation->sendEffort(control));

            // Getting updated states
            AccelerationState acceleration = model_simulation->getAcceleration();
            secondary_states.linear_acceleration.acceleration = acceleration.linear_acceleration;
            secondary_states.angular_acceleration.acceleration = acceleration.angular_acceleration;
            secondary_states.efforts = control_input;

            // Setting the sample time
            states.time = control_input.time;
            secondary_states.angular_acceleration.time = control_input.time;
            secondary_states.linear_acceleration.time  = control_input.time;
            secondary_states.efforts  = control_input;

            // Setting source and target frame names
            states.sourceFrame = _source_frame.get();
            states.targetFrame = _target_frame.get();

            // Calculating the covariance matrix
            setUncertainty(states);

            // Do something with data in derived class
            handlePoseState(states);
            handleControlInput(control_input);

            // Writing the updated states
            _pose_samples.write(states);
            _secondary_states.write(secondary_states);
        }
        else
            return;
    }
}

bool Task::checkInput(const base::LinearAngular6DCommand &control_input)
{
    if(control_input.linear.hasNaN() || control_input.angular.hasNaN())
    {
        exception(EFFORT_UNSET);
        return false;
    }

    if((control_input.time - last_control_input).toSeconds() == 0)
    {
        if(state() != COMMAND_WITH_REPEATED_TIMESTAMP)
            state(COMMAND_WITH_REPEATED_TIMESTAMP);
        return false;
    }
    return true;
}

base::samples::RigidBodyState Task::toRBS(const PoseVelocityState &states)
{
    base::samples::RigidBodyState new_state;
    new_state.position = states.position;
    new_state.orientation = states.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    new_state.velocity = states.orientation.matrix()*states.linear_velocity;
    // Transforming from euler to axis-angle representation
    new_state.angular_velocity = eulerToAxisAngle(states.angular_velocity);
    return new_state;
}

PoseVelocityState Task::fromRBS(const base::samples::RigidBodyState &states)
{
    PoseVelocityState new_state;
    new_state.position = states.position;
    new_state.orientation = states.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    new_state.linear_velocity = states.orientation.inverse()*states.velocity;
    // Transforming axis-angle representation to body frame angular velocity.
    if(!states.angular_velocity.isZero())
        new_state.angular_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(states.angular_velocity.norm(), states.angular_velocity.normalized())));
    return new_state;
}

base::Vector3d Task::eulerToAxisAngle(const base::Vector3d &states)
{
    Eigen::AngleAxisd axisAngle = Eigen::AngleAxisd(Eigen::AngleAxisd(states(2), Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(states(1), Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(states(0), Eigen::Vector3d::UnitX()));
    return axisAngle.angle() * axisAngle.axis();
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

void Task::resetStates(void)
{
    model_simulation->resetStates();
}

DynamicSimulator* Task::setSimulator(void)
{
    simulator = new DynamicKinematicSimulator();
    return simulator;
}

void Task::handleControlInput(const base::LinearAngular6DCommand &controlInput)
{}
void Task::handlePoseState(const base::samples::RigidBodyState &state)
{}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete simulator;
}
