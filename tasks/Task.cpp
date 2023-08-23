/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace uwv_dynamic_model;

Task::Task(std::string const& name, ModelSimulator simulator)
    : TaskBase(name), simulator(simulator), model_simulation(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, ModelSimulator simulator)
    : TaskBase(name, engine), simulator(simulator), model_simulation(NULL)
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

    // Creating the motion model object
    std::auto_ptr<ModelSimulation> sim(new ModelSimulation(simulator, TaskContext::getPeriod(), _sim_per_cycle.get(), 0));
    model_simulation = sim.release();

    model_simulation->setUWVParameters(_model_parameters.get());

    last_control_input = base::Time::fromSeconds(0);

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
    base::samples::RigidBodyState pose;

    if (_cmd_in.readNewest(control_input) != RTT::NewData)
        return;
    if(!checkInput(control_input))
        return;

    for(int i = 0; i < 3; i++)
    {
        if(base::isNaN(control_input.linear[i]))
            control_input.linear[i] = 0;
        if(base::isNaN(control_input.angular[i]))
            control_input.angular[i] = 0;
    }

    // Getting new samplingTime. Useful when there is no periodicity in input
    double sampling_time = (control_input.time - last_control_input).toSeconds();
    if (sampling_time > 0 && last_control_input != base::Time::fromSeconds(0))
        model_simulation->setSamplingTime(sampling_time);
    last_control_input = control_input.time;

    // Sending control input & getting updated states
    pose = toRBS(model_simulation->sendEffort(toVector6d(control_input)));

    pose.time = control_input.time;
    pose.sourceFrame = _source_frame.get();
    pose.targetFrame = _target_frame.get();

    // Calculating the covariance matrix
    setUncertainty(pose);

    // Do something with data in derived class
    handleStates(pose, control_input);
    setRuntimeState();

    // Writing the updated states
    _pose_samples.write(pose);
    _secondary_states.write(getSecondaryStates(control_input, model_simulation->getAcceleration()));

}

bool Task::checkInput(const base::LinearAngular6DCommand &control_input)
{
    if(control_input.time == base::Time::fromSeconds(0))
    {
        exception(INPUT_TIMESTAMP_NOT_SET);
        return false;
    }

    if((control_input.time - last_control_input).toSeconds() <= 0)
    {
        exception(COMMAND_WITH_REPEATED_TIMESTAMP);
        return false;
    }
    return true;
}

base::samples::RigidBodyState Task::toRBS(const PoseVelocityState &states)
{
    base::samples::RigidBodyState new_state = base::samples::RigidBodyState::invalid();
    new_state.position = states.position;
    new_state.orientation = states.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    if(_source_frame.get() == _target_frame.get())
        new_state.velocity = states.linear_velocity;
    else
        new_state.velocity = states.orientation.matrix()*states.linear_velocity;
    new_state.angular_velocity = states.angular_velocity;
    return new_state;
}

PoseVelocityState Task::fromRBS(const base::samples::RigidBodyState &states)
{
    PoseVelocityState new_state;
    new_state.position = states.position;
    new_state.orientation = states.orientation;
    // RBS velocity expressed in target frame, PoseVelocityState velocity expressed in body-frame
    if(states.sourceFrame == states.targetFrame)
        new_state.linear_velocity = states.velocity;
    else
        new_state.linear_velocity = states.orientation.inverse()*states.velocity;
    new_state.angular_velocity = states.angular_velocity;
    return new_state;
}

base::Vector6d Task::toVector6d(const base::LinearAngular6DCommand &control_input)
{
    base::Vector6d control;
    control.head(3) = control_input.linear;
    control.tail(3) = control_input.angular;
    return control;
}

SecondaryStates Task::getSecondaryStates(const base::LinearAngular6DCommand &control_input, const AccelerationState &acceleration)
{
    SecondaryStates secondary_states;
    secondary_states.linear_acceleration.acceleration = acceleration.linear_acceleration;
    secondary_states.angular_acceleration.acceleration = acceleration.angular_acceleration;
    secondary_states.angular_acceleration.time = control_input.time;
    secondary_states.linear_acceleration.time  = control_input.time;
    secondary_states.time = control_input.time;
    secondary_states.efforts = control_input;
    return secondary_states;
}

void Task::setUncertainty(base::samples::RigidBodyState &states)
{
    base::Vector6d velocityUncertainty = _velocity_uncertainty.get();
    static base::Vector6d positionUncertainty = Eigen::VectorXd::Zero(6);
    double samplingTime = TaskContext::getPeriod();
    states.cov_velocity = Eigen::Matrix3d::Zero();

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

void Task::setStates(::base::samples::RigidBodyState const & pose_state)
{
    model_simulation->setPose(fromRBS(pose_state));
}

void Task::setSimulator(ModelSimulator simulator)
{
   this->simulator = simulator;
}

void Task::handleStates(const base::samples::RigidBodyState &state, const base::LinearAngular6DCommand &control_input)
{}

void Task::setRuntimeState(void)
{
    if(state() != SIMULATING)
        state(SIMULATING);
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    model_simulation->resetStates();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete model_simulation;
    model_simulation = NULL;
}
