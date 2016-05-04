/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityEstimator.hpp"

using namespace uwv_dynamic_model;

VelocityEstimator::VelocityEstimator(std::string const& name)
    : VelocityEstimatorBase(name)
{
    setSimulator(new DynamicSimulator());
}

VelocityEstimator::VelocityEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : VelocityEstimatorBase(name, engine)
{
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::handleStates(const base::samples::RigidBodyState &state, const base::LinearAngular6DCommand &control_input)
{
    queueOfStates.push(std::make_pair(control_input, state));
}

base::samples::RigidBodyState VelocityEstimator::updatePoseWithXYVelocity(const base::samples::RigidBodyState &pose, const base::samples::RigidBodyState &dvl_sample)
{
    // Update X-Y velocities
    base::samples::RigidBodyState temp_pose = pose;
    temp_pose.velocity[0] = dvl_sample.velocity[0];
    temp_pose.velocity[1] = dvl_sample.velocity[1];
    return temp_pose;
}

base::samples::RigidBodyState VelocityEstimator::replayModel(const base::samples::RigidBodyState &dvl_sample)
{
    if(queueOfStates.empty())
        std::runtime_error("VelocityEstimator replayModel: Empty queue.");

    // Queue of replayed model, updated states
    std::queue<std::pair<base::LinearAngular6DCommand, base::samples::RigidBodyState> > tempQueueOfStates;
    base::Time last_control_input2 = base::Time::fromSeconds(0);

    base::samples::RigidBodyState temp_pose = dvl_sample;

    // Considering that only the XY velocities need to be updated in all the enqueued pose states.
    // After replaying the model and updating pose states, queueOfStates is replaced with updated states.
    while(!queueOfStates.empty())
    {
        temp_pose = updatePoseWithXYVelocity(queueOfStates.front().second, temp_pose);
        // Queue with updated states
        tempQueueOfStates.push(std::make_pair(queueOfStates.front().first, temp_pose));

        // Getting new samplingTime. Useful when there is no periodicity in input
        double sampling_time = (queueOfStates.front().first.time - last_control_input2).toSeconds();
        if (sampling_time > 0 && last_control_input2 != base::Time::fromSeconds(0))
            model_simulation2->setSamplingTime(sampling_time);
        last_control_input2 = queueOfStates.front().first.time;

        temp_pose = toRBS(model_simulation2->sendEffort(toVector6d(queueOfStates.front().first), fromRBS(temp_pose)));

        queueOfStates.pop();
    }
    // Update queue
    queueOfStates = tempQueueOfStates;
    return queueOfStates.front().second;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityEstimator.hpp for more detailed
// documentation about them.

bool VelocityEstimator::configureHook()
{
    if (! VelocityEstimatorBase::configureHook())
        return false;

    // Creating the second motion model object
    model_simulation2.reset(new ModelSimulation(simulator, TaskContext::getPeriod(), _sim_per_cycle.get(), 0));
    model_simulation2->setUWVParameters(_model_parameters.get());

    return true;
}
bool VelocityEstimator::startHook()
{
    if (! VelocityEstimatorBase::startHook())
        return false;
    return true;
}
void VelocityEstimator::updateHook()
{
    VelocityEstimatorBase::updateHook();

    // Updating linear velocity.
    // Consider timestamp of sample. Replay model with effort in queue from dvl.time with linear velocity equal  till last sample.
    base::samples::RigidBodyState dvl_sample;
    if (_dvl_samples.read(dvl_sample) == RTT::NewData)
    {
        if(!dvl_sample.hasValidVelocity())
            return;
        // Pop out old data
        while(!queueOfStates.empty() && dvl_sample.time > queueOfStates.front().first.time)
            queueOfStates.pop();
        // In case of empty queue, dvl_sample ahead of model_simulation
        if(!queueOfStates.empty())
            dvl_sample = replayModel(dvl_sample);

        base::samples::RigidBodyState temp_pose = updatePoseWithXYVelocity(toRBS(model_simulation->getPose()), dvl_sample);
        model_simulation->setPose(fromRBS(temp_pose));
    }

    // Updating orientation
    // Direct from sensor. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::RigidBodyState imu_sample;
    if (_imu_orientation.read(imu_sample) == RTT::NewData)
    {
        if(!imu_sample.hasValidOrientation())
            return;
        model_simulation->setOrientation(imu_sample.orientation);
    }

    // Angular velocity
    // Direct from sensor. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::IMUSensors fog_sample;
    if (_fog_samples.read(fog_sample) == RTT::NewData)
    {
        if(fog_sample.gyro.hasNaN())
            return;
        // TODO need validation
        base::samples::RigidBodyState temp_pose = toRBS(model_simulation->getPose());
        temp_pose.angular_velocity = fog_sample.gyro;
        model_simulation->setPose(fromRBS(temp_pose));
    }
}
void VelocityEstimator::errorHook()
{
    VelocityEstimatorBase::errorHook();
}
void VelocityEstimator::stopHook()
{
    VelocityEstimatorBase::stopHook();
}
void VelocityEstimator::cleanupHook()
{
    VelocityEstimatorBase::cleanupHook();
}
