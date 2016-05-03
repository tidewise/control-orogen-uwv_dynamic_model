/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityEstimator.hpp"

using namespace uwv_dynamic_model;

VelocityEstimator::VelocityEstimator(std::string const& name)
    : VelocityEstimatorBase(name)
{
}

VelocityEstimator::VelocityEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : VelocityEstimatorBase(name, engine)
{
}

VelocityEstimator::~VelocityEstimator()
{
}


DynamicSimulator* VelocityEstimator::setSimulator(void)
{
    simulator = new DynamicSimulator();
    return simulator;
}

void VelocityEstimator::handleControlInput(const base::LinearAngular6DCommand &control_input)
{
    queueOfEffort.push(control_input);
}
void VelocityEstimator::handlePoseState(const base::samples::RigidBodyState &state)
{
    queueOfstate.push(state);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityEstimator.hpp for more detailed
// documentation about them.

bool VelocityEstimator::configureHook()
{
    if (! VelocityEstimatorBase::configureHook())
        return false;

    // Creating the motion model object
    model_simulation2.reset(new ModelSimulation(setSimulator(), TaskContext::getPeriod(), _sim_per_cycle.get(), 0));
    model_simulation2->setUWVParameters(model_parameters);

    last_control_input2 = base::Time::fromSeconds(0);

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
//        if(dvl_sample.hasValidVelocity())
//            gMotionModel->setLinearVelocity(dvl_sample.velocity);
    }

    // Updating orientation
    // Direct from sensor. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::RigidBodyState imu_sample;
    if (_imu_orientation.read(imu_sample) == RTT::NewData)
    {
        if(dvl_sample.hasValidOrientation())
            model_simulation->setOrientation(imu_sample.orientation);
    }

    // Angular velocity
    // Direct from sensor. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::IMUSensors fog_sample;
    if (_fog_samples.read(fog_sample) == RTT::NewData)
    {
        if(!fog_sample.gyro.hasNaN())
        {   // TODO need validation
            base::samples::RigidBodyState temp_pose = toRBS(model_simulation->getPose());
            temp_pose.angular_velocity = fog_sample.gyro;
            model_simulation->setPose(fromRBS(temp_pose));
        }

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
