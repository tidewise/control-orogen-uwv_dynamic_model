/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityEstimator.hpp"

using namespace uwv_motion_model;

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



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityEstimator.hpp for more detailed
// documentation about them.

bool VelocityEstimator::configureHook()
{
    if (! VelocityEstimatorBase::configureHook())
        return false;
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

    // Updating linear velocity
    base::samples::RigidBodyState dvl_sample;
    if (_dvl_samples.read(dvl_sample) == RTT::NewData)
    {
        if(dvl_sample.hasValidVelocity())
            gMotionModel->setLinearVelocity(dvl_sample.velocity);
    }

    // Updating orientation
    base::samples::RigidBodyState imu_sample;
    if (_imu_orientation.read(imu_sample) == RTT::NewData)
    {
        if(dvl_sample.hasValidOrientation())
            gMotionModel->setOrientation(imu_sample.orientation);
    }

    // Angular velocity
    base::samples::IMUSensors fog_sample;
    if (_fog_samples.read(fog_sample) == RTT::NewData)
    {
        if(!fog_sample.gyro.hasNaN())
            gMotionModel->setAngularVelocity(fog_sample.gyro);
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
