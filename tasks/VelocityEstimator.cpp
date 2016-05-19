/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VelocityEstimator.hpp"

using namespace uwv_dynamic_model;

VelocityEstimator::VelocityEstimator(std::string const& name, DynamicSimulator* simulator)
    : VelocityEstimatorBase(name), model_simulation2(NULL)
{
    setSimulator(simulator);
}

VelocityEstimator::VelocityEstimator(std::string const& name, RTT::ExecutionEngine* engine,DynamicSimulator* simulator)
    : VelocityEstimatorBase(name, engine), model_simulation2(NULL)
{
    setSimulator(simulator);
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

double VelocityEstimator::verticalVelocityEstimation(const base::samples::RigidBodyState &depth_sample)
{
    queueOfDepthData.push_back(depth_sample);
    double vertical_velocity = 0;
    if(queueOfDepthData.size() == filter_coeff.size() )
    {
        for (int i = 0; i < queueOfDepthData.size(); ++i)
            vertical_velocity += queueOfDepthData.at(i).position[2] * filter_coeff[i];
        // Moving Average Filter for step
        double step = (queueOfDepthData.back().time - queueOfDepthData.front().time).toSeconds() / (queueOfDepthData.size()-1) ;
        vertical_velocity /= step;
        queueOfDepthData.pop_front();
    }
    return vertical_velocity;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VelocityEstimator.hpp for more detailed
// documentation about them.

bool VelocityEstimator::configureHook()
{
    if (! VelocityEstimatorBase::configureHook())
        return false;

    // Creating the second motion model object
    delete model_simulation2;
    model_simulation2 = new ModelSimulation(simulator, TaskContext::getPeriod(), _sim_per_cycle.get(), 0);
    model_simulation2->setUWVParameters(_model_parameters.get());

    // Vertical velocity filter
    numeric::SavitzkyGolayFilter(filter_coeff, _least_square_point.get(), _half_width.get(), _polynomial_order.get(), 1);

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
        Eigen::Affine3d dvl2body;
        base::Vector3d dvl_velocity;
        base::Vector3d current_angular_velocity;

        if(!dvl_sample.hasValidVelocity())
            return;

        // Sets the pose transformation between the dvl and the robot frame
        if (!getTransformation(_dvl2body, dvl2body))
            return;
        dvl_velocity = dvl2body.rotation() * dvl_sample.velocity;

        // Corrects the dvl's measured velocities to subtract the effects of the
        // vehicle's angular velocity on the linear velocity measurement
        current_angular_velocity = toRBS(model_simulation->getPose()).angular_velocity;
        if(!base::isNaN(current_angular_velocity) && !current_angular_velocity.isZero())
        {
            base::Vector3d euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(current_angular_velocity.norm(), current_angular_velocity.normalized())));
            dvl_velocity -= Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x()).cross(dvl2body.translation());
        }

        // Updates dvl's measurement for the vehicle's frame
        dvl_sample.velocity = dvl_velocity;

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
    // From orientation estimator. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::RigidBodyState orientation_sample;
    if (_orientation_samples.read(orientation_sample) == RTT::NewData)
    {
        if(!orientation_sample.hasValidOrientation())
            return;
        model_simulation->setOrientation(orientation_sample.orientation);
    }

    // Angular velocity
    // Direct from imu. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::IMUSensors imu_sample;
    if (_imu_samples.read(imu_sample) == RTT::NewData)
    {
        // receive sensor to body transformation
        Eigen::Affine3d imu2body;

        if(imu_sample.gyro.hasNaN())
            return;

        if (!getTransformation(_imu2body, imu2body))
            return;

        base::samples::RigidBodyState temp_pose = toRBS(model_simulation->getPose());
        temp_pose.angular_velocity = eulerToAxisAngle(imu2body.rotation() * imu_sample.gyro);
        model_simulation->setPose(fromRBS(temp_pose));
    }

    // Vertical velocity
    // Derived from pressure sensor. Assuming high rate sampling time, timestamp is not take in account.
    base::samples::RigidBodyState depth_sample;
    if (_depth_samples.read(depth_sample) == RTT::NewData)
    {
        Eigen::Affine3d pressure_sensor2body;

        if(!depth_sample.hasValidPosition(2))
            return;

        // Sets the pose transformation between the pressure sensor and the robot frame
        if (!getTransformation(_pressure_sensor2body, pressure_sensor2body))
            return;

        depth_sample.orientation = base::Orientation::Identity();
        depth_sample.setTransform(depth_sample.getTransform() * pressure_sensor2body.inverse());

        base::samples::RigidBodyState temp_pose = toRBS(model_simulation->getPose());
        temp_pose.velocity[2] = verticalVelocityEstimation(depth_sample);
        temp_pose.position[2] = depth_sample.position[2];
        model_simulation->setPose(fromRBS(temp_pose));
    }
}

bool VelocityEstimator::getTransformation(const transformer::Transformation &transformer, Eigen::Affine3d &transformationMatrix)
{
    if (!transformer.get(base::Time::now(), transformationMatrix))
    {
        if(state() != MISSING_TRANSFORMATION)
            state(MISSING_TRANSFORMATION);
        return false;
    }
    else
    {
        if(state() != RUNNING)
            state(RUNNING);
    }

    return true;
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

    delete model_simulation2;
    while( !queueOfDepthData.empty())
        queueOfDepthData.pop_front();
    while( !queueOfStates.empty())
        queueOfStates.pop();
}
