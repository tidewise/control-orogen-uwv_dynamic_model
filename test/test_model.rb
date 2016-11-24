require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'
require "transformer/runtime"

describe 'uwv_dynamic_model::Task configuration' do
    include Orocos::Test::Component
    start 'task', 'uwv_dynamic_model::Task' => 'task'
    reader 'task', 'pose_samples', attr_name: 'pose_samples'
    reader 'task', 'secondary_states', attr_name: 'acce_samples'
    writer 'task', 'cmd_in', attr_name: 'cmd_in'

    def zero_command
        sample = task.cmd_in.new_sample
        sample.time = Time.now
        sample.linear = Types::Base::Vector3d.Zero
        sample.angular = Types::Base::Vector3d.Zero
        sample
    end

    def zero_pose
        pose = Types::Base::Samples.RigidBodyState.new
        pose.orientation = Eigen::Quaternion.Identity
        pose.position = pose.orientation * Eigen::Vector3.new(0,0,0)
        pose.velocity = Eigen::Vector3.new(0, 0,0)
        pose.angular_velocity = Eigen::Vector3.new(0, 0,0)
        pose
    end


    it 'add constant input in surge DOF' do
        task.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])
        parameters = task.model_parameters
        parameters.inertia_matrix.data[0] = 10
        task.model_parameters = parameters
        task.configure
        task.start

        step = 0.01
        sample = zero_command
        sample.linear[0] = 42
        mass_x = task.model_parameters.inertia_matrix.data[0]
        damp_lin_x = task.model_parameters.damping_matrices[0].data[0]
        damp_quad_x = task.model_parameters.damping_matrices[0].data[0]
        puts "mass_x: #{mass_x}"
        old_pose = zero_pose


        for i in 0..200
            sample.time = sample.time + step
            cmd_in.write sample

            pose_sample = assert_has_one_new_sample pose_samples, 1
            acce_sample = assert_has_one_new_sample acce_samples, 1
            expected_acce = (sample.linear[0] - (damp_lin_x + damp_quad_x*pose_sample.velocity[0].abs)*pose_sample.velocity[0])/mass_x
            derived_acce = (pose_sample.velocity[0] - old_pose.velocity[0]) / step
            old_pose = pose_sample

            assert_in_epsilon acce_sample.linear_acceleration.acceleration[0], expected_acce, 0.01
            assert_in_epsilon acce_sample.linear_acceleration.acceleration[0], derived_acce, 0.1
        end

        # assert_in_epsilon pose_sample.velocity[0], 1, 0.02
        # assert_in_epsilon pose_sample.velocity[1], 0, 0.02
        # assert_in_epsilon pose_sample.velocity[2], 0, 0.02
        # assert_in_epsilon pose_sample.angular_velocity[0], 0, 0.02
        # assert_in_epsilon pose_sample.angular_velocity[1], 0, 0.02
        # assert_in_epsilon pose_sample.angular_velocity[2], 0, 0.02
    end


end
