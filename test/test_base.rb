require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'
require "transformer/runtime"

describe 'uwv_dynamic_model::VelocityEstimator configuration' do
    include Orocos::Test::Component
    start 'velocity_estimator', 'uwv_dynamic_model::VelocityEstimator' => 'velocity_estimator'
    reader 'velocity_estimator', 'pose_samples', attr_name: 'pose_samples'
    writer 'velocity_estimator', 'cmd_in', attr_name: 'cmd_in'
    writer 'velocity_estimator', 'dvl_samples', attr_name: 'dvl_samples'
    writer 'velocity_estimator', 'depth_samples', attr_name: 'depth_samples'
    writer 'velocity_estimator', 'orientation_samples', attr_name: 'imu_orientation'
    writer 'velocity_estimator', 'imu_samples', attr_name: 'fog_samples'

    def zero_command
        sample = velocity_estimator.cmd_in.new_sample
        sample.time = Time.now
        sample.linear = Types::Base::Vector3d.Zero
        sample.angular = Types::Base::Vector3d.Zero
        sample
    end

    def dvl_data(x, y)
        sample = velocity_estimator.dvl_samples.new_sample
        sample.time = Time.now
        sample.velocity[0] = x
        sample.velocity[1] = y
        sample.velocity[2] = 0
        sample
    end

    def depth_data(z)
        sample = velocity_estimator.depth_samples.new_sample
        sample.time = Time.now
        sample.position[0] = 0
        sample.position[1] = 0
        sample.position[2] = z
        sample
    end

    it 'add input in model' do
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['default'])

        velocity_estimator.configure
        velocity_estimator.start

        sample = zero_command
        cmd_in.write sample

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[0], 0, 0.02
        assert_in_epsilon data.velocity[1], 0, 0.02
        assert_in_epsilon data.velocity[2], 0, 0.02
        assert_in_epsilon data.angular_velocity[0], 0, 0.02
        assert_in_epsilon data.angular_velocity[1], 0, 0.02
        assert_in_epsilon data.angular_velocity[2], 0, 0.02
    end

    it 'add constant input in model' do
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])

        velocity_estimator.configure
        velocity_estimator.start

        for i in 0..200
            sample = zero_command
            sample.time = sample.time + 0.01*i
            sample.linear[0] = 2
            cmd_in.write sample
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[0], 1, 0.02
        assert_in_epsilon data.velocity[1], 0, 0.02
        assert_in_epsilon data.velocity[2], 0, 0.02
        assert_in_epsilon data.angular_velocity[0], 0, 0.02
        assert_in_epsilon data.angular_velocity[1], 0, 0.02
        assert_in_epsilon data.angular_velocity[2], 0, 0.02
    end

    it 'input with repeated timestamp' do
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])

        velocity_estimator.configure
        velocity_estimator.start

        sample = zero_command
        cmd_in.write sample
        data = assert_has_one_new_sample pose_samples, 1
        cmd_in.write sample
        assert_state_change(velocity_estimator, timeout = 1) { |state| state == :COMMAND_WITH_REPEATED_TIMESTAMP }
    end

    it 'add dvl data in velocity estimator' do
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])
        RANDOM_GENERATOR = Random.new()

        velocity_estimator.configure
        velocity_estimator.start

        for i in 0..500
            sample = zero_command
            sample.time = sample.time + 0.01*i
            sample.linear[0] = 2

            if i % 12 == 0
                number = i/(100*(i+10)).to_f
                vx = i/(i+10).to_f + RANDOM_GENERATOR.rand(-number..number)
                dvl = dvl_data(vx, 0)
                dvl.time = sample.time + RANDOM_GENERATOR.rand(-0.01..0.01) - 2
                dvl_samples.write dvl
            end
            cmd_in.write sample
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[0], 1, 0.02
        assert_in_epsilon data.velocity[1], 0, 0.02
        assert_in_epsilon data.velocity[2], 0, 0.02
        assert_in_epsilon data.angular_velocity[0], 0, 0.02
        assert_in_epsilon data.angular_velocity[1], 0, 0.02
        assert_in_epsilon data.angular_velocity[2], 0, 0.02
    end

    it 'add depth data in velocity estimator. constant vertical velocity' do
        Orocos.transformer.load_conf("static_transforms.rb")
        Orocos.transformer.setup(velocity_estimator)
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case_no_vertical_damping'])
        velocity_estimator.configure
        velocity_estimator.start

        z_position = -10
        z_velocity = 0.1
        step = 0.01

        sample = zero_command
        depth =  depth_data(z_position)

        for i in 0..100
            sample.time = sample.time + step
            depth.time = sample.time

            z_position = z_position + z_velocity * step
            depth.position[2] = z_position
            sleep(0.01)

            cmd_in.write sample
            depth_samples.write depth
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[2],  z_velocity, 0.02
    end

    it 'add depth data in velocity estimator. constant vertical velocity with influency of vertical damping' do
        Orocos.transformer.load_conf("static_transforms.rb")
        Orocos.transformer.setup(velocity_estimator)
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])

        velocity_estimator.configure
        velocity_estimator.start

        z_position = -10
        z_velocity = 0.1
        step = 0.01

        sample = zero_command
        depth =  depth_data(z_position)

        for i in 0..100
            sample.time = sample.time + step
            depth.time = sample.time

            z_position = z_position + z_velocity * step
            depth.position[2] = z_position
            sleep(0.01)

            cmd_in.write sample
            depth_samples.write depth
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[2], z_velocity, 0.02
    end

    it 'add depth data in velocity estimator. varing vertical velocity' do
        Orocos.transformer.load_conf("static_transforms.rb")
        Orocos.transformer.setup(velocity_estimator)
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case_no_vertical_damping'])

        velocity_estimator.configure
        velocity_estimator.start

        z_position = -10
        z_vel_amplitude = 0.1
        z_vel_frequency = 0.1
        z_velocity = 0
        step = 0.01

        sample = zero_command
        depth =  depth_data(z_position)

        for i in 0..100
            sample.time = sample.time + step
            depth.time = sample.time

            depth.position[2] = z_position + z_vel_amplitude*Math.sin(z_vel_frequency * sample.time.to_f)
            z_velocity = z_vel_amplitude * z_vel_frequency * Math.cos(z_vel_frequency * sample.time.to_f)
            sleep(0.01)

            cmd_in.write sample
            depth_samples.write depth
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[2], z_velocity, 0.02
    end

    it 'add depth data in velocity estimator. filtering high frequency noise' do
        Orocos.transformer.load_conf("static_transforms.rb")
        Orocos.transformer.setup(velocity_estimator)
        velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case_no_vertical_damping'])

        velocity_estimator.configure
        velocity_estimator.start

        z_position = -10
        z_vel_amplitude = 0.1
        z_vel_frequency = 0.1
        z_velocity = 0
        step = 0.01

        sample = zero_command
        depth =  depth_data(z_position)

        for i in 0..100
            sample.time = sample.time + step
            depth.time = sample.time

            depth.position[2] =  z_position + z_vel_amplitude * Math.sin(z_vel_frequency * sample.time.to_f)
                + z_vel_amplitude/100 * Math.sin(z_vel_frequency*100 * sample.time.to_f)

            # Analytically derive component with the smaller frequencies
            z_velocity = z_vel_amplitude * z_vel_frequency * Math.cos(z_vel_frequency * sample.time.to_f)
            sleep(0.01)

            cmd_in.write sample
            depth_samples.write depth
        end

        data = assert_has_one_new_sample pose_samples, 1
        assert_in_epsilon data.velocity[2], z_velocity, 0.02
    end

end