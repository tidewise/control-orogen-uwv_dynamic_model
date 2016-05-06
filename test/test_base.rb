require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'

describe 'uwv_dynamic_model::VelocityEstimator configuration' do
  include Orocos::Test::Component
  start 'velocity_estimator', 'uwv_dynamic_model::VelocityEstimator' => 'velocity_estimator'
  reader 'velocity_estimator', 'pose_samples', :attr_name => 'pose_samples'
  writer 'velocity_estimator', 'cmd_in', :attr_name => 'cmd_in'
  writer 'velocity_estimator', 'dvl_samples', :attr_name => 'dvl_samples'
  writer 'velocity_estimator', 'imu_orientation', :attr_name => 'imu_orientation'
  writer 'velocity_estimator', 'fog_samples', :attr_name => 'fog_samples'


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
      sample.velocity[2] = y
      sample
  end

  it 'add input in model' do

    velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['default'])

    velocity_estimator.configure
    velocity_estimator.start

    sample = zero_command

    cmd_in.write sample

    data = assert_has_one_new_sample pose_samples, 1

    assert (data.velocity[0] + 0).abs < 0.001
    assert (data.velocity[1] + 0).abs < 0.001
    assert (data.velocity[2] + 0).abs < 0.001
    assert (data.angular_velocity[0] + 0).abs < 0.001
    assert (data.angular_velocity[1] + 0).abs < 0.001
    assert (data.angular_velocity[2] + 0).abs < 0.001
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

  assert (data.velocity[0] - 1).abs < 0.001
  assert (data.velocity[1] + 0).abs < 0.001
  assert (data.velocity[2] + 0).abs < 0.001
  assert (data.angular_velocity[0] + 0).abs < 0.001
  assert (data.angular_velocity[1] + 0).abs < 0.001
  assert (data.angular_velocity[2] + 0).abs < 0.001
end

it 'input with repeated timestamp' do

  velocity_estimator.apply_conf_file("uwv_dynamic_model.yml",['simple_case'])

  velocity_estimator.configure
  velocity_estimator.start

  for i in 0..100
    sample = zero_command
    sample.time = sample.time + 0.01*i
    if i == 90
      repeated_time = sample.time
    end
    if i == 91
      sample.time = repeated_time
    end
    sample.linear[0] = 2
    cmd_in.write sample

    begin
        data = assert_has_one_new_sample pose_samples, 1
    rescue Exception
        puts velocity_estimator.state
        # How does assert_state_change for exception state works??
        #assert_state_change(velocity_estimator) { |s| s == :COMMAND_WITH_REPEATED_TIMESTAMP }
      raise
    end

  end
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

  assert (data.velocity[0] - 1).abs < 0.1
  assert (data.velocity[1] + 0).abs < 0.001
  assert (data.velocity[2] + 0).abs < 0.001
  assert (data.angular_velocity[0] + 0).abs < 0.001
  assert (data.angular_velocity[1] + 0).abs < 0.001
  assert (data.angular_velocity[2] + 0).abs < 0.001
end

end