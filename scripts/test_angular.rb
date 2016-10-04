require 'orocos'
require 'Qt'
require 'vizkit'
include Orocos

Orocos.initialize

Orocos.run 'auv_control::ConstantCommand'         => 'ConstantCommand',
           'uwv_dynamic_model::Task'               => 'MotionModel' do
#            :valgrind=>['AccelerationController'],
#            :output => '%m-%p.log' do


	## Get the specific task context ##
	constantCommand        = TaskContext.get 'ConstantCommand'
	motionModel            = TaskContext.get 'MotionModel'

  init = constantCommand.cmd
  init.linear[0] = 0
  init.linear[1] = 0
  init.linear[2] = 0
  init.angular[0] = 0
  init.angular[1] = 0
  init.angular[2] = 0
  constantCommand.cmd = init

  ##########################################################################
  #                       CONFIGURATION FILES
  ##########################################################################

  pose = Types::Base::Samples.RigidBodyState.new
  pose.orientation = Eigen::Quaternion.from_angle_axis( Math::PI*30/180, Eigen::Vector3.new( 1, 0, 0 ) )
  pose.position = pose.orientation * Eigen::Vector3.new(0,0,0)
  pose.velocity = Eigen::Vector3.new(0, 0,0)
  pose.angular_velocity = Eigen::Vector3.new(0, 0, 188.495)


  # motionModel.apply_conf_file("uwv_dynamic_model.yml",["spinning_disc"])
  motionModel.apply_conf_file("uwv_dynamic_model.yml",["spinning_disc"])
  ##########################################################################
  #		                    COMPONENT INPUT PORTS
  ##########################################################################

  constantCommand.cmd_out.connect_to motionModel.cmd_in

  # Configuring and starting the component
  constantCommand.configure
  constantCommand.start
  motionModel.configure
  motionModel.setStates(pose)
  motionModel.start

  motionmodelproxy = Orocos::Async.proxy("MotionModel")
  statesPort = motionmodelproxy.port("pose_samples")

  rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization

  # Connecting the vehicle model output to the 3D Visualization plugin
  statesPort.connect_to do |sample, |
    rbs_3DVisualization.updateRigidBodyState(sample)
  end


  Vizkit.exec

end
