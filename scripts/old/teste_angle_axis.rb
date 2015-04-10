#***********************************************************************/
#  Rafael Meireles Saback                                              */
#  rafael.saback@dfki.de                                               */
#  DFKI - BREMEN 2014                                                  */
#***********************************************************************/


require 'orocos'
require 'Qt'
require 'vizkit'
require './SupervisoryCode.rb'
include Orocos

Orocos.initialize

Orocos.run 'control_loop',
           'motion_model::Task' => 'MotionModel',
           'auv_control::WorldToAligned' => 'WorldToAligned',
           'auv_control::AlignedToBody'=> 'AlignedToBody' do

  ## Get the specific task context ##
  constantCommand   = TaskContext.get 'ConstantCommand'
  worldToAligned    = TaskContext.get 'WorldToAligned'
  pidPos            = TaskContext.get 'PIDPos'
  alignedToBody     = TaskContext.get 'AlignedToBody'
  pidVel            = TaskContext.get 'PIDVel'
  effortToControl   = TaskContext.get 'EffortToControl'
  motionModel       = TaskContext.get 'MotionModel'
  
  
  rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization
  
  configFilesFolder = ENV['AUTOPROJ_CURRENT_ROOT']+"/control/orogen/smith_predictor/scripts/"
	controllableDOFS = [:SURGE, :SWAY, :HEAVE, :YAW]	
  
  ##########################################################################
  #                         CONFIGURATION FILES
  ##########################################################################
  
  constantCommand.apply_conf_file("constant_command.yml")
  worldToAligned.apply_conf_file("world_to_aligned.yml")
  pidPos.apply_conf_file("pid.yml",["PIDPosition"])
  alignedToBody.apply_conf_file("aligned_to_body.yml")
  pidVel.apply_conf_file("pid.yml",["PIDVelocity"])
  effortToControl.apply_conf_file("effort_to_control.yml")
  motionModel.apply_conf_file("motion_model.yml")
  
  
	##########################################################################
  #                CONNECTING AND INITIALIZATING COMPONENTS
	##########################################################################

  # Connecting components
  constantCommand.cmd_out.connect_to          worldToAligned.cmd_in          
  worldToAligned.cmd_out.connect_to           pidPos.cmd_in
  pidPos.cmd_out.connect_to                   pidVel.cmd_in          
  pidVel.cmd_out.connect_to                   alignedToBody.cmd_in
  alignedToBody.cmd_out.connect_to            effortToControl.cmd_in
#  alignedToBody.cmd_out.connect_to            pidVel.cmd_in
#  pidVel.cmd_out.connect_to                   effortToControl.cmd_in
  effortToControl.cmd_out.connect_to          motionModel.cmd_in
  
  motionModel.cmd_out.connect_to              pidPos.pose_samples
  motionModel.cmd_out.connect_to              pidVel.pose_samples
  motionModel.cmd_out.connect_to              worldToAligned.pose_samples
  motionModel.cmd_out.connect_to              alignedToBody.orientation_samples
    
  
  # Configuring and starting components
  constantCommand.configure
  constantCommand.start
  worldToAligned.configure
  worldToAligned.start  
  pidPos.configure
  pidPos.start  
  alignedToBody.configure
  alignedToBody.start  
  pidVel.configure
  pidVel.start  
  effortToControl.configure
  effortToControl.start  
  motionModel.configure
  motionModel.start
 
 	##########################################################################
	#		     WIDGET CONFIGURATION (Check file: SupervisoryCode.rb)
	##########################################################################

           
  ## Defining the proxy for each task           
  constantCommandproxy = Orocos::Async.proxy("ConstantCommand")
  pidPosproxy = Orocos::Async.proxy("PIDPos")
  pidVelproxy = Orocos::Async.proxy("PIDVel")
  etcproxy = Orocos::Async.proxy("EffortToControl")
  motionmodelproxy = Orocos::Async.proxy("MotionModel")
  
  ## Defining the port variables using the proxys
  constantCommandPort = constantCommandproxy.port("cmd_out")
  pidPosPort = pidPosproxy.port("cmd_out")
  pidVelPort = pidVelproxy.port("cmd_out")
  thrusterControlPort = etcproxy.port("cmd_out")
  statesPort = motionmodelproxy.port("cmd_out")    

  # Connecting the vehicle model output to the 3D Visualization plugin  
  statesPort.connect_to do |sample, |
    rbs_3DVisualization.updateRigidBodyState(sample)
  end
#=begin  
  supervisory = Supervisory.new(constantCommand.cmd, pidVel.parallel_pid_settings, pidPos.parallel_pid_settings)
  #supervisory = Supervisory.new(constantCommand.cmd, pidPos.parallel_pid_settings)
  
  ## Connecting the components output to the widget for graphical exibition
  for i in 0..(controllableDOFS.size-1)
    
    
    supervisory.connect_position(statesPort, controllableDOFS[i])
    supervisory.connect_velocity(statesPort, controllableDOFS[i])
    supervisory.connect_pos_reference(constantCommandPort, controllableDOFS[i])
    supervisory.connect_vel_reference(pidPosPort, controllableDOFS[i])
    
    
    supervisory.connect_pid_position(pidPosPort, controllableDOFS[i])
    supervisory.connect_pid_velocity(pidVelPort, controllableDOFS[i])
  end
  supervisory.connect_thruster_control(thrusterControlPort)
  

  supervisory.show
  
  ## Timer for updating the pid properties that were set on the widget
  @timer = Qt::Timer.new
  @timer.start(10.0)
  @timer.connect(SIGNAL('timeout()')) do
            pidPos.parallel_pid_settings = supervisory.updatePIDParam(:POSITION)
            pidVel.parallel_pid_settings = supervisory.updatePIDParam(:VELOCITY)
            constantCommand.cmd = supervisory.updateReference
  end
#=end
  Vizkit.exec
    
end



