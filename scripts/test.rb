#***********************************************************************/
#                                                                      */
#                                                                      */
# FILE --- test.rb                                                     */
#                                                                      */
# PURPOSE --- Tests for the motion model component                     */
#                                                                      */
#  Rafael Meireles Saback                                              */
#  rafael.saback@dfki.de                                               */
#  DFKI - BREMEN 2014                                                  */
#***********************************************************************/


require 'orocos'
require 'Qt'
require 'vizkit'
include Orocos

Orocos.initialize

Orocos.run 'auv_control::ConstantCommand'         => 'ConstantCommand',
           'auv_control::AccelerationController'  => 'AccelerationController',
           'uwv_motion_model::Task'               => 'MotionModel' do
#            :valgrind=>['AccelerationController'],
#            :output => '%m-%p.log' do
             

	## Get the specific task context ##
	constantCommand        = TaskContext.get 'ConstantCommand'
	accelerationController = TaskContext.get 'AccelerationController'
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

  
  accelerationController.apply_conf_file("acceleration_controller.yml",["effort_control"])
  motionModel.apply_conf_file("uwv_motion_model.yml",["effort_control"])
#  accelerationController.apply_conf_file(config_files_folder+"acceleration_controller.yml",["test_acc_controller"])
#  motionModel.apply_conf_file(config_files_folder+"uwv_motion_model.yml",["test_acc_controller"])

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

	constantCommand.cmd_out.connect_to accelerationController.cmd_in
	accelerationController.cmd_out.connect_to motionModel.cmd_in
	
	# Configuring and starting the component
  constantCommand.configure
  constantCommand.start
  accelerationController.configure
  accelerationController.start
  motionModel.configure
  motionModel.start
  
  motionmodelproxy = Orocos::Async.proxy("MotionModel")
  statesPort = motionmodelproxy.port("cmd_out")    

  rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization

  # Connecting the vehicle model output to the 3D Visualization plugin  
  statesPort.connect_to do |sample, |
    rbs_3DVisualization.updateRigidBodyState(sample)
  end
  

  Vizkit.exec

end
