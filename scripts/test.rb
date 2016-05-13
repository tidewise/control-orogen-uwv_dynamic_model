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
           'uwv_dynamic_model::Task'               => 'MotionModel' do
#            :valgrind=>['AccelerationController'],
#            :output => '%m-%p.log' do


	## Get the specific task context ##
	constantCommand        = TaskContext.get 'ConstantCommand'
	motionModel            = TaskContext.get 'MotionModel'

  init = constantCommand.cmd
  init.linear[0] = 2
  init.linear[1] = 0
  init.linear[2] = 0
  init.angular[0] = 1
  init.angular[1] = 0
  init.angular[2] = 2
  constantCommand.cmd = init

  ##########################################################################
  #                       CONFIGURATION FILES
  ##########################################################################

  #motionModel.apply_conf_file("uwv_dynamic_model.yml",["default"])
  motionModel.apply_conf_file("uwv_dynamic_model.yml",["angular"])

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

	constantCommand.cmd_out.connect_to motionModel.cmd_in

	# Configuring and starting the component
  constantCommand.configure
  constantCommand.start
  motionModel.configure
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
