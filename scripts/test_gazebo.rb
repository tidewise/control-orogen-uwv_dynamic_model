require 'orocos'
require 'Qt'
require 'vizkit'
include Orocos

Orocos.initialize

Orocos.run  'uwv_dynamic_model::Task' => 'motionModel',
            'auv_control::ThrusterForce2BodyEffort' => 'efforts' do
#            :valgrind=>['AccelerationController'],
#            :output => '%m-%p.log' do


	## Get the specific task context ##
	motionModel            = TaskContext.get 'motionModel'
    efforts                = TaskContext.get 'efforts'
    thruster               = TaskContext.get 'optimal_body_compensated_effort2body_thrusters' #expected_effort
    flat_fish              = TaskContext.get 'gazebo:underwater:flat_fish' #poose_samples


    ##########################################################################
    #                       CONFIGURATION FILES
    ##########################################################################

    #motionModel.apply_conf_file("uwv_dynamic_model.yml",["default"])
    motionModel.apply_conf_file("uwv_dynamic_model.yml",["gazebo"])
    efforts.apply_conf_file("thruster_force_2_body_effort.yml")

	##########################################################################
	#		                    COMPONENT INPUT PORTS
	##########################################################################

    # thurster = Types::AuvControl::LinearAngular6DCommand.new

    # thurster = efforts
	thruster.cmd_out.connect_to efforts.thruster_forces
    efforts.body_efforts.connect_to motionModel.cmd_in

    # Configuring and starting the component
    motionModel.configure
    efforts.configure
    motionModel.start
    efforts.start

    motionmodelproxy = Orocos::Async.proxy("MotionModel")
    statesPort = motionmodelproxy.port("pose_samples")

    rbs_3DVisualization = Vizkit.default_loader.RigidBodyStateVisualization

    # Connecting the vehicle model output to the 3D Visualization plugin
    statesPort.connect_to do |sample, |
        rbs_3DVisualization.updateRigidBodyState(sample)
    end


    Vizkit.exec

end
