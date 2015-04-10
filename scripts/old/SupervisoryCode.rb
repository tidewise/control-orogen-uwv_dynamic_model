###########################################################################
##                                                                       ##
##                      SUPERVISORY SYSTEM CLASS                         ##
##                                                                       ##
###########################################################################


class Supervisory
  
  def initialize(referencesProperty, pidVelParam, pidPosParam, parent = nil)

    @window = Vizkit.load(File.join(File.dirname(__FILE__), 'SupervisoryGUI.ui'), parent)
    @references = referencesProperty
    @pidVelParam = pidVelParam
    @pidPosParam = pidPosParam
    @pidPosDOF = 0   #Initial DOF = Surge
    @pidVelDOF = 0   #Initial DOF = Surge


  ##########################################################################
  #                     ONFIGURING PLOT2D OBJETCS                          #
  ##########################################################################

    ## Array of Plot2d objets. This array will be used to set up the Plot2d objects.
    @plotArray = [ @window.surge_pose,          @window.sway_pose,          @window.heave_pose,          @window.yaw_pose,
                   @window.pred_surge_pose,     @window.pred_sway_pose,     @window.pred_heave_pose,     @window.pred_yaw_pose,
                   @window.surge_vel,           @window.sway_vel,           @window.heave_vel,           @window.yaw_vel,
                   @window.pred_surge_vel,      @window.pred_sway_vel,      @window.pred_heave_vel,      @window.pred_yaw_vel,
                   @window.pid_surge_pose,      @window.pid_sway_pose,      @window.pid_heave_pose,      @window.pid_yaw_pose,
                   @window.pid_surge_vel,       @window.pid_sway_vel,       @window.pid_heave_vel,       @window.pid_yaw_vel,
                   @window.thruster_1,          @window.thruster_2,         @window.thruster_3,          @window.thruster_4,
                   @window.thruster_9]
                   
    numberOfDOFS = 4
    
    ## Setting the Plot2d objetcs properties
    for i in 0..(@plotArray.size - 1)
        
        @plotArray[i].set_y_axis_scale(-2.1, 2.1)    
        @plotArray[i].options[:time_window] = 50        
        @plotArray[i].getLegend.setVisible(false)          
        #@plotArray[i].options[:update_period] = sampleTime
        
        if i < @plotArray.size - 5
          
          round = i/numberOfDOFS       

          if (i%(4*round == 0? 4: 4*round) == 0)
            @plotArray[i].setTitle("Surge")
          elsif (i == 1 || i%(i < 4? 4: 1 + 4*round) == 0)
            @plotArray[i].setTitle("Sway")
          elsif (i%(2 + 4*round) == 0)  
            @plotArray[i].setTitle("Heave")
          elsif (i%(3 + 4*round) == 0)  
            @plotArray[i].setTitle("Yaw")
          end
          
        else
          if (i == 24)
            @plotArray[i].setTitle("Thruster 1")
          elsif (i == 24)
            @plotArray[i].setTitle("Thruster 2")
          elsif (i == 26)  
            @plotArray[i].setTitle("Thruster 3")
          elsif (i == 27)  
            @plotArray[i].setTitle("Thruster 4")
          elsif (i == 28)  
            @plotArray[i].setTitle("Thruster 5")
          end
        end
            
    end
    
  ##########################################################################
  #                   GETING INITIAL PID TUNING VALUES                     #
  ##########################################################################    
    
    ## Setting initial values for the comboBoxes of the position and velocity PIDs
    
    @window.pid_pos_kp.setValue(@pidPosParam.linear[@pidPosDOF].Kp)
    @window.pid_pos_ki.setValue(@pidPosParam.linear[@pidPosDOF].Ki)
    @window.pid_pos_kd.setValue(@pidPosParam.linear[@pidPosDOF].Kd)
    @window.pid_pos_umax.setValue(@pidPosParam.linear[@pidPosDOF].YMax)
    @window.pid_pos_umin.setValue(@pidPosParam.linear[@pidPosDOF].YMin)
    
    @window.pid_vel_kp.setValue(@pidVelParam.linear[@pidVelDOF].Kp)
    @window.pid_vel_ki.setValue(@pidVelParam.linear[@pidVelDOF].Ki)
    @window.pid_vel_kd.setValue(@pidVelParam.linear[@pidVelDOF].Kd)
    @window.pid_vel_umax.setValue(@pidVelParam.linear[@pidVelDOF].YMax)
    @window.pid_vel_umin.setValue(@pidVelParam.linear[@pidVelDOF].YMin)
    
  ##########################################################################
  #                        SETTING THE REFERENCE                           #
  ##########################################################################    
    
    ## Callback for the click event on the button "APPLY" in the "Desired Position"
    
    @window.set_reference.connect(SIGNAL('clicked()')) do || 
      @references.linear[0] = @window.ref_x_axis.value
      @references.linear[1] = @window.ref_y_axis.value
      @references.linear[2] = @window.ref_z_axis.value
      @references.angular[0]= 0
      @references.angular[1]= 0
      @references.angular[2] = @window.ref_yaw.value*Math::PI/180
    end

  ##########################################################################
  #                         PID POSITION PARAMETERS                        #
  ##########################################################################
    
    
    ## Loading the parameters values for the new DOF that was chosen on the spinBox (Surge, Sway, etc..)
    
    @window.pid_pos_selection.connect(SIGNAL('currentIndexChanged(const QString &)')) do |value|      
      case value
      when "Surge"
        @pidPosDOF = 0          
      when "Sway"
        @pidPosDOF = 1          
      when "Heave"
        @pidPosDOF = 2          
      when "Roll"
        @pidPosDOF = 3          
      when "Pitch"
        @pidPosDOF = 4          
      when "Yaw"
        @pidPosDOF = 5
      end
      if @pidPosDOF < 3
        @window.pid_pos_kp.setValue(@pidPosParam.linear[@pidPosDOF].Kp)
        @window.pid_pos_ki.setValue(@pidPosParam.linear[@pidPosDOF].Ki)
        @window.pid_pos_kd.setValue(@pidPosParam.linear[@pidPosDOF].Kd)
        @window.pid_pos_umax.setValue(@pidPosParam.linear[@pidPosDOF].YMax)
        @window.pid_pos_umin.setValue(@pidPosParam.linear[@pidPosDOF].YMin)
      else
        @window.pid_pos_kp.setValue(@pidPosParam.angular[@pidPosDOF-3].Kp)
        @window.pid_pos_ki.setValue(@pidPosParam.angular[@pidPosDOF-3].Ki)
        @window.pid_pos_kd.setValue(@pidPosParam.angular[@pidPosDOF-3].Kd)
        @window.pid_pos_umax.setValue(@pidPosParam.angular[@pidPosDOF-3].YMax)
        @window.pid_pos_umin.setValue(@pidPosParam.angular[@pidPosDOF-3].YMin)
      end
    end
    
    ## Callback for the click event on the button "APPLY" in the PID position parameters
    
    @window.set_pid_pos_param.connect(SIGNAL('clicked()')) do |value|
      if @pidPosDOF < 3
        @pidPosParam.linear[@pidPosDOF].Kp = @window.pid_pos_kp.value
        @pidPosParam.linear[@pidPosDOF].Ki = @window.pid_pos_ki.value
        @pidPosParam.linear[@pidPosDOF].Kd = @window.pid_pos_kd.value
        @pidPosParam.linear[@pidPosDOF].YMax = @window.pid_pos_umax.value
        @pidPosParam.linear[@pidPosDOF].YMin = @window.pid_pos_umin.value
      else
        @pidPosParam.angular[@pidPosDOF-3].Kp = @window.pid_pos_kp.value
        @pidPosParam.angular[@pidPosDOF-3].Ki = @window.pid_pos_ki.value
        @pidPosParam.angular[@pidPosDOF-3].Kd = @window.pid_pos_kd.value
        @pidPosParam.angular[@pidPosDOF-3].YMax = @window.pid_pos_umax.value
        @pidPosParam.angular[@pidPosDOF-3].YMin = @window.pid_pos_umin.value
      end
    end
  
  ##########################################################################
  #                         PID VELOCITY PARAMETERS                        #
  ##########################################################################
  
    ## Loading the parameters values for the new DOF that was chosen on the spinBox (Surge, Sway, etc..)  
  
    @window.pid_vel_selection.connect(SIGNAL('currentIndexChanged(const QString &)')) do |value|      
      case value
      when "Surge"
        @pidVelDOF = 0          
      when "Sway"
        @pidVelDOF = 1          
      when "Heave"
        @pidVelDOF = 2          
      when "Roll"
        @pidVelDOF = 3          
      when "Pitch"
        @pidVelDOF = 4          
      when "Yaw"
        @pidVelDOF = 5
      end
      if @pidVelDOF < 3
        @window.pid_vel_kp.setValue(@pidVelParam.linear[@pidVelDOF].Kp)
        @window.pid_vel_ki.setValue(@pidVelParam.linear[@pidVelDOF].Ki)
        @window.pid_vel_kd.setValue(@pidVelParam.linear[@pidVelDOF].Kd)
        @window.pid_vel_umax.setValue(@pidVelParam.linear[@pidVelDOF].YMax)
        @window.pid_vel_umin.setValue(@pidVelParam.linear[@pidVelDOF].YMin)
      else
        @window.pid_vel_kp.setValue(@pidVelParam.angular[@pidVelDOF-3].Kp)
        @window.pid_vel_ki.setValue(@pidVelParam.angular[@pidVelDOF-3].Ki)
        @window.pid_vel_kd.setValue(@pidVelParam.angular[@pidVelDOF-3].Kd)
        @window.pid_vel_umax.setValue(@pidVelParam.angular[@pidVelDOF-3].YMax)
        @window.pid_vel_umin.setValue(@pidVelParam.angular[@pidVelDOF-3].YMin)
      end
    end
    
    ## Callback for the click event on the button "APPLY" in the PID velocity parameters
    
    @window.set_pid_vel_param.connect(SIGNAL('clicked()')) do |value|
      if @pidVelDOF < 3
        @pidVelParam.linear[@pidVelDOF].Kp = @window.pid_vel_kp.value
        @pidVelParam.linear[@pidVelDOF].Ki = @window.pid_vel_ki.value
        @pidVelParam.linear[@pidVelDOF].Kd = @window.pid_vel_kd.value
        @pidVelParam.linear[@pidVelDOF].YMax = @window.pid_vel_umax.value
        @pidVelParam.linear[@pidVelDOF].YMin = @window.pid_vel_umin.value
      else
        @pidVelParam.angular[@pidVelDOF-3].Kp = @window.pid_vel_kp.value
        @pidVelParam.angular[@pidVelDOF-3].Ki = @window.pid_vel_ki.value
        @pidVelParam.angular[@pidVelDOF-3].Kd = @window.pid_vel_kd.value
        @pidVelParam.angular[@pidVelDOF-3].YMax = @window.pid_vel_umax.value
        @pidVelParam.angular[@pidVelDOF-3].YMin = @window.pid_vel_umin.value
      end
    end
    
  end


  ##########################################################################
  #                         UPDATE PID PARAM                               #
  ##########################################################################
    
  ## This function will be executed every 0.01 seconds in order to update the PIDs parameters      
  def updatePIDParam(type)
    if type == :POSITION
      return @pidPosParam
    elsif type == :VELOCITY
      return @pidVelParam
    else
      puts "Your argument (#{type}) was invalid. You should choose between :POSITION or :VELOCITY."
    end
  end
  
  ##########################################################################
  #                         UPDATE REFERENCE                        #
  ##########################################################################
    
  ## This function will be executed every 0.01 seconds in order to update the PIDs parameters      
  def updateReference
    return @references
  end

  ##########################################################################
  #                            GETTING SAMPLES                             #
  ##########################################################################
  
  ## Getting position reference
  def connect_pos_reference (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.surge_pose.update(sample.linear[0], "Ref")
                  @window.pred_surge_pose.update(sample.linear[0], "Ref")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.sway_pose.update(sample.linear[1], "Ref")
                  @window.pred_sway_pose.update(sample.linear[1], "Ref")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.heave_pose.update(sample.linear[2], "Ref")
                  @window.pred_heave_pose.update(sample.linear[2], "Ref")
              end
                                 
          when :YAW
              @port.connect_to do |sample, _|
                  @window.yaw_pose.update(sample.angular[2]*180/Math::PI, "Ref")
                  @window.pred_yaw_pose.update(sample.angular[2]*180/Math::PI, "Ref")
              end
    end
  end
    

  ## Getting postion samples    
  def connect_position (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.surge_pose.update(sample.position[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.sway_pose.update(sample.position[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.heave_pose.update(sample.position[2], "Heave")
              end
          when :YAW
              @port.connect_to do |sample, _|
                  @window.yaw_pose.update(sample.orientation.to_euler[0]*180/Math::PI, "Yaw")
              end
    end
  end

  ## Getting predicted postion samples    
  def connect_predicted_position (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.pred_surge_pose.update(sample.position[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.pred_sway_pose.update(sample.position[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.pred_heave_pose.update(sample.position[2], "Heave")
              end
          when :YAW
              @port.connect_to do |sample, _|
                  @window.pred_yaw_pose.update(sample.orientation.to_euler[0]*180/Math::PI, "Yaw")
              end
    end
  end

  ## Getting velocity reference
  def connect_vel_reference (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.surge_vel.update(sample.linear[0], "Ref")
                  @window.pred_surge_vel.update(sample.linear[0], "Ref")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.sway_vel.update(sample.linear[1], "Ref")
                  @window.pred_sway_vel.update(sample.linear[1], "Ref")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.heave_vel.update(sample.linear[2], "Ref")
                  @window.pred_heave_vel.update(sample.linear[2], "Ref")
              end
                                 
          when :YAW
              @port.connect_to do |sample, _|
                  @window.yaw_vel.update(sample.angular[2], "Ref")
                  @window.pred_yaw_vel.update(sample.angular[2], "Ref")
              end
    end
  end
  
  ## Getting velocity samples
  def connect_velocity (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.surge_vel.update(sample.velocity[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.sway_vel.update(sample.velocity[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.heave_vel.update(sample.velocity[2], "Heave")
              end
          when :YAW
              @port.connect_to do |sample, _|
                  @window.yaw_vel.update(sample.angular_velocity[2], "Yaw")
              end
    end
  end

  ## Getting predicted velocity samples
  def connect_predicted_velocity (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.pred_surge_vel.update(sample.velocity[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.pred_sway_vel.update(sample.velocity[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.pred_heave_vel.update(sample.velocity[2], "Heave")
              end
          when :YAW
              @port.connect_to do |sample, _|
                  @window.pred_yaw_vel.update(sample.angular_velocity[2], "Yaw")
              end
    end
  end

  ## Getting pid control samples
  def connect_pid_position (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.pid_surge_pose.update(sample.linear[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.pid_sway_pose.update(sample.linear[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.pid_heave_pose.update(sample.linear[2], "Heave")
              end
                                 
          when :YAW
              @port.connect_to do |sample, _|
                  @window.pid_yaw_pose.update(sample.angular[2]*180/Math::PI, "Yaw")
              end
    end
  end


 ## Getting pid velocity samples
  def connect_pid_velocity (port, dof)
    @port = port
    case dof
          when :SURGE
              @port.connect_to do |sample, _|
                  @window.pid_surge_vel.update(sample.linear[0], "Surge")
              end      
          when :SWAY
              @port.connect_to do |sample, _|
                  @window.pid_sway_vel.update(sample.linear[1], "Sway")
              end      
          when :HEAVE
              @port.connect_to do |sample, _|
                  @window.pid_heave_vel.update(sample.linear[2], "Heave")
              end
                                 
          when :YAW
              @port.connect_to do |sample, _|
                  @window.pid_yaw_vel.update(sample.angular[2], "Yaw")
              end
    end
  end

  ## Getting nonlinear control samples
  def connect_thruster_control (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.thruster_1.update(sample.elements[0].raw, "Thruster 1")
        @window.thruster_2.update(sample.elements[1].raw, "Thruster 2")
        @window.thruster_3.update(sample.elements[2].raw, "Thruster 3")
        @window.thruster_4.update(sample.elements[3].raw, "Thruster 4")
    end      
    
  end
  
  
  def show
    @window.show
  end

  
end
