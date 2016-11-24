require 'orocos'
require 'Qt'
require 'vizkit'
include Orocos

Orocos.initialize

 Orocos.run do

    flat_fish              = TaskContext.get 'gazebo:underwater:flat_fish' #poose_samples

    aligned_pos            = TaskContext.get 'auv_control_world_pos2aligned_pos'
    aligned_vel            = TaskContext.get 'auv_control_aligned_pos2aligned_vel'
    body_effort            = TaskContext.get 'auv_control_aligned_vel2body_effort'
    comp_effort            = TaskContext.get 'body_effort2compensated_effort'
    thruster               = TaskContext.get 'optimal_body_compensated_effort2body_thrusters' #expected_effort

    ##################
    # ports
    #################
    flat_fish_pose = flat_fish.pose_samples
    ff_pose = flat_fish_pose.reader

    aligned_pos_cmd_out = aligned_pos.cmd_out
    aligned_pos_cmd = aligned_pos_cmd_out.reader

    aligned_vel_cmd_out = aligned_vel.cmd_out
    aligned_vel_cmd = aligned_vel_cmd_out.reader

    body_effort_cmd_out = body_effort.cmd_out
    body_effort_cmd = body_effort_cmd_out.reader

    comp_effort_cmd_out = comp_effort.cmd_out
    comp_effort_cmd = comp_effort_cmd_out.reader

    thruster_cmd_out = thruster.cmd_out
    thruster_cmd = thruster_cmd_out.reader

    ##################
    # counters
    ##################
    last_time_ff_pose = Time.at(0)
    last_system_time_ff_pose = Time.at(0)
    count_repeated_ff_pose = 0

    last_time_aligned_pos_cmd = Time.at(0)
    count_repeated_aligned_pos_cmd = 0

    last_time_aligned_vel_cmd = Time.at(0)
    count_repeated_aligned_vel_cmd = 0

    last_time_body_effort_cmd = Time.at(0)
    count_repeated_body_effort_cmd = 0

    last_time_comp_effort_cmd = Time.at(0)
    count_repeated_comp_effort_cmd = 0

    last_time_thruster_cmd = Time.at(0)
    last_system_time_thruster_cmd = Time.at(0)
    count_repeated_thruster_cmd = 0

    puts "monitoring"
    while true do
        if sample = ff_pose.read_new
            nw = Time.now()
            if (last_time_ff_pose.usec == sample.time.usec)
                puts "count_repeated_ff_pose #{count_repeated_ff_pose}"
                count_repeated_ff_pose += 1
            else
                step = (sample.time - last_time_ff_pose)
                puts "pose step: #{step}"
                puts "system step: #{nw - last_system_time_ff_pose} "
                puts " "
            end
            last_time_ff_pose = sample.time
            last_system_time_ff_pose = nw
        end

        # if sample = aligned_pos_cmd.read_new
        #     if (last_time_aligned_pos_cmd.usec == sample.time.usec)
        #         puts "count_repeated_aligned_pos_cmd #{count_repeated_aligned_pos_cmd}: #{last_time_aligned_pos_cmd} + #{last_time_aligned_pos_cmd.usec}"
        #         count_repeated_aligned_pos_cmd += 1
        #     else
        #         count_repeated_aligned_pos_cmd = 0
        #     end
        #     last_time_aligned_pos_cmd = sample.time
        # end

        # if sample = aligned_vel_cmd.read_new
        #     if (last_time_aligned_vel_cmd.usec == sample.time.usec)
        #         puts "count_repeated_aligned_vel_cmd #{count_repeated_aligned_vel_cmd}: #{last_time_aligned_vel_cmd} + #{last_time_aligned_vel_cmd.usec}"
        #         count_repeated_aligned_vel_cmd += 1
        #     end
        #     last_time_aligned_vel_cmd = sample.time
        # end
        #
        # if sample = body_effort_cmd.read_new
        #     if (last_time_body_effort_cmd.usec == sample.time.usec)
        #         puts "count_repeated_body_effort_cmd #{count_repeated_body_effort_cmd}: #{last_time_body_effort_cmd} + #{last_time_comp_effort_cmd.usec}"
        #         count_repeated_body_effort_cmd += 1
        #     end
        #     last_time_body_effort_cmd = sample.time
        # end
        #
        # if sample = comp_effort_cmd.read_new
        #     if (last_time_comp_effort_cmd.usec == sample.time.usec)
        #         puts "count_repeated_comp_effort_cmd #{count_repeated_comp_effort_cmd}: #{last_time_comp_effort_cmd} + #{last_time_comp_effort_cmd.usec}"
        #         count_repeated_comp_effort_cmd += 1
        #     else
        #         count_repeated_comp_effort_cmd = 0
        #     end
        #     last_time_comp_effort_cmd = sample.time
        # end

        if sample = thruster_cmd.read_new
            nw = Time.now()
            if (last_time_thruster_cmd.usec == sample.time.usec)
                puts "count_repeated_thruster_cmd #{count_repeated_thruster_cmd}: #{last_time_thruster_cmd}"
                count_repeated_thruster_cmd += 1
            else
                count_repeated_thruster_cmd = 0
                step = (sample.time - last_time_thruster_cmd)
                puts "thruster step: #{step}"
                puts "system step: #{nw - last_system_time_thruster_cmd} "
                puts " "
            end
            last_system_time_thruster_cmd = nw
            last_time_thruster_cmd = sample.time
        end
    end

end
