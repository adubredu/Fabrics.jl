function quat2euler(quat)
    q0 = quat.w; q1 = quat.x; q2 = quat.y; q3 = quat.z
    r = atan(2(q0*q1+q2*q3), 1-2(q1^2 + q2^2))
    p = asin(2*(q0*q2 - q3*q1))
    y = atan(2*(q0*q3 + q1*q2), 1-2*(q2^2+q3^2))
    return [r,p,y]
end

function mirror_joint_configurations!(observation, env::Digit)
    floating_translation = observation.base_translation
    floating_orientation = quat2euler(observation.base_orientation)
    floating_linear_velocity = observation.base_linear_velocity
    floating_angular_velocity = observation.base_angular_velocity 

    motor_position = observation.motor_position
    motor_velocity = observation.motor_velocity 

    joint_position = observation.joint_position 
    joint_velocity = observation.joint_velocity 

    joint_names = [collect(keys(env.motorID))..., collect(keys(env.jointID))...]

    motor_names = collect(keys(env.motorID))
    joint_names = collect(keys(env.jointID))

    for jn in joint_names
        joint = get_joint(env.mechanism, jn)
        if jn in motor_names
            set_minimal_coordinates!(env.mechanism, joint, [motor_position[env.motorID[jn]]])
            set_minimal_velocities!(env.mechanism, joint, [motor_velocity[env.motorID[jn]]])
            
        else
            set_minimal_coordinates!(env.mechanism, joint, [joint_position[env.jointID[jn]]])
            set_minimal_velocities!(env.mechanism, joint, [joint_velocity[env.jointID[jn]]])
        end
    end
    t = floating_translation
    o = floating_orientation
    set_minimal_coordinates!(env.mechanism, get_joint(env.mechanism, :auto_generated_floating_joint), [t[1]; t[2]; t[3]; o[1]; o[2]; o[3]])

    t = floating_linear_velocity
    o = floating_angular_velocity
    set_minimal_velocities!(env.mechanism, get_joint(env.mechanism, :auto_generated_floating_joint), [t[1]; t[2]; t[3]; o[1]; o[2]; o[3]])
end