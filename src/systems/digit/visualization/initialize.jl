function get_digit(;
    timestep=0.01,
    gravity=-9.81,
    friction_coefficient=0.8,
    spring=0.0,
    damper=0.0,
    contact_feet=false,
    contact_body=false,
    model_type=:simple,
    T=Float64)

    path = joinpath(@__DIR__, "model/digit_model.urdf")
    mech = Mechanism(path, !(model_type==:armless), T, 
        gravity=gravity, 
        timestep=timestep, 
        spring=spring, 
        damper=damper)
        
    for joint in mech.joints[2:end]
        joint.damper = true
        joint.spring = true
        joint.translational.spring=spring
        joint.translational.damper=damper
        joint.rotational.spring=spring
        joint.rotational.damper=damper
    end
    set_minimal_coordinates!(mech, get_joint(mech, :auto_generated_floating_joint), [0.0; 0.0; 0.9385; 0.0; 0.0; 0.0])    

    return mech
end

function initialize_digit!(mechanism::Mechanism;
    model_type=:simple,
    body_position=[0.0, 0.0, 0.0],
    body_orientation=[0.0, 0.0, 0.0]) where T
    body_position +=  [0.0, 0.0, 0.9385]
    try
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :auto_generated_floating_joint), [body_position; body_orientation])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :LeftHipRoll), [0.337])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :RightHipRoll), [-0.337])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :LeftToePitch), [-0.126])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :RightToePitch), [0.126])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :LeftShoulderPitch), [0.463])
        set_minimal_coordinates!(mechanism, get_joint(mechanism, :RightShoulderPitch), [-0.463])
    catch
        println("cannot set init configuration")
        nothing
    end
    Dojo.zero_velocity!(mechanism)
    return nothing
end
