mutable struct PickleRick 
    o::Vector{Vector{Float64}}
    r::Vector{Float64}
    θ::Vector{Float64}
    θ̇ ::Vector{Float64} 
    body_observables
    robot_keypoint_observables 
    obstacle_observables
    tail_observable
    dynamic::Bool 
    trail::Bool
    show_contacts::Bool 
    l0::Float64
    l1::Float64 
    l2::Float64 
    l3::Float64 
    l4::Float64
    l5::Float64 
    l6::Float64
    l7::Float64 
    l8::Float64
    l9::Float64 
    l10::Float64
    w::Float64
    g::Vector{Float64}
    task_maps::Vector{Symbol}
    Δt::Float64

    function PickleRick(obstacle_positions, obstacle_radii, joint_positions, joint_velocities, goal_position)
        time_step = 0.001
        task_maps = [:repeller]
        new(obstacle_positions, obstacle_radii, joint_positions, joint_velocities, nothing, nothing, nothing, nothing, false, false, false, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, goal_position, task_maps, time_step)
    end
end