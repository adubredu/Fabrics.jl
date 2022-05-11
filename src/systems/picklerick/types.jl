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
    θᵣ::Vector{Float64}
    Δt::Float64

    function PickleRick(obstacle_positions, obstacle_radii, joint_positions, joint_velocities, goal_position)
        time_step = 10E-4
        task_maps = [:repeller, :default_config, :lefthand_attractor]
        θᵣ = [π/2, 2π/3, π/6, 2π/3, π/6, π/2, π/3, 7π/12, 2π/3, 5π/12]

        new(obstacle_positions, obstacle_radii, joint_positions, joint_velocities, nothing, nothing, nothing, nothing, false, false, false, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, goal_position, task_maps, θᵣ, time_step)
    end
end