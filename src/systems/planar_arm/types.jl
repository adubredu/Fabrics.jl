mutable struct PlanarArm 
    θ::Vector{Float64}
    θ̇ ::Vector{Float64}
    o::Vector{Vector{Float64}}
    r::Vector{Float64}
    l1::Float64
    l2::Float64
    l3::Float64
    Δt::Float64
    show_contacts::Bool 
    g::Vector{Float64}
    link_observables 
    joint_observables 
    obstacle_observables
    
    function PlanarArm(robot_position, robot_velocity, obstacle_positions, obstacle_radii, goal_position)
        new(robot_position, robot_velocity, obstacle_positions, obstacle_radii, 2.5, 2.5, 2.5, 10E-4, false, goal_position, nothing, nothing, nothing)
    end
end