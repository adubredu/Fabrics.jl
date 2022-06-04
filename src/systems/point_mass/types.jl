mutable struct PointMass 
    x::Vector{Float64}
    ẋ::Vector{Float64} 
    r::Float64
    o::Vector{Vector{Float64}}
    or::Vector{Float64}
    k::Float64
    λ::Float64
    Δt::Float64
    g::Vector{Float64}
    show_tail::Bool 
    dynamic::Bool 
    obs_x
    obs_tail
    obs_o
    obstacle_speed::Float64
    task_maps::Vector{Symbol}

    function PointMass(robot_position, robot_velocity, robot_radius, obstacle_positions, obstacle_radii, goal_position)
        k = 0.5
        λ = 0.7
        Δt = 0.001
        obstacle_speed = 0.017
        task_maps = [:attractor, :repeller]
        new(robot_position, robot_velocity, robot_radius, obstacle_positions, 
            obstacle_radii, k, λ, Δt, goal_position, false, false, nothing, nothing, nothing, obstacle_speed, task_maps)
    end
end