mutable struct PointMass 
    x::Vector{Float64}
    ẋ::Vector{Float64}
    o::Vector{Float64}
    r::Float64
    k::Float64
    λ::Float64
    Δt::Float64
    g::Vector{Float64}
    obs_x
    obs_tail
    task_maps::Vector{Symbol}

    function PointMass(robot_position, robot_velocity, obstacle_position, obstacle_radius, goal_position)
        k = 0.5
        λ = 0.7
        Δt = 0.001
        task_maps = [:attractor, :repeller]
        new(robot_position, robot_velocity, obstacle_position, 
            obstacle_radius, k, λ, Δt, goal_position, nothing, nothing, task_maps)
    end
end