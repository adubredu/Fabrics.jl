mutable struct PointMass 
    x::Vector{Float64}
    ẋ::Vector{Float64}
    o::Vector{Float64}
    r::Float64
    k::Float64
    λ::Float64
    Δt::Float64
    obs_x
    obs_tail

    function PointMass(robot_position, robot_velocity, obstacle_position, obstacle_radius)
        k = 0.5
        λ = 0.7
        Δt = 0.001
        new(robot_position, robot_velocity, obstacle_position, 
            obstacle_radius, k, λ, Δt, nothing, nothing)
    end
end