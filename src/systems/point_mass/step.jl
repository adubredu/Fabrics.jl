function step!(ẍ::Vector{Float64}, env::PointMass)
    x = env.x + env.ẋ * env.Δt
    ẋ = env.ẋ + ẍ * env.Δt 
    # ẋ = clamp.(ẋ, -10.0, 10.0)
    
    env.obs_x[] = x 
    push!(env.obs_tail[], SVector(x...))
    env.obs_tail[] = env.obs_tail[]

    env.x = x 
    env.ẋ = ẋ
end