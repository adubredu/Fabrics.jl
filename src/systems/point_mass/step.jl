function step!(ẍ::Vector{Float64}, env::PointMass)
    x = env.x + env.ẋ * env.Δt
    ẋ = env.ẋ + ẍ * env.Δt 
    ẋ = clamp.(ẋ, -15.0, 15.0)
    
    env.obs_x[] = x 

    if env.show_tail
        push!(env.obs_tail[], SVector(x...))
        env.obs_tail[] = env.obs_tail[]
    end

    

    env.x = x 
    env.ẋ = ẋ
end

function move_obstacles!(env::PointMass) 
    for ob in env.obs_o
        val = collect(ob.val)
        val[2] -= env.obstacle_speed
        ob[] = SVector(val...)
        if ob[][2] < -10.0
            val[2] = 10
            ob[] = SVector(val...)
        end
    end 
end