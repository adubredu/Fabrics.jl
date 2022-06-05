function step!(θ̈ ::Vector{Float64}, env::PlanarArm)
    θ = env.θ + env.θ̇  * env.Δt
    θ̇ = env.θ̇ + θ̈  * env.Δt 
    vellims = 10.0*ones(length(θ))
    θ̇  = clamp.(θ̇ , -vellims, vellims)

    p1, p2 = link_poses(θ, env)
    env.link_observables[] = [SVector(0.0, 0.0), SVector(p1[1], p1[2]), SVector(p2[1], p2[2])]
    env.joint_observables[] = [SVector(p1[1], p1[2]), SVector(p2[1], p2[2])]

    env.θ = θ
    env.θ̇ = θ̇

    if env.dynamic
        for ob in env.obstacle_observables
            val = collect(ob.val)
            val[1] -= 0.0125
            ob[] = SVector(val...)
            if ob[][1] < -10.0
                val[1] = 10
                ob[] = SVector(val...)
            end
        end
    end

end