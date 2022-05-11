function step!(θ̈ ::Vector{Float64}, env::PickleRick)
    θ = env.θ + env.θ̇  * env.Δt
    θ̇ = env.θ̇ + θ̈  * env.Δt 
    vellims = 10.0*ones(length(θ))
    θ̇  = clamp.(θ̇ , -vellims, vellims)
    
    chains = link_poses(θ, env)
    env.body_observables[1][] = [SVector(a[1], a[2]) for a in chains[1]]
    env.body_observables[2][] = [SVector(a[1], a[2]) for a in chains[2]]
    env.body_observables[3][] = [SVector(a[1], a[2]) for a in chains[3]]
    env.body_observables[4][] = [SVector(a[1], a[2]) for a in chains[4]]
    env.body_observables[5][] = [SVector(a[1], a[2]) for a in chains[5]]
    env.body_observables[6][] = [SVector(chains[3][3][1], chains[3][3][2])]

    env.θ = θ
    env.θ̇ = θ̇

    if env.dynamic
        for ob in env.obstacle_observables
            val = collect(ob.val)
            val[1] -= 0.00625
            ob[] = SVector(val...)
            if ob[][1] < -10.0
                val[1] = 10
                ob[] = SVector(val...)
            end
        end
    end



end