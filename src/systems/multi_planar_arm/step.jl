function step!(θ̈ ::Vector{Float64}, env::MultiPlanarArm)
    θ = env.θ + env.θ̇  * env.Δt
    θ̇ = env.θ̇ + θ̈  * env.Δt 
    vellims = 10.0*ones(length(θ))
    θ̇  = clamp.(θ̇ , -vellims, vellims)

    p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 = link_poses(θ, env)
    env.link_observables[] = [SVector(0.0, 0.0), SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2]), SVector(p4[1], p4[2]), SVector(p5[1], p5[2]), SVector(p6[1], p6[2]), SVector(p7[1], p7[2]), SVector(p8[1], p8[2]), SVector(p9[1], p9[2]), SVector(p10[1], p10[2])]
    
    env.joint_observables[] = [SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2]), SVector(p4[1], p4[2]), SVector(p5[1], p5[2]), SVector(p6[1], p6[2]), SVector(p7[1], p7[2]), SVector(p8[1], p8[2]), SVector(p9[1], p9[2]), SVector(p10[1], p10[2])]
    

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