function attractor_task_map(θ, env::MultiPlanarArm)
    x₉ = env.g
    _,_,_,_,_,_,_,_,_, x = link_poses(θ, env)
    return x - x₉
end

function repeller_task_map(θ, env::MultiPlanarArm)
    # os = env.o
    os = env.obstacle_observables
    rs = 0.75*env.r 
    x1, x2, x3, x4, x5, x6, x7, x8, x9, x10 = link_poses(θ, env) 
    xs = []
    for x in [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10]
        for (o,r) in zip(os, rs) 
            Δ =  (norm(x - o.val)/r)[1] - 1.0
            push!(xs, Δ)
        end
    end
    return xs
end
 

function attractor_fabric(x, ẋ, env::MultiPlanarArm)
    k = 100.0; αᵩ = 10.0; β=20.5; K=1.0
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -K*δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function repeller_fabric(x, ẋ, env::MultiPlanarArm)
    kᵦ = 0.2; αᵦ = 0.1 
    s = [v < 0 ? 1.0 : 0.0 for v in ẋ]
    M = diagm((s.*kᵦ) ./ (x.^2))
    ψ(θ) = αᵦ ./ (2θ.^8) 
    x = convert(Vector{Float64}, x)
    δx = ForwardDiff.jacobian(ψ, x) 
    ẍ = vec((-s .* ẋ.^2)' * δx)   
    return (M, ẍ)
end


function fabric_eval(x, ẋ, name::Symbol, env::MultiPlanarArm)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
end

function energize(ẍ, M, env::MultiPlanarArm; ϵ=1e-10)
    ẋ = env.θ̇  + ẍ*env.Δt 
    ẋ = ẋ/(norm(ẋ)) 
    ẍₑ = (I(size(M)[1]) - ϵ*ẋ*ẋ')*ẍ
    return ẍₑ
end


function multiplanar_arm_fabric_solve(θ, θ̇ , env::MultiPlanarArm)
    xₛ = []; ẋₛ = []; cₛ = []
    Mₛ = []; ẍₛ = []; Jₛ = []
    for t in env.task_maps
        ψ = eval(Symbol(t, :_task_map))
        x = ψ(θ, env) 
        ẋ = jvp(σ->ψ(σ, env), θ, θ̇ )
        c = jvp(σ -> jvp(σ->ψ(σ, env), σ, θ̇  ), θ, θ̇ )
        J = ForwardDiff.jacobian(σ->ψ(σ, env), θ)
        M, ẍ = fabric_eval(x, ẋ, t, env)
        push!(xₛ, x); push!(ẋₛ, ẋ); push!(cₛ, c) 
        push!(Mₛ, M); push!(ẍₛ, ẍ); push!(Jₛ, J) 
    end   
    Mᵣ = sum([J' * M * J for (J, M) in zip(Jₛ, Mₛ)])
    fᵣ = sum([J' * M * (ẍ - c) for (J, M, ẍ, c) in zip(Jₛ, Mₛ, ẍₛ, cₛ)])
    Mᵣ = convert(Matrix{Float64}, Mᵣ)
    ẍ = pinv(Mᵣ) * fᵣ 
    ẍ =  energize(ẍ, Mᵣ, env)
    return ẍ 
end