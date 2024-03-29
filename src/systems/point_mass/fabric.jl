function jvp(f, x, u)
    return ForwardDiff.derivative(t->f(x + t*u), 0.0)
end

function attractor_task_map(θ, env::PointMass)
    θ₉ = env.g
    return θ - θ₉
end

function repeller_task_map(θ, env::PointMass)
    os = env.obs_o
    xs = []
    for (i, o) in enumerate(os)
        r = 0.5*env.or[i]
        o = o.val
        x = (norm(θ - o)/r) - 1.0
        push!(xs, x)
    end
    return xs
end

function attractor_fabric(x, ẋ, env::PointMass)
    k = 150.0; αᵩ = 10; β=100.5; K=10
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -K*δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function repeller_fabric(x, ẋ, env::PointMass)
    kᵦ = 75; αᵦ = 50.0 
    s = [v < 0 ? 1.0 : 0.0 for v in ẋ]
    M = diagm((s.*kᵦ) ./ (x.^2))
    ψ(θ) = αᵦ ./ (2*θ.^8)
    x = convert(Vector{Float64}, x)
    δx = ForwardDiff.jacobian(ψ, x)
    ẍ = vec((-s .* ẋ.^2)' * δx)
    return (M, ẍ)
end

function fabric_eval(x, ẋ, name::Symbol, env::PointMass)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
end

function energize(ẍ, M, env::PointMass; ϵ=1e-1)
    ẋ = env.ẋ + ẍ*env.Δt 
    ẋ = ẋ/(norm(ẋ)) 
    ẍₑ = (I(size(M)[1]) - ϵ*ẋ*ẋ')*ẍ
    return ẍₑ
end

function pointmass_fabric_solve(θ, θ̇ , env::PointMass)
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
    ẍ = energize(ẍ, Mᵣ, env)
    return ẍ 
end

