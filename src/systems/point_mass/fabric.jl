function jvp(f, x, u)
    return ForwardDiff.derivative(t->f(x + t*u), 0.0)
end

function attractor_task_map(θ, env::PointMass)
    θ₉ = env.g
    return θ - θ₉
end

function repeller_task_map(θ, env::PointMass)
    o = env.o
    r = env.r 
    x = (norm(θ - o)/r) - 1.0
    return [x]
end

function attractor_fabric(x, ẋ, env::PointMass)
    k = 10.0; αᵩ = 10.0; β=2.5
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function repeller_fabric(x, ẋ, env::PointMass)
    kᵦ = 50; αᵦ = 1 
    s = ẋ[1] < 0 ? 1 : 0
    M = (s*kᵦ) / (x[1]^2)
    ψ(θ) = αᵦ / (2θ^8)
    δx = ForwardDiff.derivative(ψ, x[1])
    ẍ = -s * ẋ[1]^2 * δx
    # @show (M, [ẍ])
    return (M, [ẍ])
end

function fabric_eval(x, ẋ, name::Symbol, env::PointMass)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
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
    return ẍ 
end

# function task_map(θ, env::PointMass)
#     θ₀ = env.o
#     r = env.r
#     ϕ = (norm(θ-θ₀)-r)/r 
#     return ϕ
# end

# function potential(θ, env::PointMass)
#     k = env.k
#     ϕ(q) = task_map(q, env)
#     ψ = k/(ϕ(θ)^2)
#     return ψ
# end

# function geometry_fabric(θ, θ̇ , env::PointMass)
#     λ = env.λ
#     f(q) = potential(q, env)
#     @show f(θ)
#     δ₀ = ForwardDiff.gradient(f, θ)
#     ẍ = -λ * norm(θ̇ )^2 * δ₀
#     return ẍ
# end

# function finsler_fabric(θ, θ̇ , env::PointMass)
#     λ = env.λ
#     ψ(q) = potential(q, env)
#     δ₀ = ForwardDiff.gradient(ψ, θ)
#     ϕ(q) = [task_map(q, env)]
#     J = ForwardDiff.jacobian(ϕ, θ) 
#     Lₑ = (1/(2*ϕ(θ)[1]^2)) * (J * θ̇ ).^2
#     ẍ = -λ * Lₑ .* δ₀
#     return ẍ

# end

# function pointmass_fabric(θ, θ̇ , env::PointMass)
#     ẍ = geometry_fabric(θ, θ̇ , env)
#     return ẍ
# end
