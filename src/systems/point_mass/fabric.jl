function task_map(θ, env::PointMass)
    θ₀ = env.o
    r = env.r
    ϕ = (norm(θ-θ₀)-r)/r 
    return ϕ
end

function potential(θ, env::PointMass)
    k = env.k
    ϕ(q) = task_map(q, env)
    ψ = k/(ϕ(θ)^2)
    return ψ
end

function geometry_fabric(θ, θ̇ , env::PointMass)
    λ = env.λ
    f(q) = potential(q, env)
    δ₀ = ForwardDiff.gradient(f, θ)
    ẍ = -λ * norm(θ̇ )^2 * δ₀
    return ẍ
end

function finsler_fabric(θ, θ̇ , env::PointMass)
    λ = env.λ
    ψ(q) = potential(q, env)
    δ₀ = ForwardDiff.gradient(ψ, θ)
    ϕ(q) = [task_map(q, env)]
    J = ForwardDiff.jacobian(ϕ, θ) 
    Lₑ = (1/(2*ϕ(θ)[1]^2)) * (J * θ̇ ).^2
    ẍ = -λ * Lₑ .* δ₀
    return ẍ

end

function pointmass_fabric(θ, θ̇ , env::PointMass)
    ẍ = finsler_fabric(θ, θ̇ , env)
    return ẍ
end
