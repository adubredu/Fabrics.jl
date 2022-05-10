function attractor_task_map(θ, env::PlanarArm)
    x₉ = env.g
    _,_,x = link_poses(θ, env)
    return x - x₉
end

function repeller_task_map(θ, env::PlanarArm)
    os = env.o
    r = env.r 
    _,_,x = link_poses(θ, env) 
    xs = []
    for o in os 
        Δ =  (norm(x - o)/r)[1] - 1.0
        push!(xs, Δ)
    end
    return xs
end

function joint_lower_limit_task_map(θ, env::PlanarArm)
    return θ-env.lb
end

function joint_upper_limit_task_map(θ, env::PlanarArm)
    return env.ub-θ
end

function attractor_fabric(x, ẋ, env::PlanarArm)
    k = 50.0; αᵩ = 10.0; β=2.5
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function repeller_fabric(x, ẋ, env::PlanarArm)
    kᵦ = 50; αᵦ = 1.0 
    s = zero(ẋ)
    for i=1:length(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm((s*kᵦ) ./ (x.^2))
    ψ(θ) = αᵦ ./ (2θ.^8) 
    x = convert(Vector{Float64}, x)
    δx = ForwardDiff.jacobian(ψ, x) 
    ẍ = -s .* norm(ẋ)^2 .* δx  
    ẍ = vec(ẍ)
    return (M, ẍ)
end

function joint_lower_limit_fabric(x, ẋ, env::PlanarArm)
    λ = 0.25
    α₁ = 0.4; α₂ = 0.2; α₃ = 20; α₄ = 5.0
    s = zero(ẋ)
    for i=1:length(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (α₁./(θ.^2)) .+ α₂*log.(exp.(-α₃*(θ.-α₄)) .+ 1) 
    δx = ForwardDiff.jacobian(ψ, x) 
    ẍ = δx* (-s .* norm(ẋ)^2)
    ẍ = vec(ẍ) 
    return (M, ẍ)
end

function joint_upper_limit_fabric(x, ẋ, env::PlanarArm)
    λ = 0.25
    α₁ = 0.4; α₂ = 0.2; α₃ = 20; α₄ = 5.0
    s = zero(ẋ)
    for i=1:length(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (α₁./(θ.^2)) .+ α₂*log.(exp.(-α₃*(θ.-α₄)) .+ 1) 
    δx = ForwardDiff.jacobian(ψ, x) 
    ẍ = δx* (-s .* norm(ẋ)^2)
    ẍ = vec(ẍ) 
    return (M, ẍ)
end

function fabric_eval(x, ẋ, name::Symbol, env::PlanarArm)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
end

function planararm_fabric_solve(θ, θ̇ , env::PlanarArm)
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