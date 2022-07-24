function repeller_task_map(θ, env::PickleRick)
    os = env.obstacle_observables
    rs = env.r
    chains = link_poses(θ, env)
    xs = []; o = os[1]; r = 2*rs[1]
    lh = chains[5][end]; rh = chains[4][end]
    lf = chains[1][end]; rf = chains[2][end]
    hd = chains[3][end]; nk = chains[6][1]
    kp = [lh, rh, hd, nk]#[hd, lh, rh, lf, rf]
    for k in kp 
        Δ = (norm(k - o.val)/r)[1] - 1.0
        push!(xs, Δ)
    end
    return xs
end

function default_config_task_map(θ, env::PickleRick)
    return θ - env.θᵣ
end

function lefthand_attractor_task_map(θ, env::PickleRick)
    x₉ = env.g
    chains = link_poses(θ, env)
    xh = chains[5][end]
    return xh - x₉
end

function righthand_attractor_task_map(θ, env::PickleRick)
    x₉ = env.g
    chains = link_poses(θ, env)
    xh = chains[4][end]
    return xh - x₉
end

function lbalance_task_map(θ, env::PickleRick)
    com = compute_COM(θ, env)
    return [com[1] - (-env.w)]
end

function rbalance_task_map(θ, env::PickleRick)
    com = compute_COM(θ, env)
    return [env.w - com[1]]
end

function repeller_fabric(x, ẋ, env::PickleRick)
    kᵦ = 50; αᵦ = 1.0 
    s = zero(ẋ)
    for i=1:length(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm((s*kᵦ) ./ (x.^2))
    ψ(θ) = αᵦ ./ (2θ.^8) 
    x = convert(Vector{Float64}, x)
    # @show size(x)
    δx = ForwardDiff.jacobian(ψ, x) 
    # @show (δx)
    δx = diag(δx)
    ẍ = -s .* norm(ẋ)^2 .* δx  
    ẍ = vec(ẍ)
    # @show size(ẍ)
    return (M, ẍ)
end

#dance: 1000 2.5
#stable dodge: 1500 20.5
#reach: 300 5.5 | 30 50.5
function default_config_fabric(x, ẋ, env::PickleRick) 
    λᵪ = 0.25; k = 1500.0; αᵩ = 10.0; β=20.5
    M = λᵪ * I(length(x))
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -δx - β*ẋ
    return (M, ẍ)
end

function lefthand_attractor_fabric(x, ẋ, env::PickleRick)
    k = 50.0; αᵩ = 10.0; β=2.5
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function righthand_attractor_fabric(x, ẋ, env::PickleRick)
    k = 50.0; αᵩ = 10.0; β=2.5
    m₊ = 2.0; m₋ = 0.2; αₘ = 0.75
    ψ(θ) = k * (norm(θ) + (1/αᵩ)*log(1+exp(-2αᵩ*norm(θ))))
    δx = ForwardDiff.gradient(ψ, x)
    ẍ = -δx - β*ẋ
    M = (m₊ - m₋) * exp(-(αₘ*norm(x))^2) * I(2) + m₋*I(2)
    return (M, ẍ)
end

function lbalance_fabric(x, ẋ, env::PickleRick)
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

function rbalance_fabric(x, ẋ, env::PickleRick)
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

function fabric_eval(x, ẋ, name::Symbol, env::PickleRick)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
end

function energize(ẍ, M, env::PickleRick; ϵ=1e-1)
    ẋ = env.θ̇  + ẍ*env.Δt 
    ẋ = ẋ/(norm(ẋ)) 
    ẍₑ = (I(size(M)[1]) - ϵ*ẋ*ẋ')*ẍ
    return ẍₑ
end

function picklerick_fabric_solve(θ, θ̇ , env::PickleRick)
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