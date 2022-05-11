function repeller_task_map(θ, env::PickleRick)
    os = env.obstacle_observables
    rs = env.r
    chains = link_poses(θ, env)
    xs = []; o = os[1]; r = rs[1]
    lh = chains[5][end]; rh = chains[4][end]
    lf = chains[1][end]; rf = chains[2][end]
    hd = chains[3][end]
    kp = [hd, lh, rh, lf, rf]
    for k in kp 
        Δ = (norm(k - o.val)/r)[1] - 1.0
        push!(xs, Δ)
    end
    return xs
end

function repeller_fabric(x, ẋ, env::PickleRick)
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

function fabric_eval(x, ẋ, name::Symbol, env::PickleRick)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, env)
    return (M, ẍ)
end

function planararm_fabric_solve(θ, θ̇ , env::PickleRick)
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