# foot as reference
function link_poses(θ::Vector{Float64}, env::PickleRick)
    l1 = env.l1
    l2 = env.l2
    l3 = env.l3
    l4 = env.l4
    l5 = env.l5
    l6 = env.l6
    l7 = env.l7
    l8 = env.l8
    l9 = env.l9
    l10 = env.l10
    l0 = env.l0
    w = env.w 
    x8 = -w
    y8 = 0.0
    x10 = w 
    y10 = 0.0
    x7 = x8 + l8 * cos(θ[8])
    y7 = y8 + l8 * sin(θ[8])
    x9 = x10 + l10 * cos(θ[10])
    y9 = y10 + l10 * sin(θ[10]) 
    x6 = 0.5*(x7 + l7 * cos(θ[7]) + x9 + l9 * cos(θ[9]))
    y6 = 0.5*(y7 + l7 * sin(θ[7]) + y9 + l9 * sin(θ[9]))
    x1 = x6 + l1 * cos(θ[6])
    y1 = y6 + l1 * sin(θ[6])
    x2 = x1 - l2 * sin(θ[2])
    y2 = y1 + l2 * cos(θ[2])
    x3 = x2 - l3 * sin(θ[3])
    y3 = y2 + l3 * cos(θ[3])
    x4 = x1 + l4 * sin(θ[4])
    y4 = y1 + l4 * cos(θ[4])
    x5 = x4 + l5 * sin(θ[5])
    y5 = y4 + l5 * cos(θ[5])
    x0 = x1 + l0 * cos(θ[1])
    y0 = y1 + l0 * sin(θ[1])

    return [[[x6, y6], [x7, y7], [x8, y8]], [[x6, y6], [x9, y9], [x10, y10]], 
    [[x6, y6], [x1, y1], [x0, y0]], [[x1, y1], [x4, y4], [x5, y5]], [[x1, y1], [x2, y2], [x3, y3]]]
end


function visualize_system!(env::PickleRick)
    object_positions = env.o 
    obstacle_radii = env.r*2
    goal_position = env.g
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-5.,5.,-1.,5.)) 
    obstacle_observables = []
    for (i, ob) in enumerate(object_positions)  
        ox =  Observable(SVector{2,Float64}(ob...))
        scatter!(ax, ox; marker=:circle, markersize=obstacle_radii[i], markerspace=SceneSpace, color=:black) 
        push!(obstacle_observables, ox)
        if env.show_contacts
            keypoints = get_keypoints(collect(ox.val), 0.5*obstacle_radii[i],N=8) 
            scatter!(ax, [k[1] for k in keypoints], [k[2] for k in keypoints]; marker=:circle, markerspace=SceneSpace, markersize=0.25, color=:red) 
        end
    end 

    θ = env.θ 
    chains = link_poses(θ, env)
    chain1 = Observable([SVector(a[1], a[2]) for a in chains[1]])
    chain2 = Observable([SVector(a[1], a[2]) for a in chains[2]])
    chain3 = Observable([SVector(a[1], a[2]) for a in chains[3]])
    chain4 = Observable([SVector(a[1], a[2]) for a in chains[4]])
    chain5 = Observable([SVector(a[1], a[2]) for a in chains[5]])
    head = Observable([SVector(chains[3][3][1], chains[3][3][2])])

    lines!(ax, chain1; linewidth=5, color=:purple)
    lines!(ax, chain2; linewidth=5, color=:purple)
    lines!(ax, chain3; linewidth=5, color=:purple)
    lines!(ax, chain4; linewidth=5, color=:purple)
    lines!(ax, chain5; linewidth=5, color=:purple)

    scatter!(ax, chain1; marker=:circle, color=:black, markersize=0.2, markerspace=SceneSpace)
    scatter!(ax, chain2; marker=:circle, color=:black, markersize=0.2, markerspace=SceneSpace)
    scatter!(ax, chain3; marker=:circle, color=:black, markersize=0.2, markerspace=SceneSpace)
    scatter!(ax, chain4; marker=:circle, color=:black, markersize=0.2, markerspace=SceneSpace)
    scatter!(ax, chain5; marker=:circle, color=:black, markersize=0.2, markerspace=SceneSpace)
    scatter!(ax, head; marker=:circle, color=:black, markersize=0.6, markerspace=SceneSpace)
    og = SVector{2,Float64}(goal_position...)  
    scatter!(ax, og; marker=:rect, markersize=0.5, markerspace=SceneSpace, color=:green)

    env.body_observables = [chain1, chain2, chain3, chain4, chain5, head]
    env.obstacle_observables = obstacle_observables 
    hidedecorations!(ax)
    display(fig)
    return ax, fig
end