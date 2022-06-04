function visualize_system!(env::PointMass) 
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-10.,10.,-10.,10.))
    
    obstacle_observables = []
    for (i, ob) in enumerate(env.o)
        ox = Observable(SVector{2, Float64}(ob...))
        scatter!(ax, ox; marker=:circle, markersize=env.or[i], color=:black, markerspace=SceneSpace)
        push!(obstacle_observables, ox)
    end

    scatter!(ax, [env.g[1]], [env.g[2]]; marker=:rect, markersize=1.0, color=:green, markerspace=SceneSpace)

    robot_position_observable = Observable(SVector{2, Float64}(env.x...))
    scatter!(ax, robot_position_observable; marker=:circle, markersize=env.r, color=:blue, markerspace=SceneSpace)

    tail = nothing
    if env.show_tail
        tail = CircularBuffer{SVector{2, Float64}}(10000)
        fill!(tail, SVector{2, Float64}(env.x...))
        tail = Observable(tail)
        c = to_color(:purple)
        tailcol = [RGBA(c.r, c.g, c.b, (i/300)^2) for i in 1:10000]
        lines!(ax, tail; linewidth = 3, color = tailcol)
    end
    
    env.obs_x = robot_position_observable
    env.obs_tail = tail
    env.obs_o = obstacle_observables

    hidedecorations!(ax)
    display(fig)
    
    return ax, fig
end