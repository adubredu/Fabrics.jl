function link_poses(θ, env::PlanarArm)
    l1 = env.l1
    l2 = env.l2
    l3 = env.l3
    x1 = l1 * cos(θ[1])
    y1 = l1 * sin(θ[1])
    x2 = x1 +  l2 * cos(θ[2])
    y2 = y1 + l2 * sin(θ[2])
    x3 = x2 +  l3 * cos(θ[3])
    y3 = y2 + l3 * sin(θ[3])
    return [[x1, y1], [x2, y2], [x3, y3]]
end

function visualize_system!(env::PlanarArm)
    obstacle_positions = env.o
    obstacle_radii = env.r*2
    goal_position = env.g
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-10., 10, -1.,15.))
    obstacle_observables = []
    for (i, ob) in enumerate(obstacle_positions)  
        ox =  Observable(SVector{2,Float64}(ob...))
        scatter!(ax, ox; marker=:circle, markersize=obstacle_radii[i], markerspace=SceneSpace, color=:black) 
        push!(obstacle_observables, ox)
        if env.show_contacts
            keypoints = get_keypoints(collect(ox.val), 0.5*obstacle_radii[i],N=8) 
            scatter!(ax, [k[1] for k in keypoints], [k[2] for k in keypoints]; marker=:circle, markerspace=SceneSpace, markersize=0.25, color=:red) 
        end
    end 
    θ = env.θ
    p1, p2, p3 = link_poses(θ, env)
    links = Observable([SVector(0.0, 0.0), SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2])])

    joints = Observable([SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2])])

    lines!(ax, links; linewidth=5, color=:purple)
    scatter!(ax, joints; marker=:circle, color=:black, markersize=0.3, markerspace=SceneSpace)
    scatter!(ax, [0.0], [0.0]; marker=:rect, markersize=0.5, color=:black, markerspace=SceneSpace)
   
    og = SVector{2,Float64}(goal_position...)  
    scatter!(ax, og; marker=:rect, markersize=0.5, markerspace=SceneSpace, color=:green)

    env.link_observables = links 
    env.joint_observables = joints
    env.obstacle_observables = obstacle_observables 
    hidedecorations!(ax)
    display(fig)
    return ax, fig
end