function link_poses(θ, env::MultiPlanarArm)
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
    x1 = l1 * cos(θ[1])
    y1 = l1 * sin(θ[1])
    x2 = x1 + l2 * cos(θ[2])
    y2 = y1 + l2 * sin(θ[2])
    x3 = x2 + l3 * cos(θ[3])
    y3 = y2 + l3 * sin(θ[3])
    x4 = x3 + l4 * cos(θ[4])
    y4 = y3 + l4 * sin(θ[4])
    x5 = x4 + l5 * cos(θ[5])
    y5 = y4 + l5 * sin(θ[5])
    x6 = x5 + l6 * cos(θ[6])
    y6 = y5 + l6 * sin(θ[6])
    x7 = x6 + l7 * cos(θ[7])
    y7 = y6 + l7 * sin(θ[7])
    x8 = x7 + l8 * cos(θ[8])
    y8 = y7 + l8 * sin(θ[8])
    x9 = x8 + l9 * cos(θ[9])
    y9 = y8 + l9 * sin(θ[9])
    x10 = x9 +  l10 * cos(θ[10])
    y10 = y9 + l10 * sin(θ[10])
    return [[x1, y1], [x2, y2], [x3, y3], [x4, y4], [x5, y5], [x6, y6], [x7, y7], [x8, y8], [x9, y9], [x10, y10]]
end

function visualize_system!(env::MultiPlanarArm)
    obstacle_positions = env.o
    obstacle_radii = env.r*2
    goal_position = env.g
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-10., 10, -10.,10.))
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
    p1, p2, p3, p4, p5, p6, p7, p8, p9, p10 = link_poses(θ, env)

    links = Observable([SVector(0.0, 0.0), SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2]), SVector(p4[1], p4[2]), SVector(p5[1], p5[2]), SVector(p6[1], p6[2]), SVector(p7[1], p7[2]), SVector(p8[1], p8[2]), SVector(p9[1], p9[2]), SVector(p10[1], p10[2])])
    
    joints = Observable([SVector(p1[1], p1[2]), SVector(p2[1], p2[2]), SVector(p3[1], p3[2]), SVector(p4[1], p4[2]), SVector(p5[1], p5[2]), SVector(p6[1], p6[2]), SVector(p7[1], p7[2]), SVector(p8[1], p8[2]), SVector(p9[1], p9[2]), SVector(p10[1], p10[2])])

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