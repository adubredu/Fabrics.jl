using Revise
using Fabrics

object_positions = [[-1.0, 2.0]]
goal_position = [-2.0, 2.5]
init_joint_positions = [0.0001, 0.0001]
obstacle_radii = 1.0 * ones(length(object_positions))

env = PlanarArm(init_joint_positions, 
        zeros(length(init_joint_positions)), 
        object_positions, 
        0.5*obstacle_radii, 
        goal_position)
ax, fig = visualize_system!(env)
env.dynamic = false

θ = init_joint_positions
θ̇ = zero(θ)
horizon = 5000

for i=1:horizon
    global θ, θ̇
    ẍ = planararm_fabric_solve(θ, θ̇ , env)
    step!(ẍ, env)
    θ = env.θ
    θ̇ = env.θ̇
    sleep(env.Δt/horizon)
end