using Revise
using Fabrics

object_positions = [[2.0, 14]]
goal_position = [-2.0, 10.0]
init_joint_positions = [π/6,2π/3,π/2]
obstacle_radii = 1.875 * ones(length(object_positions))

env = PlanarArm(init_joint_positions, zeros(length(init_joint_positions)), object_positions, obstacle_radii, goal_position)
ax, fig = visualize_system!(env)
env.dynamic = true

θ = init_joint_positions
θ̇ = zero(θ)

for i=1:20000
    global θ, θ̇
    ẍ = planararm_fabric_solve(θ, θ̇ , env)
    step!(ẍ, env)
    θ = env.θ
    θ̇ = env.θ̇
    sleep(0.005*env.Δt)
end