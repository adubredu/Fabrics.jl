using Revise
using Fabrics

object_positions = [[4.0, 2.0]]
goal_position = [-2.0, 2.5]
init_joint_positions = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
obstacle_radii = 3.0 * ones(length(object_positions))

env = MultiPlanarArm(init_joint_positions, 
        zero(init_joint_positions), 
        object_positions, 
        0.5*obstacle_radii, 
        goal_position)
ax, fig = visualize_system!(env)
env.dynamic = false

θ = init_joint_positions
θ̇ = zero(θ)
horizon = 20000

for i=1:horizon
    global θ, θ̇
    ẍ = multiplanar_arm_fabric_solve(θ, θ̇ , env)
    step!(ẍ, env)
    θ = env.θ
    θ̇ = env.θ̇
    sleep(env.Δt/horizon)
end