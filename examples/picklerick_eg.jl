using Revise 
using Fabrics  

object_positions = [[-4.0, 3.0]]
goal_position = [-2.0, 2.5]
init_joint_positions = [π/2, 2π/3, π/6, 2π/3, π/6, π/2, π/3, 7π/12, 2π/3, 5π/12]
obstacle_radii = 1.0 * ones(length(object_positions))

env = PickleRick(object_positions,
                0.5 .* obstacle_radii,
              init_joint_positions,
              zero(init_joint_positions),
              goal_position)
env.trail = false
env.show_contacts = false 
ax, fig = visualize_system!(env)
# env.dynamic = false

# θ = init_joint_positions
# θ̇ = zero(init_joint_positions)

# for i=1:20000
#     global θ, θ̇ 
#     a = arm2d_rmp_solve(θ, θ̇ , env) 
#     arm2d_step!(a, env)
#     θ = env.joint_positions 
#     θ̇ = env.joint_velocities
#     sleep(env.time_step*0.001)
# end