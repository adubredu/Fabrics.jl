using Revise 
using Fabrics  

object_positions = [[-4.0, 3.5]]
# object_positions = [[-4.0, 4.0]]
goal_position = [-2.0, 4.5]
init_joint_positions = zeros(10)#[π/2, 2π/3, π/6, 2π/3, π/6, π/2, π/3, 7π/12, 2π/3, 5π/12]
obstacle_radii = 1.0 * ones(length(object_positions))

env = PickleRick(object_positions,
                0.5 .* obstacle_radii,
              init_joint_positions,
              zero(init_joint_positions),
              goal_position)
env.trail = false
env.show_contacts = false 
ax, fig = visualize_system!(env)
env.dynamic = true

θ = init_joint_positions
θ̇ = zero(init_joint_positions)

for i=1:30000
    global θ, θ̇ 
    ẍ = picklerick_fabric_solve(θ, θ̇ , env) 
    step!(ẍ, env)
    θ = env.θ 
    θ̇ = env.θ̇
    sleep(env.Δt)
end