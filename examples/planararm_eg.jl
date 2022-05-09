using Revise
using Fabrics

object_positions = [[4.0, 2.0]]
goal_position = [-2.0, 2.5]
init_joint_positions = [π/6,2π/3,π/2]
obstacle_radii = 1.5 * ones(length(object_positions))

env = PlanarArm(init_joint_positions, zeros(length(init_joint_positions)), object_positions, obstacle_radii, goal_position)

ax, fig = visualize_system!(env)