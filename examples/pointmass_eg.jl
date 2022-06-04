using Revise
using Fabrics

x = [5.5, 0.0]
ẋ = [0.0, 0.0]
O = [[-2.0, 5.0], [2.5, -7.5], [-2.0, 9.5], [2.5, 7.5]]
# O = [[-2.0, 0.0], [2.5, -2.5], [-2.0, 4.5], [2.5, 2.5]]
g = [-5.0, 0.0]
or = 1.75*ones(length(O))
r = 0.5

env = PointMass(x, ẋ, r, O, or, g)
env.show_tail = true
env.dynamic = true
env.obstacle_speed = 0.017 #17 mm/sec
ax, fig = visualize_system!(env)
horizon = 10000

for i=1:horizon
    global x, ẋ
    if env.dynamic move_obstacles!(env) end
    ẍ = pointmass_fabric_solve(x, ẋ, env)
    step!(ẍ, env)
    x = env.x
    ẋ = env.ẋ
    sleep(env.Δt/horizon)
end
