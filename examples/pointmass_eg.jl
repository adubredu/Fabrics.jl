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
ax, fig = visualize_system!(env)


for i=1:10000
    global x, ẋ
    if env.dynamic move_obstacles!(env) end
    ẍ = pointmass_fabric_solve(x, ẋ, env)
    step!(ẍ, env)
    x = env.x
    ẋ = env.ẋ
    sleep(0.0001*env.Δt)
end
