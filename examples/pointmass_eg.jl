using Revise
using Fabrics

x = [3.5, 0.0]
ẋ = [0.0, 0.0]
O = [0.0, 0.0]
g = [-5.0, 0.0]
r = 3.5

env = PointMass(x, ẋ, O, r, g)
ax, fig = visualize_system!(env) 

for i=1:5000
    global x, ẋ
    ẍ = pointmass_fabric_solve(x, ẋ, env)
    step!(ẍ, env)
    x = env.x
    ẋ = env.ẋ
    sleep(env.Δt)
end
