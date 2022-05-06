using Revise
using Fabrics

α = 1.5
x = [3.5, -1.0]
ẋ = α * [-1.0, 0]
O = [0.0, 0.0]
r = 3.5

env = PointMass(x, ẋ, O, r)
ax, fig = visualize_system!(env) 

for i=1:5000
    global x, ẋ
    ẍ = pointmass_fabric(x, ẋ, env)
    step!(ẍ, env)
    @show x = env.x
    ẋ = env.ẋ
    sleep(env.Δt)
end
