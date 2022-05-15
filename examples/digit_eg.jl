using Revise 
using Fabrics  
using MeshCat
using Dojo

vis = Visualizer()
mech = get_digit()
initialize_digit!(mech)

env = Digit()
env.vis = vis
env.mechanism = mech 

anim = MeshCat.Animation()
Dojo.build_robot(env.mechanism; vis=env.vis)

for i=1:100
    Fabrics.step!(zeros(2), env)
    MeshCat.atframe(anim, i) do 
        Dojo.set_robot(env.vis, env.mechanism, Dojo.get_maximal_state(env.mechanism))
    end
    @show i
end
MeshCat.setanimation!(env.vis, anim)