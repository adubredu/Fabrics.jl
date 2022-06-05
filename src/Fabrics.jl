module Fabrics

using GLMakie
using DataStructures: CircularBuffer
using ForwardDiff
using LinearAlgebra
using StaticArrays
using Colors 

include("systems/point_mass/types.jl")
include("systems/point_mass/visualize.jl")
include("systems/point_mass/step.jl")
include("systems/point_mass/fabric.jl")

include("systems/planar_arm/types.jl")
include("systems/planar_arm/visualize.jl")
include("systems/planar_arm/step.jl")
include("systems/planar_arm/fabric.jl")

include("systems/multi_planar_arm/types.jl")
include("systems/multi_planar_arm/visualize.jl")
include("systems/multi_planar_arm/step.jl")
include("systems/multi_planar_arm/fabric.jl")


include("systems/picklerick/types.jl")
include("systems/picklerick/visualize.jl")
include("systems/picklerick/step.jl")
include("systems/picklerick/fabric.jl") 

export visualize_system!,
       step!,
       jvp,
       compute_COM

export PointMass,
       pointmass_fabric_solve,
       move_obstacles!

export PlanarArm,
       link_poses,
       planararm_fabric_solve

export MultiPlanarArm,
       multiplanar_arm_fabric_solve

export PickleRick,
       picklerick_fabric_solve 

end
