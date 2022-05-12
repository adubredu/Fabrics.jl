module Fabrics

using GLMakie
using DataStructures: CircularBuffer
using ForwardDiff
using LinearAlgebra
using StaticArrays
using Colors
using PyCall

include("systems/point_mass/types.jl")
include("systems/point_mass/visualize.jl")
include("systems/point_mass/step.jl")
include("systems/point_mass/fabric.jl")

include("systems/planar_arm/types.jl")
include("systems/planar_arm/visualize.jl")
include("systems/planar_arm/step.jl")
include("systems/planar_arm/fabric.jl")

include("systems/picklerick/types.jl")
include("systems/picklerick/visualize.jl")
include("systems/picklerick/step.jl")
include("systems/picklerick/fabric.jl")

include("systems/digit/types.jl")
include("systems/digit/step.jl")
inlude("systems/digit/fabric.jl")

export visualize_system!,
       step!,
       jvp

export PointMass,
       pointmass_fabric_solve

export PlanarArm,
       link_poses,
       planararm_fabric_solve

export PickleRick,
       picklerick_fabric_solve

export Digit, 
       digit_fabric_solve
end
