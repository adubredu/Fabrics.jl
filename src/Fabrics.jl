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

export visualize_system!,
       step!,
       jvp

export PointMass,
       pointmass_fabric_solve

export PlanarArm

end
