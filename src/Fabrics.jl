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

export visualize_system!,
       step!

export PointMass,
       pointmass_fabric

end
