module Fabrics

using GLMakie
using DataStructures: CircularBuffer
using ForwardDiff
using LinearAlgebra
using StaticArrays
using Colors
using PyCall
using Dojo 
# using Pkg
ENV["PYTHON"]="/home/alphonsus/anaconda3/envs/digit/bin/python"
# Pkg.build("PyCall")

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

# include("systems/digit/types.jl")
# include("systems/digit/visualize.jl")
# include("systems/digit/step.jl")
# include("systems/digit/fabric.jl")
# include("systems/digit/visualization/initialize.jl")

export visualize_system!,
       step!,
       jvp,
       compute_COM

export PointMass,
       pointmass_fabric_solve

export PlanarArm,
       link_poses,
       planararm_fabric_solve

export PickleRick,
       picklerick_fabric_solve

# export Digit, 
#        digit_fabric_solve,
#        init_digit_server,
#        get_observation,
#        send_command,
#        mirror_joint_configurations!,
#        get_digit,
#        initialize_digit!

end
