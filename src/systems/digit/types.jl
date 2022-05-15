mutable struct Digit  
    o::Vector{Vector{Float64}}
    r::Vector{Float64}
    dynamic::Bool 
    g::Vector{Float64}
    task_maps::Vector{Symbol}
    Δt::Float64 
    vis 
    mechanism
    motorID::Dict{Symbol, Int64}
    jointID::Dict{Symbol, Int64}

    function Digit()
        g = zeros(2)
        o = [zeros(2)]
        r = zeros(2)
        Δt = 10E-4
        task_maps = [:default_config]
        motorID = Dict(
            :LeftHipRoll => 1,
            :LeftHipYaw => 2,
            :LeftHipPitch => 3,
            :LeftKnee => 4,

            :RightHipRoll => 7,
            :RightHipYaw => 8,
            :RightHipPitch => 9,
            :RightKnee => 10,

            :LeftShoulderRoll => 13,
            :LeftShoulderPitch => 14,
            :LeftShoulderYaw => 15,
            :LeftElbow => 16,

            :RightShoulderRoll => 17,
            :RightShoulderPitch => 18,
            :RightShoulderYaw => 19,
            :RightElbow => 20
        )

        jointID = Dict(
            :LeftShin => 1, 
            :LeftTarsus => 2,
            :LeftToePitch => 3,
            :LeftToeRoll => 4,

            :RightShin => 6,
            :RightTarsus => 7,
            :RightToePitch => 8,
            :RightToeRoll => 9
        )
        new(o, r, false, g, task_maps, Δt, nothing, nothing, motorID, 
        jointID)
    end
end