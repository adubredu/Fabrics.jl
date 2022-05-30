using Revise 
using Fabrics 

Kp = 1050.0
Kd = 0.5

θrefs = [0.32869133647921467, -0.02792180592249217, 0.3187324455828634, 0.36118057019763633, -0.14684031092035302, 
0.11311574329868718, -0.32875125760374146, 0.02783743697915846, -0.31868450868324194, -0.3611086648482042, 0.14674060216914045, 
-0.11315409281838432, -0.15050988058637318, 1.0921200187801636, 0.00017832526659170586, 
-0.13909131109654943, 0.15051467427633533, -1.0921631619898227, -0.00017832526659170586, 0.13910089847647372]

θrefs[1] = -0.5
θrefs[7] = -0.5


init_digit_server()
Damping = 2
while true
    obs = get_observation()
    θ = collect(obs.motor_position)
    torques = zeros(length(θrefs))
    velocities = zeros(length(θrefs))
    dampings = zeros(length(θrefs))
    for i=1:20
        torques[i] = Kp * (θrefs[i]-θ[i])
        velocities[i] = 0.0
        dampings[i] = Kd * obs.motor_limit_damping[i]
    end
    send_command(Damping, true, torques, velocities, dampings)
    # sleep(0.001)
end
        

#=
torso pitch limits 
    [0.7, -0.7] l r : backward
torso roll limits 
    same polarities: foot moves instead of torso
=#