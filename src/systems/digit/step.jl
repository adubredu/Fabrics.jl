py"""
import time 
import rospy
import sys 
sys.path.append("src/systems/digit/sim/ws/devel/lib/python2.7/dist-packages")
from digit_msgs.msg import Digit_Observation, Digit_Commands
from digit_msgs.srv import *
"""

function init_digit_server()
    py"""
    rospy.init_node("comms")
    """
end

function get_observation()
    py"""
    def get_observation():
        try:
            channel =  rospy.ServiceProxy("/observation_service", Digit_Observation_srv)
            send_request = Digit_Observation_srvRequest()
            send_request.req.data = True 

            response = channel(send_request)
            if response.status:
                return response.obs
            else:
                return None
        except rospy.ServiceException as e:
            print(e)
            return None
    """
    obs = py"get_observation"()
    return obs
end

function send_command(fallback_opmode::Int64, apply_command::Bool,
        motor_torque::Vector{Float64}, motor_velocity::Vector{Float64},
        motor_damping::Vector{Float64})
    py"""
    try:
        channel = rospy.ServiceProxy("/command_service", Digit_Commands_srv)
        send_request = Digit_Commands_srvRequest()
        send_request.cmd.fallback_opmode=$fallback_opmode
        send_request.cmd.apply_command = $apply_command
        send_request.cmd.motor_torque = $motor_torque
        send_request.cmd.motor_velocity = $motor_velocity
        send_request.cmd.motor_damping = $motor_damping
        response = channel(send_request) 
    except rospy.ServiceException as e:
        print(e)

    """
end
 
function step!(θ̈ ::Vector{Float64}, env::Digit)

end




# init_digit_server()
# for i=1:10
#     send_command(i, true, ones(20), ones(20), ones(20))
#     sleep(1.0/300.0)
# end