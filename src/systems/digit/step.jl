# using PyCall 
# rospy = pyimport("rospy")
# Cmd = pyimport("cmd")

# obs = Cmd.MyNumpy()
# for i=1:10
#     @show keys(obs)
#     sleep(0.5)
# # end
# py"""
# import rospy
# import sys 
# sys.path.append("/home/alphonsus/research/projects/Fabrics/src/systems/digit/sim/ws/devel/lib/python2.7/dist-packages")
# from digit_msgs.msg import Digit_Observation, Digit_Commands 

# class Cmd:
#     def __init__(self):
#         rospy.init_node("cmd_node")
#         self.obs = None
#         rospy.Subscriber("/observation", Digit_Observation, self.obs_callback)
#         rospy.spin()
    
#     def obs_callback(self, data):
#         self.obs = data
#         print(data.time)
# """

# Cmd = py"Cmd"
# cmd = Cmd()
# # sleep(2)
# for i=1:1000
#     try
#     @show cmd.obs
#     sleep(0.5)
#     catch 
#     end
# end

using RobotOS 
@rosimport digit_msgs.msg: Digit_Observation, Digit_Commands 
rostypegen()
using .digit_msgs.msg 

env = Digit_Observation[]

function callback(msg::Digit_Observation, env)
    push!(env, msg)
    println("got it")
end
callback(Digit_Observation(), env)
const sub = Subscriber("/observation", Digit_Observation, callback, (env,), queue_size=10)
rossleep(Duration(3.0))

for i=1:10
    @show env[1].joint_position
    @show length(env)
    sleep(0.5)
end