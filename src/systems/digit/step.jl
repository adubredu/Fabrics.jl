using PyCall
py"""
import time 
import sys 
sys.path.append("src/systems/digit/sim/ws/devel/lib/python2.7/dist-packages")
import rospy
from digit_msgs.msg import Digit_Observation, Digit_Commands
from digit_msgs.srv import *
import json
from ws4py.client.threadedclient import WebSocketClient

class BasicClient(WebSocketClient):
    def opened(self):
        self.operation_mode = None
        self.responded = True
        privilege_request = ['request-privilege', 
                                {'privilege' : 'change-action-command',
                                 'priority' : 0}]
        self.send(json.dumps(privilege_request))

    def received_message(self, m):
        dataloaded = json.loads(m.data)
        message_type = str(dataloaded[0])
        message_dict = dataloaded[1]

        if message_type == 'privileges':
            self.done = message_dict['privileges'][0]['has']
            if self.done:
                print("Privilege request executed successfully.")

        elif message_type == 'robot-status':
            self.responded = True
            self.operation_mode = str(message_dict['operation-mode'])

        elif message_type == 'error':
            self.error_info = str(message_dict['info'])
            print('Error: ', self.error_info)

        elif message_type == 'action-status-changed':
            if message_dict['status'] == 'running':
                self.completed = False

            elif message_dict['status'] == 'success':
                self.completed = True
        elif message_type == 'object-kinematics':  
            self.arm_pose = message_dict['transform']['rpyxyz'] 

        elif message_type == 'object':
            self.obstacle_id = message_dict['id']
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
 
# function step!(env::Digit)
#     Dojo.set_robot(env.vis, env.mechanism, Dojo.get_maximal_state(env.mechanism))
# end

# function step!(θ̈ ::Vector{Float64}, env::Digit)
#     observation = get_observation()
#     mirror_joint_configurations!(observation, env)
#     step!(env)


# end

function connect()
    py"""
    def connect():
        ws = BasicClient('ws://127.0.0.1:8080', protocols=['json-v1-agility'])
        while True:
            try:
                ws.connect()
                print("WS connection established")
                time.sleep(1)
                break
            except:
                print('WS connection NOT established')
                time.sleep(1) 
        return ws
    """
    ws = py"connect"()
    return ws
end

function move_obstacle(ws, current_pose; delta=0.001, axis=1, dir=-1)
    pose = current_pose 
    pose[axis]+= dir*delta 
    py"""
    def move_obstacle(ws, pose):
        p = [i for i in pose]
        msg = [
            "set-object-kinematics",
            {
                "object": {"object-id": $ws.obstacle_id},
                "transform": {"xyz":p},
                "velocity": None,
                "relative-to": {
                "special-frame": "world"
                },
                "in-coordinates-of": {},
                "stored-relative-to": {}
            }
        ]
        ws.send(json.dumps(msg))
        return pose
    """
    pose = py"move_obstacle"(ws, pose)
    return pose
end

function create_obstacle(ws)
    py"""
    def create_obstacle(ws):
        msg = [
            "add-object",
            {
                "attributes": {"name":"obstacle",
                            "mass":0.1,
                            "box-geometry":[0.05, 0.05, 0.05]},
                "transform": {"xyz":[1.0, 0.0, 1.0]},
                "velocity": None,
                "relative-to": {
                "special-frame": "world"
                },
                "in-coordinates-of": {},
                "stored-relative-to": {}
            }
        ]
        ws.send(json.dumps(msg)) 
        time.sleep(0.1)
    """
    py"create_obstacle"(ws)
    sleep(0.1)
end


# init_digit_server()
# for i=1:10
#     send_command(i, true, ones(20), ones(20), ones(20))
#     sleep(1.0/300.0)
# end

ws = connect()
create_obstacle(ws)
current_pose = [1.0, 0.15, 1.4]
for i=1:5000
    move_obstacle(ws, current_pose)
    sleep(0.001)
end
println("Done")

