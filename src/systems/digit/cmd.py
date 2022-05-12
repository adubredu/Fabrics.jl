#! /usr/bin/env/python3
import rospy
import sys 
import time
import numpy as np
sys.path.append("/home/alphonsus/research/projects/Fabrics/src/systems/digit/sim/ws/devel/lib/python2.7/dist-packages")
from digit_msgs.msg import Digit_Observation, Digit_Commands
from digit_msgs.srv import *

def init_server():
     rospy.init_node("comms")

def get_observation():
     try:
          channel =  rospy.ServiceProxy("/observation_service", Digit_Observation_srv)
          send_request = Digit_Observation_srvRequest()
          send_request.req.data = True 

          response = channel(send_request)
          if response.status:
               print("received observation")
               print(response.obs.time)
          else:
               print("failed to receive observation")

     except rospy.ServiceException as e:
          print(e)
          sys.exit()

def send_command():
     try:
          channel = rospy.ServiceProxy("/command_service", Digit_Commands_srv)
          send_request = Digit_Commands_srvRequest()
          send_request.cmd.fallback_opmode=1
          send_request.cmd.apply_command = True
          send_request.cmd.motor_torque = np.random.randn(20)
          send_request.cmd.motor_velocity = np.random.randn(20)
          send_request.cmd.motor_damping = np.random.randn(20)
          response = channel(send_request)
          if response.status.data:
               print("sent command")
          else:
               print("failed to send command")
     except rospy.ServiceException as e:
          print(e)
          sys.exit()

if __name__ == "__main__":
     init_server()
     for i in range(10):
          send_command()
          time.sleep(1.0)


