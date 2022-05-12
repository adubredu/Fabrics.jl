#! /usr/bin/env/python3
import rospy
import sys 
import time
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

if __name__ == "__main__":
     init_server()
     for i in range(10):
          get_observation()
          time.sleep(1)


