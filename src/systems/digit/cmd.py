#! /usr/bin/env/python3
import rospy
# from digit_msgs.msg import Digit_Observation, Digit_Commands
import numpy as np

# class Cmd:
#     def __init__(self):
#         rospy.init_node("cmd_node")
#         self.obs = "2"
#         rospy.Subscriber("/observation", Digit_Observation, self.obs_callback)
#         rospy.spin()
    
#     def obs_callback(self, data):
#         self.obs = data 
    
class MyNumpy:
     def __init__(self,n,m):
          self.array = np.zeros((n,m))
          self.size = (n,m)




# def callback(data):
#     print(data)

# def listener():
#     rospy.init_node('listener')
#     rospy.Subscriber('/observation', Digit_Observation, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()