#! /usr/bin/env/python3
import rospy
from digit_msgs.msg import Digit_Observation, Digit_Commands

def callback(data):
    print(data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('/observation', Digit_Observation, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()