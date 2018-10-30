#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from mill_controller.msg import Trajectory


# Use this publisher to send the output
pub = rospy.Publisher('trajectory_input',Trajectory,queue_size=1)



def callback(input):
    # What does the callback do?





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("text_input_parser_node")
    rospy.Subscriber('string_input',String,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
