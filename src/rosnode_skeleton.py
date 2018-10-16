#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from mill_controller.msg import Float32Array
from mill_controller.msg import Trajectory


# Use this publisher to send the translated input
pub = rospy.Publisher('trajectory_input',Trajectory,queue_size=1)



def callback(input):
    # Upon receiving a string message, translate the string to a Trajectory and republish it.
    s = input.data

    # Parse the string into a list of lists.


    # Zero counters.
    i=0
    k=0

    # Increment through the list of lists and the Trajectory message, copying data from former to latter.





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("text_input_parser_node")
    rospy.Subscriber('string_input',String,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
