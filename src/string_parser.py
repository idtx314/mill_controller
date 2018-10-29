#!/usr/bin/env python

'''
Process an input String message into a list of lists and publish it as a Trajectory message
Ex Input:
'[[0.0,1.0,1.0,1.0],[0.1,1,2,1],[0.2,2,2,1]]'
'''


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
    # Ex: "[[1.0,1.0,1.0,1.0] , [2.0,2.0,2.0,2.0] , ...]"

    # Parse the string into a list of lists.
    # Remove extraneous characters
    output = s.replace('\n','')
    output = s.replace(' ','')
    # Ex: "[[1.0,1.0,1.0,1.0],[2.0,2.0,2.0,2.0],...]"

    # Strip brackets from both ends
    output = output.strip('[]')
    # Ex: "1.0,1.0,1.0,1.0],[2.0,2.0,2.0,2.0],..."

    # Split the values into a list of strings
    output = output.split('],[')
    # Ex: ["1.0,1.0,1.0,1.0" , "2.0,2.0,2.0,2.0" , ... ]

    # Split each string into a list of strings and translate each of those strings into a float
    for index in range(len(output)):
        output[index] = output[index].split(',')
        # Ex [["1.0","1.0","1.0","1.0"] , ["2.0","2.0","2.0","2.0"] , ...]
        for subdex in range(len(output[index])):
            output[index][subdex] = float(output[index][subdex])
            # Ex: [[1.0,1.0,1.0,1.0] , [2.0,2.0,2.0,2.0] , ... ]

    # Create message
    msg = Trajectory()

    # Increment through the list of lists of floats and the arrays of the Trajectory message, copying data from former to latter.
    for index in range(len(output)):
        msg.x.append(output[index][0])
        msg.y.append(output[index][1])
        msg.z.append(output[index][2])
        msg.t.append(output[index][3])
    msg.length = len(output)

    # Publish msg
    pub.publish(msg)






def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("string_parser_node")
    rospy.Subscriber('string_input',String,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
