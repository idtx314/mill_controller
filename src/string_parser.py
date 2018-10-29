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

    # Increment through the list of lists of floats and the Trajectory message, copying data from former to latter.
    for list in output:
        # Create Float32Array()
        arr = Float32Array()
        # Fill Float32Array with floats from list[0] to list[3]
        for i in range(4):
            arr.point[i] = list[i]
        # Append Float32Array to msg.trajectory
        msg.trajectory.append(arr)


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
