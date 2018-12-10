#!/usr/bin/env python

"""
This script contains a ROS node that subscribes to a string input topic. When a string is received, the node will attempt to parse it into an absolute path containing a .csv file. It will attempt to open a file at that location and parse the contents into a Trajectory message. The message will be published to the topic /trajectory_input.
"""
'''
TODO
Include error checking at path parsing for invalid paths
include error checking at file open for missing files
Add path parameter to launch file with default value 'null'
Read this parameter when launched. If not null, initiate the callback with that string, it has been remapped during launch
if run with an argument, attempt to parse that argument into an absolute path. Return error if unsuccessful, run with that path if successful. It has been typed in during run and should override parameter
'''

import rospy
import sys
from std_msgs.msg import String
from mill_controller.msg import Trajectory




def callback(msg):
    # This function receives a string message as an argument. It attempts to interpret that string as an absolute file path of a csv file, opens the file, and parses the contained data into a Trajectory message. The Trajectory message is then published to /trajectory_input

    print type(msg)
    print msg



def main(args):
    # This function accepts arguments from the command line. If any are received it creates a string message with the argument as data and activates callback, passing the message. If none are received, it attempts to read a parameter. If the parameter is read and is not 'null', it creates a string message with the parameter as data and activates callback, passing the message. It then proceeds to spin.

    # Initialize the ROS node
    rospy.init_node("csv_parser_node")

    # If an argument was passed, create a message and pass it to callback()
    if len(args) > 1:
        msg = String()
        msg.data = args[1]
        callback(msg)
    else:
        # If a parameter was set, create a message and pass it to callback()
        param = rospy.get_param("csv_absolute_path", "null")
        if param != "null":
            msg = String()
            msg.data = param
            callback(msg)


    # Initialize the subscriber
    rospy.Subscriber('csv_absolute_path_topic', String, callback)

    # Spin until shut down
    rospy.spin()




'''
Send the whole trajectory
or
Send the part of the trajectory from after the timestamp

The pipe can take trajectories from multiple sources. How am I going to implement receding horizon control?
    I want to bake receding horizon control into the pipe itself. It will accept a trajectory and then follow that trajectory until it reaches the correct timestamp. That cuttoff will have to exist in trajectory_parser, which will stop writing lines based on t > cutoff. cutoff 0 = open loop
    In normal mode node will receive a flag, read a trajectory from csv, create a Trajectory message, and publish it to trajectory parser. The csv will be updated before the next flag is sent?
    In fake mode, the node will receive a flag, read a trajectory from csv, create a trajectory message with each timestamp reduced by horizon*number of times the pipe has published a feedback message. If the stamp of a point is below zero it is excluded from the message. Each time a feedback message is published, the counter will be incremented and a new trajectory message will be made
        A separate node will subscribe to pipe output and publish on this node's flag channel. When launched it will read a .csv file, modify the timestamps by counter*horizon, and write the default input csv for this node. timestamps less than zero excluded, if no timestamps remain then end.


    If launched in quick launch mode the node will not wait for a flag before starting the first trajectory message. Should take a file name as a parameter. Name will have to be a null signal by default. If the name is null or does not exist, run in normal mode and use default file name






trajectory is sent
trajectory is followed until timestamp
feedback runs
new trajectory is sent based on result. Timestamp starts over?
trajectory is followed until timestamp
feedback runs

Where is Ahalya using this array?
Is there any standard format for an array there?
What about a .csv file instead of a message and helper class?
Visual demonstration, maybe the Rviz output
'''


if __name__ == '__main__':
    main(sys.argv)
