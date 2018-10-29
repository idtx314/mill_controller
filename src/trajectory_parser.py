#!/usr/bin/env python

'''
Accepts Trajectory messages and produces a GCode file based on the included trajectory data. Publishes a bool message as a flag to the next node in the pipeline.
This script presumes that the machine has been properly homed and had the G54 workspace set up beforehand.
'''
# TODO
# Add workspace and machine space safety checks


import rospy
import sys
from std_msgs.msg import Bool
from mill_controller.msg import Trajectory


# Use this publisher to send the output flag
pub = rospy.Publisher('gcode_ready_flag',Bool,queue_size=1)



def callback(message):
    # Upon receiving a Trajectory message, parse the information and write a GCode file based on it.

    # Open output file
    f = open('output.gcode', 'w')

    # Write the Header
    s = ''

    # Basic Settings
    s = s + 'G90' + '\n'                    # Set absolute coordinates
    s = s + 'G21' + '\n'                    # Set mm
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate to mm/min
    s = s + 'G54' + '\n'                    # Use WCS G54 Coordinates

    f.write(s)                              # Write to file



    # Write the Body
    # Set Feed rate and move to starting point.
    s = 'G0' + ' X' + str(message.x[0]) + ' Y' + str(message.y[0]) + '\n'
    s = s + 'G0' + ' Z-15' + '\n'
    s = s + 'G1' + ' Z-20.0762' + ' F50' + '\n'

    f.write(s)

    # Follow trajectory. Operating on the assumption that the trajectory can be approximated as a series of straight line motions from point to point. Improvement of this model will probably need example trajectories to test. Starting with fixed feed rate.
    for index in range(message.length):
        s = 'G1' + ' X' + str(message.x[index]) + ' Y' + str(message.y[index]) + ' F200.0' + '\n'
        # Write appropriate command to output
        f.write(s)



    # Write the Footer
    s = ''
    s = s + 'G90' + '\n'                    # Set to absolute coordinates
    s = s + 'G21' + '\n'                    # Set to mm
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate to mm/min
    s = s + 'G54' + '\n'                    # Use WCS G54 Coordinates

    s = s + 'G1 Z-15 F50' + '\n'       # Move bit out of harms way
    s = s + 'G0 Z0' + '\n'
    s = s + 'G0 X330 Y228' + '\n'

    s = s + 'G4 P0.1' + '\n'                # Dwell for a moment
    f.write(s)                              # Write to file


    # Close output
    f.close()

    # Publish to a flag topic to trigger the next node.
    flag = Bool()
    flag.data = True
    pub.publish(flag)




def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("trajectory_parser_node")
    rospy.Subscriber('trajectory_input',Trajectory,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
