#!/usr/bin/env python

'''
Accepts Trajectory messages and produces a GCode file based on the included trajectory data. Publishes a bool message as a flag to the next node in the pipeline.
'''
# TODO
# Remove Z axis parsing


import rospy
import sys
from mill_controller.msg import Trajectory
from std_msgs.msg import Bool


# Use this publisher to send the output flag
pub = rospy.Publisher('gcode_flag',Bool,queue_size=1)



def callback(message):
    # Upon receiving a Trajectory message, parse the information and write a GCode file based on it.

    # If sending this gcode through easel, set to true
    easel = True

    # Open output file
    f = open('output.gcode', 'w')

    # Write the Header
    s = ''

    if(not easel):
        # Home
        s = s + '$22=1' + '\n'                  # Enable homing cycle
        s = s + '$H' + '\n'                     # Begin homing cycle

    # Basic Settings
    s = s + 'G90' + '\n'                    # Set absolute coordinates
    s = s + 'G21' + '\n'                    # Set mm
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate mode
    s = s + 'G54' + '\n'                    # Use WCS G54

    if(not easel):
        # Establish new origin for WCS
        s = s + 'G10 L20 P1 X0.0Y0.0Z0.0' + '\n'# Reset G54 WCS origin to 0,0,0 offset from this position.

        # Set G28 reference
        s = s + 'G0 X1.0 Y1.0' + '\n'           # Move to safe position in XY plane. Leave Z up from homing.
        s = s + 'G28.1' + '\n'                  # Set reference point at this position.

        # # Create Work Coordinate System G54
        # s = s + 'G0 X1.0 Y1.0' + '\n'           # Move to work 0. This should be taken as a user setting.
        # s = s + 'G10 L20 P1 X0 Y0 Z0' + '\n'    # Reset G54 WCS origin to this position.

    f.write(s)                              # Write to file



    # Write the Body
    # Set Feed rate and starting point
    s = 'G0' + ' X' + str(message.trajectory[0].point[0]) + ' Y' + str(message.trajectory[0].point[1]) + ' Z' + str(message.trajectory[0].point[2]) + ' F500.0' + '\n'

    f.write(s)

    # TODO Modify this section to use Trajectory "message"
    # Follow trajectory. Operating on the assumption that the trajectory can be approximated as a series of straight line motions from point to point. Improvement of this model will probably need example trajectories to test. Starting with fixed feed rate.
    for point_message in message.trajectory:
        # x, y, z, t = point_message.point[0], point_message.point[1], point_message.point[2], point_message.point[3]
        s = 'G1' + ' X' + str(point_message.point[0]) + ' Y' + str(point_message.point[1]) + ' Z' + str(point_message.point[2]) + ' F500.0' + '\n'
        # Write appropriate command to output
        f.write(s)



    # Write the Footer
    s = ''
    s = s + 'G90' + '\n'                    # Set to absolute coordinates
    s = s + 'G20' + '\n'                    # Set to inches
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate mode
    s = s + 'G54' + '\n'                    # Set WCS to G54
    s = s + 'G1 Z0.15000 F9.0' + '\n'       # Move bit out of harms way
    if(easel):
        s = s + 'G0 X0.0 Y0.0 Z0.0' + '\n'      # Move to work zero
    else:
        s = s + 'G28' + '\n'                    # Return to reference position
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
