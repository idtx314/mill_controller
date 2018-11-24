#!/usr/bin/env python

"""
This node should constantly record the most recent image broadcast over 'usb_cam/image_raw'. Whenever it receives a bool message over the topic 'gcode_sent_flag' it will publish the most recent image on the topic 'raw_image'.
"""
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


# Use this publisher to send the output
pub = rospy.Publisher('raw_image',Image,queue_size=1)

# Create a global variable to hold the most recent image message
msg = Image()


def callback(input):
    # Upon receiving an image, save it into a global message to be available for publishing.
    global msg
    msg = input

def callback2(input):
    # Upon receiving a bool message over the topic 'gcode_sent_flag', publish the most recently recorded image.
    global msg
    pub.publish(msg)



def main():
    # Perform camera calibration routine

    # Initialize the ROS node and Subscriber
    rospy.init_node("image_collector_node")
    rospy.Subscriber('gcode_sent_flag',Bool,callback2)
    rospy.Subscriber('usb_cam/image_raw',Image,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
