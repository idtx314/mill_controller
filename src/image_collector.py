#!/usr/bin/env python

"""
This node should constantly record the most recent image in a global variable. Whenever it receives a bool message over the topic 'gcode_sent_flag' it will publish the most recent image on the topic 'raw_image'.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError


# Use this publisher to send the output
pub = rospy.Publisher('raw_image',Image,queue_size=1)

# Create a global variable to hold the most recent image message
msg = Image()

# Create a camera object that monitors video1
# cam = cv2.VideoCapture(1)

# Create a translator object to convert images
converter = CvBridge()

# # Collect data about the camera
# width = cam.get(3)
# height = cam.get(4)

def callback(input):
    # Upon receiving an image, save it into a global message to be available for publishing.
    msg = input

def callback2(input):
    # Upon receiving a bool message over the topic 'gcode_sent_flag', publish the most recently recorded image.
    # Assert the encoding
    msg.encoding = "bgr8"

    # Publish the message
    pub.publish(msg)

    """
    # Upon receiving a flag from the gcode_sender node, the callback will collect a picture from the camera, translate it to a ROS message, and publish it for the next node.

    # Grab some frames to clear out the buffer
    # TODO experiment with reducing this number as much as possible
    for i in range(20):
        cam.grab()

    # Attempt to take the picture
    (ret, frame) = cam.read()

    # If successful
    if(ret):
        # Try to
        try:
            # Translate into a ROS message with encoding 8UC3
            img = converter.cv2_to_imgmsg(frame,"passthrough")

            # Assert the encoding to be bgr8.
            img.encoding = "bgr8"

            # Publish
            pub.publish(img)

        # If failure, print error
        except CvBridgeError as e:
            print(e)
    """




def main():
    # Perform camera calibration routine

    # Initialize the ROS node and Subscriber
    rospy.init_node("image_collector_node")
    rospy.Subscriber('gcode_sent_flag',Bool,callback2)
    rospy.Subscriber('usb_cam/image_raw',Image,callback)

    # Spin until shut down
    rospy.spin()

    # Release the video stream. Object will be destroyed on script exit.
    # cam.release()





if __name__ == '__main__':
    main()
