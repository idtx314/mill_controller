#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError


# Use this publisher to send the output
pub = rospy.Publisher('raw_image',Image,queue_size=1)

# Create a camera object that monitors video1
cam = cv2.VideoCapture(1)

# Create a translator object to convert images
converter = CvBridge()

# # Collect data about the camera
# width = cam.get(3)
# height = cam.get(4)


def callback(input):
    # Upon receiving a flag from the gcode_sender node, the callback will collect a picture from the camera, translate it to a ROS message, and publish it for the next node.

    # Attempt to take the picture
    (ret, frame) = cam.read()

    # If successful
    if(ret):
        # Translate into a ROS message with existing encoding and publish
        try:
            pub.publish(converter.cv2_to_imgmsg(frame,"passthrough"))
        except CvBridgeError as e:
            print(e)








def main():
    # Perform camera calibration routine

    # Initialize the ROS node and Subscriber
    rospy.init_node("image_collector_node")
    rospy.Subscriber('gcode_sent_flag',Bool,callback)

    # Spin until shut down
    rospy.spin()

    # Release the video stream. Object will be destroyed on script exit.
    cam.release()





if __name__ == '__main__':
    main()
