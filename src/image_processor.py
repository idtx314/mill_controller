#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Use this publisher to send the output
pub = rospy.Publisher('processed_image',Image,queue_size=1)

# Create a translator object to convert images
converter = CvBridge()


def callback(input):
    # The callback accepts an Image message, performs visual processing and masking on it, and publishes the result for the pointcloud converter node.

    # Translate Image message into a cv2 image using existing encoding
    cv_image = converter.imgmsg_to_cv2(input,"bgr8")

    # Perform visual processing on the image to extract relevant data

    # Translate cv2 image back to an Image message using existing encoding
    msg = converter.cv2_to_imgmsg(cv_image,"passthrough")

    # Publish image message
    pub.publish(msg)





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("image_processor_node")
    rospy.Subscriber('raw_image',Image,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
