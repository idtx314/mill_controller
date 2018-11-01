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

    # Detect the message encoding

    # Translate Image message into a cv2 image using existing encoding. The image will revert to a more general CV encoding, like 8UC3.
    cv_image = converter.imgmsg_to_cv2(input,"passthrough")

    # Perform visual processing on the image to extract relevant data
    print cv_image.shape # Extract the dimensions and channel count(?)
    print cv_image.dtype # Extract the data type of the image data

    # Translate cv2 image back to an Image message using existing encoding
    msg = converter.cv2_to_imgmsg(cv_image,"passthrough")

    # Publish image message
    msg.encoding = "bgr8"
    pub.publish(msg)





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("image_processor_node")
    rospy.Subscriber('raw_image',Image,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
