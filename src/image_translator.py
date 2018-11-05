#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError


# Use this publisher to send the output
pub = rospy.Publisher('pointcloud_input',PointCloud2,queue_size=1)

# Create a translator object to convert images
converter = CvBridge()

# Set Scaling factor
scale = 1.0


def callback(input):
    # Upon receiving a processed image, this function translates it into a cv2 image, cycles through each pixel, and wherever a pixel is white it adds a point to a pointcloud at coordinates "pixel_index * scale". Once the image is processed it publishes the pointcloud message.

    # Convert Image message to cv2 image


    #





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("image_translator_node")
    rospy.Subscriber('processed_image',Image,callback)

    # Spin until shut down
    rospy.spin()





if __name__ == '__main__':
    main()
