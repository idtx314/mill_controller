#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Use this publisher to send the output
pub = rospy.Publisher('processed_image',Image,queue_size=1)

# Create a translator object to convert images
converter = CvBridge()

# Set threshold for color processing
threshold = 90


def callback(input):
    # The callback accepts an Image message, performs visual processing and masking on it, and publishes the result for the pointcloud converter node. Exactly how to process this seems a little delicate. I'm going to start by assuming the presence of material at every location in the frame except where subsurface/ink color is present. I'll test this initially by rendering those colors white and every other color black.

    # Translate Image message into a cv2 image using existing encoding. The image will revert to a more general CV encoding, like 8UC3.
    cv_image = converter.imgmsg_to_cv2(input,"passthrough")

    # Make a copy, as the original image will not be editable after conversion
    cv_image = cv_image.copy()

    # Perform visual processing on the image to extract relevant data.
    (rows,cols,chans)  = cv_image.shape # Extract the (rows, columns, channels).

    # Cycle through each row
    for row in range(rows):
        # Cycle through each column
        for col in range(cols):
            total = 0
            # Cycle through each channel
            for chan in range(chans):
                # Sum the intensity of the channels
                total += cv_image.item(row,col,chan)

            # Cycle through each channel
            for chan in range(chans):
                # If the total intensity is less than the threshold
                if (total < threshold):
                    # Set color to black
                    cv_image.itemset((row,col,chan),0)
                else:
                    # Set the color to white
                    cv_image.itemset((row,col,chan),255)


    # Translate cv2 image back to an Image message using existing encoding
    msg = converter.cv2_to_imgmsg(cv_image,"passthrough")

    # Assert encoding
    msg.encoding = "bgr8"

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
