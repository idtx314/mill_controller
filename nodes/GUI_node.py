#!/usr/bin/env python

import cv2
import rospkg



def main():

    # Open image, using file images for now
    rospack = rospkg.RosPack()
    path = rospack.get_path('mill_controller') + '/images/resize.jpg'
    img = cv2.imread(path,cv2.IMREAD_COLOR)

    # Show image
    cv2.imshow('window',img)
    cv2.waitKey(0)

    # Draw
    img_copy = img.copy()
    center = (200,150)
    radius = 10
    color = (0,0,255)
    cv2.circle(img_copy,center, radius,color)

    start = (5, 5)
    end = (100, 200)
    cv2.line(img_copy,start,end,color)

    # Show image
    cv2.imshow('window',img_copy)
    cv2.waitKey(0)














#     # Image resizing code. My reference was much too large
#     img = cv2.resize(img, (640,480))

#     cv2.imshow('window',img)
#     cv2.waitKey(0)

#     cv2.imwrite('resize.jpg',img)


    # Close image windows for shutdown
    cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
