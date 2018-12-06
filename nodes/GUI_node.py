#!/usr/bin/env python

import cv2
import rospkg



def main():

    # Open image, using file images for now
    rospack = rospkg.RosPack()
    path = rospack.get_path('mill_controller') + '/images/shapes.jpg'
    img = cv2.imread(path,cv2.IMREAD_COLOR)

    # Show image
    cv2.imshow('window',img)

    cv2.waitKey(0)

    img = cv2.resize(img, (640,480))

    cv2.imshow('window',img)
    cv2.waitKey(0)

    cv2.imwrite('resize.jpg',img)

    cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
