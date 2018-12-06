#!/usr/bin/env python

import cv2
import rospkg

# Global for
l_down = False  # flag to do things in callback while button held down probably
_img = None     # Image to work with in callback

# event = event type "cv2.EVENT_..."
# x, y = u,v, coordinates where click occurred
# flags = ?
# param = A third argument that can be passed when setting the callback. Exact usage uncertain but presumably to send more data.
# Note this this will trigger when moving around the window as well
def mouse_call(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(_img,(400,400),10,(0,0,0))


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

    str = 'Input text'
    text_origin = (400,400)
    fontFace = cv2.FONT_HERSHEY_PLAIN
    fontScale = 1
    cv2.putText(img_copy, str, text_origin,fontFace,fontScale,color)


    # Now mouse stuff
    global _img
    _img = img
    window_name = 'window'
    cv2.setMouseCallback(window_name, mouse_call)





    # Show image
    cv2.imshow('window',img_copy)
    cv2.waitKey(0)














    # # Image resizing code. My reference was much too large
    # img = cv2.resize(img, (640,int(640/1.5))  # Resize to given (width,height)
    # # img = cv2.resize(img, (0,0), fx=.3, fy=.3)  # Halve image size

    # cv2.imshow('window',img)
    # cv2.waitKey(0)

    # path = rospack.get_path('mill_controller') + '/images/resize.jpg'
    # cv2.imwrite(path,img)


    # # Close image windows for shutdown
    # cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
