#!/usr/bin/env python

import cv2
import rospkg
import numpy as np

# Global for
l_down = False  # flag to do things in callback while button held down probably
_img = None     # Image to work with in callback
_old_img = None # The original image to reset in callback
# Globals to hold the coordinates of the most recent circle
_point = (-1,-1)
_counter = 0

# event = event type "cv2.EVENT_..."
# x, y = u,v, coordinates where click occurred
# flags = modifiers like holding down another mouse button or ctrl key
# param = A third argument that can be passed when setting the callback. Use this to send a variable and you can provide more data to a callback
# Note this this will trigger when moving around the window as well
def mouse_call(event, x, y, flags, param):
    global l_down
    global _img
    global _old_img
    global _point

    # Handle mouse event type
    if event == cv2.EVENT_LBUTTONDOWN:
        _img = _old_img.copy()
        cv2.circle(_img,(x,y),10,(0,0,0))
        _point = (x,y)
        l_down = True
    elif event == cv2.EVENT_MOUSEMOVE and l_down:
        _img = _old_img.copy()
        cv2.circle(_img,(x,y),10,(0,0,0))
    elif event == cv2.EVENT_MBUTTONDOWN:
        _
    elif event == cv2.EVENT_LBUTTONUP:
        l_down = False



def main():
    # Open image, using file images for now
    rospack = rospkg.RosPack()
    path = rospack.get_path('mill_controller') + '/images/resize.jpg'
    img = cv2.imread(path,cv2.IMREAD_COLOR)

    # # Show image
    cv2.namedWindow('window')
    # cv2.waitKey(0)

    # # Draw
    img_copy = img.copy()
    # center = (200,150)
    # radius = 10
    # color = (0,0,255)
    # cv2.circle(img_copy,center, radius,color)

    # start = (5, 5)
    # end = (100, 200)
    # cv2.line(img_copy,start,end,color)

    # str = 'Input text'
    # text_origin = (400,400)
    # fontFace = cv2.FONT_HERSHEY_PLAIN
    # fontScale = 1
    # cv2.putText(img_copy, str, text_origin,fontFace,fontScale,color)


    # Now mouse stuff
    global _img
    global _old_img
    _img = img.copy()
    _old_img = img.copy()
    window_name = 'window'
    cv2.setMouseCallback(window_name, mouse_call)

    # A list to hold clicked points
    plist = [(0,0),(0,0),(0,0),(0,0)]


    # # Show image
    # cv2.imshow('window',img_copy)
    # cv2.waitKey(0)


    # Reloads image until broken
    global _counter
    while(_counter < 4):
        # If esc is pressed (maximum wait 10ms)
        key = cv2.waitKey(20)
        if(key & 0xFF == 27):
            break
            print("Exiting")
        # If space is pressed
        elif(key & 0xFF == 32):
            # Save point
            print("Saving")
            plist[_counter] = _point
            # Update saved image
            _old_img = _img.copy()
            # Move to next place in record
            _counter += 1

        #TODO Add more modes, like choosing which corner to draw

        # # Print current instruction
        # s = ["Top Left Corner", "Top Right Corner", "Bottom Left Corner", "Bottom Right Corner"]
        # cv2.putText(_img,"Click and drag to select "+s[_counter], (_img.shape[1]/2,_img.shape[0]/10),fontFace,fontScale,color)

        # Show image
        cv2.imshow('window',_img)


    print("Corners collected")
    print plist

    # Homography test
    # Convert plist into numpy array
    plist = np.array(plist)
    # Produce blank reference image
    ref_img = np.zeros((500,500),np.uint8)
    # Make numpy array from reference image corners. Remember, u,v style coords.
    rlist = np.array([[0,0],[500,0],[500,500],[0,500]])
    # Find homography
    h,status = cv2.findHomography(plist,rlist)
    # Apply homogrphy to image, warping to reference dimensions
    aligned_img = cv2.warpPerspective(img.copy(),h,(500,500))
    # Display warped image
    cv2.imshow("window",aligned_img)
    # Waitkey()
    cv2.waitKey()








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
