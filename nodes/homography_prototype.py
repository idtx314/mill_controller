import cv2
import rospkg
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



# Globals
_l_down = False
_img = None
_old_img = None
_point = (0,0)
_counter = 0
_w_h = False
_m_h = False
_calibrated = False





def mouse_callback(event, x, y, flags, param):
    global _l_down, _img, _old_img, _point

    # Handle mouse event type
    if event == cv2.EVENT_LBUTTONDOWN:
        _img = _old_img.copy()
        cv2.circle(_img,(x,y),10,(0,0,0))
        _point = (x,y)
        _l_down = True
    elif event == cv2.EVENT_MOUSEMOVE and _l_down:
        _img = _old_img.copy()
        cv2.circle(_img,(x,y),10,(0,0,0))
    elif event == cv2.EVENT_LBUTTONUP:
        _l_down = False







def main(input):
    global _img, _old_img, _counter, _point

    w_h_accepted = False
    m_h_accepted = False
    # A list to hold clicked points
    plist = [(0,0),(0,0),(0,0),(0,0)]

    # Translate from message
    img = input

    # Save image data to globals
    _img = img.copy()
    _old_img = img.copy()

    # Start display window
    cv2.namedWindow('window')

    # Begin mouse callbacks
    cv2.setMouseCallback('window', mouse_callback)

    print("Click and drag to place a corner. Press space to save each corner")
    while(not w_h_accepted):
        # Until 4 corners collected
        while(_counter < 4):
            # Display image to user
            cv2.imshow('window',_img)

            # Wait for key
            key = cv2.waitKey(20)

            # If esc was pressed (maximum wait 20ms)
            if(key & 0xFF == 27):
                print("Exiting")
                break
            # If space was pressed
            elif(key & 0xFF == 32):
                # Save point
                print("Saving")
                plist[_counter] = _point
                # Update saved image
                _old_img = _img.copy()
                # Move to next point
                _counter += 1

            #TODO Add more modes, like choosing which corner to draw

        print("Corners collected")
        # Calculate homography
        # Convert plist into numpy array
        plist = np.array(plist)
        # Produce blank reference image
        ref_img = np.zeros((500,500,3),np.uint8)
        # Make numpy array from reference image corners. Remember, u,v style coords.
        rlist = np.array([[0,0],[500,0],[500,500],[0,500]])
        # Find homography
        # Save homography as w_h
        w_h,status = cv2.findHomography(plist,rlist)

        # Apply homography to image copy
        aligned_img = cv2.warpPerspective(img.copy(),w_h,(500,500))

        # Reset counter
        _counter = 0

        # Display result to user
        print('press "s" to accept transform')
        cv2.imshow("window",aligned_img)
        key = cv2.waitKey()

        # If accepted
        if(key & 0xFF == ord('s')):
            w_h_accepted = True
            # save warped image
            img = aligned_img.copy()


    # Save image data to globals
    _img = img.copy()
    _old_img = img.copy()

    print("Click and drag to place a corner. Press space to save each corner")
    while(not m_h_accepted):
        # Until 4 corners collected
        while(_counter < 4):
            # Display image to user
            cv2.imshow('window',_img)

            # Wait for key (maximum wait 20ms)
            key = cv2.waitKey(20)

            # If esc was pressed
            if(key & 0xFF == 27):
                print("Exiting")
                break
            # If space was pressed
            elif(key & 0xFF == 32):
                # Save point
                print("Saving")
                plist[_counter] = _point
                # Update saved image
                _old_img = _img.copy()
                # Move to next point
                _counter += 1

            #TODO Add more modes, like choosing which corner to draw

        print("Corners collected again")
        # Calculate homography
        # Convert plist into numpy array
        plist = np.array(plist)
        # Produce blank reference image TODO in correct ratio
        ref_img = np.zeros((640,480,3),np.uint8)
        # Make numpy array from reference image corners. Remember, u,v style coords.
        rlist = np.array([[0,0],[640,0],[640,480],[0,480]])
        # Find homography
        # Save homography as m_h
        m_h,status = cv2.findHomography(plist,rlist)

        # Apply homography to new image copy
        aligned_img = cv2.warpPerspective(img.copy(),m_h,(640,480))

        # Reset counter
        _counter = 0

        # Display result to user
        print('press "s" to accept transform')
        cv2.imshow("window",aligned_img)
        key = cv2.waitKey()

        # If accepted
        if(key & 0xFF == ord('s')):
            m_h_accepted = True
            # save warped image
            img = aligned_img.copy()


    # End callback

    print("Calibration complete")
    print("any key to close")
    cv2.imshow('window',img)
    cv2.waitKey(0)






















if __name__ == '__main__':
    # Open image, using file images for now
    rospack = rospkg.RosPack()
    path = rospack.get_path('mill_controller') + '/images/resize.jpg'
    img = cv2.imread(path,cv2.IMREAD_COLOR)

    main(img)
