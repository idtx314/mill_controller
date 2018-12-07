import cv2
import rospkg
import numpy as np



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
        l_down = False







def main():
    _w_h_accepted = False
    _m_h_accepted = False

    # Guard for repeated message calls
    if(_calibrated):
        return 0


    while(!_w_h_accepted):
        # Display image to user
        # Request workspace corners
        # Calculate homography
        # Save homography as _w_h
        # Apply homography to image copy
        # Display result to user
        # if(accepted):
        #     # set _w_h_accepted
        #     # save image copy

    while(!_m_h_accepted):
        # Display image copy to user
        # Request material corners
        # Calculate homography
        # Save homography as _m_h
        # Apply homography to new image copy
        # display result to user
        # if(accepted):
        #     # set _m_h_accepted

    # Set _calibrated

    # End callback

























if __name__ == '__main__':
    main()
