#!/usr/bin/env python

'''
This script launches a ROS node that guides the user through calibration of the image processing functions.
'''

import cv2
import math
import numpy as np
import rospkg
import rospy
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


_l_down = False
_dragging = -1
_confirming = True
_mouse = 0
_mats = []
_corners = [(100,100),(200,100),(200,200),(100,200)]


'''
TODO
Implement proper instructions
'''



def main(args):

    # Initialize ROS node
    rospy.init_node("image_calibration_node")

    args[1] = args[1].upper()
    if (args[1] == 'NONE'):
        print("Skipped Calibration")
        return 0
    elif (args[1] == 'NORMAL'):
        full_cal = False
    elif (args[1] == 'FULL'):
        full_cal = True
    else:
        rospy.logerr("Malformed argument to image_calibration")
        return 1


    rospack = rospkg.RosPack()

    # Create a translator object to convert images
    converter = CvBridge()

    # Get environment ready
    prep_windows()
    prep_mice()

    # Retrieve an image from usb_cam
    msg = rospy.wait_for_message('usb_cam/image_raw',Image,None)
    # Convert new image using bgr8 encoding(?).
    img = converter.imgmsg_to_cv2(msg,"bgr8")

    if(full_cal):
        calibrate_workspace(img)

    # Load ws_cal
    path = rospack.get_path('mill_controller') + '/homographies/ws_hom.npy'
    try:
        ws_hom = np.load(path)
    except IOError:
        rospy.logerr("Error loading ws_hom.npy. File not found. Perform full calibration.")
        return 1

    ws_img = cv2.warpPerspective(img.copy(),ws_hom,(500,500))

    calibrate_materialspace(ws_img)

    print "completed calibration"



# cv2.putText(img_copy, str, text_origin,fontFace,fontScale,color)


def calibrate_workspace(img):
    global _confirming, _mats, _corners, _mouse
    rospack = rospkg.RosPack()
    calibrated = False
    homlist = []
    imlist = []

    while not calibrated:
        clear_windows()

        # Acquire corner selection from user
        # Load instructions on blank copy
        inst = cv2.putText(_mats[0].copy(), "Do this", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        cv2.imshow("0",inst)
        # Wait for feedback
        while(1):
            img_c = img.copy()
            for i in range(4):
                img_c = cv2.circle(img_c,_corners[i],10,(255,255,255))
            cv2.imshow("1",img_c)

            # Wait for enter
            key = cv2.waitKey(20)
            if(key & 0xFF == 13):
                break

        # Calculate and apply homographies
        ref = np.zeros((500,500,3), np.uint8)
        plist = np.array(_corners)
        rlist = np.array([[0,0],[500,0],[500,500],[0,500]])
        for i in range(4):
            homlist.append(cv2.findHomography(plist,rlist)[0])
            imlist.append(cv2.warpPerspective(img.copy(),homlist[i],(500,500)))

        # Request choice of homography from user
        _confirming = True
        # Load instructions on blank copy
        inst = cv2.putText(_mats[0].copy(), "Do that", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        cv2.imshow("0",inst)
        # Wait for feedback
        while(1):
            for i in range(4):
                cv2.imshow(str(i+2),imlist[i])
            if _mouse:
                img_r = cv2.rectangle(imlist[_mouse-2].copy(),(50,50),(450,450),(0,255,0),2)
                cv2.imshow(str(_mouse),img_r)
                cv2.imshow("1",imlist[_mouse-2])
                # load new instructions to blank copy
                inst = cv2.putText(_mats[0].copy(), "Do the other", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
                cv2.imshow("0",inst)

            # Wait for enter
            key = cv2.waitKey(20)
            if(key & 0xFF == 13 and _mouse):
                # Save chosen homography
                path = rospack.get_path('mill_controller') + '/homographies/ws_hom.npy'
                np.save(path,homlist[_mouse-2])
                calibrated = True
                break
            if(key & 0xFF == 27):
                break

        # clear variables
        _confirming = False
        homlist = []
        imlist = []
        _mouse = 0
        _corners = [(100,100),(200,100),(200,200),(100,200)]

    clear_windows()



def calibrate_materialspace(img):
    global _confirming, _mats, _corners, _mouse
    rospack = rospkg.RosPack()
    calibrated = False
    homlist = []
    imlist = []

    while not calibrated:
        clear_windows()

        # Acquire corner selection from user
        # Load instructions on blank copy
        inst = cv2.putText(_mats[0].copy(), "Do this again", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        cv2.imshow("0",inst)
        # Wait for feedback
        while(1):
            img_c = img.copy()
            for i in range(4):
                img_c = cv2.circle(img_c,_corners[i],10,(255,255,255))
            cv2.imshow("1",img_c)

            # Wait for enter
            key = cv2.waitKey(20)
            if(key & 0xFF == 13):
                break

        # Calculate and apply homographies
        ref = np.zeros((640,int(640*.75),3), np.uint8)
        plist = np.array(_corners)
        rlist = np.array([[0,0],[640,0],[640,480],[0,480]])
        for i in range(4):
            homlist.append(cv2.findHomography(plist,rlist)[0])
            imlist.append(cv2.warpPerspective(img.copy(),homlist[i],(640,480)))

        # Request choice of homography from user
        _confirming = True
        # Load instructions on blank copy
        inst = cv2.putText(_mats[0].copy(), "Do that again", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
        cv2.imshow("0",inst)
        # Wait for feedback
        while(1):
            for i in range(4):
                cv2.imshow(str(i+2),imlist[i])
            if _mouse:
                img_r = cv2.rectangle(imlist[_mouse-2].copy(),(50,50),(450,450),(0,255,0),2)
                cv2.imshow(str(_mouse),img_r)
                cv2.imshow("1",imlist[_mouse-2])
                # load new instructions to blank copy
                inst = cv2.putText(_mats[0].copy(), "Do the other again", (50,100),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255))
                cv2.imshow("0",inst)

            # Wait for enter
            key = cv2.waitKey(20)
            if(key & 0xFF == 13 and _mouse):
                # Save chosen homography
                path = rospack.get_path('mill_controller') + '/homographies/ms_hom.npy'
                np.save(path,homlist[_mouse-2])
                calibrated = True
                break
            if(key & 0xFF == 27):
                break

        # clear variables
        _confirming = False
        homlist = []
        imlist = []
        _mouse = 0
        _corners = [(100,100),(200,100),(200,200),(100,200)]

    clear_windows()


def prep_windows():
    global _mats

    # list of empty mats
    _mats.append(np.zeros((300,300,3),np.uint8))
    _mats.append(np.zeros((700,700,3),np.uint8))
    for i in range(2,6):
        _mats.append(np.zeros((200,200,3),np.uint8))

    # Create and move windows around
    coords = [(100,100),(300,100),(100,300),(300,300),(100,500),(300,500)]
    for i in range(6):
        cv2.namedWindow(str(i),cv2.WINDOW_GUI_NORMAL)
        cv2.moveWindow(str(i),coords[i][0],coords[i][1])
        clear_windows()



def clear_windows(w_list=[]):
    global _mats

    if len(w_list) == 0:
        # Clear all windows
        for i in range(6):
            cv2.imshow(str(i),_mats[i])
    else:
        for i in range(len(w_list)):
            cv2.imshow(w_list[i],_mats[int(w_list[i])])




def prep_mice():
    # Start mouse callbacks for each window
    callback = [m_call_0, m_call_1, m_call_2, m_call_3, m_call_4, m_call_5]
    for i in range(6):
        # tie callback[i] to window[i]
        cv2.setMouseCallback(str(i), callback[i])



def m_call_0(event, x, y, flags, param):
    # Dead window for instruction clicks
    pass

def m_call_1(event, x, y, flags, param):
    # Main window mouse handler
    global _l_down, _corners, _dragging
    closest_point = (-1, 8000)
    position = (x,y)

    # Find nearest corner
    for i in range(4):
        d = find_distance(position, _corners[i])
        if d < closest_point[1]:
            closest_point = (i, d)

    if event == cv2.EVENT_LBUTTONDOWN:
        # If the corner is close enough, move it
        if closest_point[1] < 10:
            _corners[closest_point[0]] = (x,y)
            _dragging = closest_point[0]
        _l_down = True
    elif event == cv2.EVENT_MOUSEMOVE and _l_down and (_dragging != -1):
        # If the corner is close enough, move it
        _corners[_dragging] = (x,y)
    elif event == cv2.EVENT_LBUTTONUP:
        _dragging = -1
        _l_down = False

def find_distance(point1, point2):
    d = math.sqrt(pow(point1[0]-point2[0],2) + pow(point1[1]-point2[1],2))
    return d

def m_call_2(event, x, y, flags, param):
    global _confirming, _mouse
    # Selection window mouse handler
    if event == cv2.EVENT_LBUTTONDOWN and _confirming:
        _mouse = 2

def m_call_3(event, x, y, flags, param):
    global _confirming, _mouse
    # Selection window mouse handler
    if event == cv2.EVENT_LBUTTONDOWN and _confirming:
        _mouse = 3

def m_call_4(event, x, y, flags, param):
    global _confirming, _mouse
    # Selection window mouse handler
    if event == cv2.EVENT_LBUTTONDOWN and _confirming:
        _mouse = 4

def m_call_5(event, x, y, flags, param):
    global _confirming, _mouse
    # Selection window mouse handler
    if event == cv2.EVENT_LBUTTONDOWN and _confirming:
        _mouse = 5

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.argv.append('NORMAL')

    main(sys.argv)

