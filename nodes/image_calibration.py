import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


_l_down = False
_confirming = True
_mouse = 0
_full_cal = True
_mats = []
_corners = [(100,100),(200,100),(100,200),(200,200)]


'''
TODO
Derive _full_cal from launch arguments
'''



def main():

    # Initialize ROS node
    rospy.init_node("image_calibration_node")

    # Create a translator object to convert images
    converter = CvBridge()

    # Get environment ready
    prep_windows()
    prep_mice()

    # Retrieve an image from usb_cam
    msg = rospy.wait_for_message('usb_cam/image_raw',Image,None)
    # Convert new image using bgr8 encoding(?).
    img = converter.imgmsg_to_cv2(msg,"bgr8")

    if(_full_cal):
        calibrate_workspace(img)


    # load_ws_cal()
    # ws_img = warpPerspective(img)

    # calibrate_materialspace(ws_img)




# cv2.putText(img_copy, str, text_origin,fontFace,fontScale,color)


def calibrate_workspace(img):
    global _confirming, _mats, _corners, _mouse
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
                # Todo save homography[mouse global]
                calibrated = True
                break
            if(key & 0xFF == 27):
                break

        # clear variables
        _confirming = False
        homlist = []
        imlist = []
        _mouse = 0

    clear_windows()



def calibrate_materialspace(img):
    pass

    # While uncalibrated
        # Clear windows
        # Load instructions on blank copy
        # while 1
            # draw circles on cam image copy at global coords
            # Load cam image copy
            # key = waitKey(20)
            # if key == enter
                # break

        # reference mat = np.zeros((width,height,3), np.uint8)
        # for i in range(2,6)
            # homography[i] = calculate homography(global coords)
            # imlist[i] = apply homography[i] to cam image copy
        # Set confirming flag for mouse cbs so that clicks do something
        # Load instructions on blank copy
        # while 1
            # for i in range(2,6)
                # display imlist[i] to window i
            # if mouse global
                # draw rectangle on imlist[mouse global] copy
                # display imlist[mouse global] copy to window i
                # display imlist[mouse global] to main window
                # load new instructions to blank copy
            # key = waitkey(20)
            # if key is enter and mouse global
                # save homography[mouse global]
                # set not uncalibrated
                # break
            # if key is esc
                # break
        # clear confirming flag
        # clear homography list
        # clear imlist
        # clear mouse global to 0
    # clear calibrated
    # clear windows


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
    global _l_down, _corners

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
    main()
