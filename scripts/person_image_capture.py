#!/usr/bin/env python3

import sys
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from mavros_msgs.msg import RCIn
import os, re
import sys, datetime
from pathlib import Path
import time
import numpy as np

WINDOW_TITLE = "USB CAMERA"
CAMERA_ID = "/dev/video0"


SAVE_IMG = True
VIEW_IMG = False


# --------------------------------- CAMERA PARAMETERS --------------------------------- #
# Command : v4l2-ctl --list-formats-ext -d 0 # 0 is video0
# without compression : (800X600)@20fps, (3840X2160)@1fps, (3840X2880)@1fps, (416X312)@1fps, (640X480)@30fps, (1280X720)@10fps, (1920X1080)@5fps, (3840X3032)@1fps
# with compression : (800X600), (3840X2160), (3840X2880), (416X312), (640X480), (1280X720), (3840X3032) @30fps
COMPRESSION = False
FRAME_WIDTH = 640 # 640, 1920, 1280
FRAME_HEIGHT = 480 # 480, 1080, 720

# Command for available settings: v4l2-ctl -d /dev/video0 -list
BRIGHTNESS = 35                     # min=-64    max=64    step=1   default=0 
CONTRAST = 25                        # min=0      max=95    step=1   default=8
SATURATION = 100 #152                 # min=0      max=150   step=1   default=62
HUE = 0                              # min=-2000  max=2000  step=1   default=0
AUTO_WB = 1                          #                               default=1
GAMMA = 100                          # min=100    max=300   step=1   default=100
GAIN = 32 # 255                      # min=0      max=255   step=1   default=32
WHITE_BALANCE_TEMP = 4600 # 2800     # min=2800   max=6500  step=1   default=4600 value=4600 flags=inactive
SHARPNESS = 15                       # min=1      max=15    step=1   default=10
BACKLIGHT_COMPENSATION = 5           # min=0      max=12    step=1   default=5
EXPOSURE_AUTO = 1                    # min=0      max=3              default=3 manual_mode = 1; aperture_priority_mode = 3
EXPOSURE_ABSOLUTE = 180 # 166        # min=3      max=2047  step=1   default=166 value=166 flags=inactive
# --------------------------------- CAMERA PARAMETERS --------------------------------- #



# Initializing parameters
guided_mode = False
save_image = False



# Creating a saving directory
username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day))
username = os.getlogin()

# "/home/swarm1/Documents/3d_person"
if SAVE_IMG:
    maindir = Path('/home/%s/Documents/3d_person/image_dataset/' % username)
    runs_today = list(maindir.glob('*%s*_usbcam' % stamp))
    if runs_today:
        runs_today = [str(name) for name in runs_today]
        regex = 'run\d\d'
        runs_today=re.findall(regex,''.join(runs_today))
        runs_today = np.array([int(name[-2:]) for name in runs_today])
        new_run_num = max(runs_today)+1
    else:
        new_run_num = 1
    savedir = maindir.joinpath('%s_run%02d_usbcam' % (stamp,new_run_num))
    os.makedirs(savedir)



def state_callback(state):
    """
    Check if drone FCU is in loiter or guided mode
    """
    global guided_mode

    if state.mode == 'GUIDED':
        guided_mode = True
        print(f'GUIDED', end="\r")
    else:
        print(f"Loiter", end="\r")



def rc_callback(data):
    """
    Check if drone FCU is in loiter or guided mode
    """
    global save_image

    if data.channels[6] >= 1500:
        save_image = True
    else:
        save_image = False



def init_cameranode():
    global image_pub
    rospy.init_node('camera_publisher', anonymous=True)
    rospy.Subscriber('/drone7/mavros/state',State,state_callback)
    rospy.Subscriber('/drone7/mavros/rc/in', RCIn,rc_callback)
    image_pub = rospy.Publisher('/drone7/camera/image', Image, queue_size=1)
    show_camera()
    rospy.spin()
    
    



def show_camera():
    global guided_mode, image_pub

    video_capture = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
    bridge = CvBridge()
    i = 0

    # Compreesion settings
    # Command : v4l2-ctl --list-formats-ext -d 0 # 0 is video0
    if COMPRESSION: 
        video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    video_capture.set(cv2.CAP_PROP_BRIGHTNESS, BRIGHTNESS)
    video_capture.set(cv2.CAP_PROP_CONTRAST, CONTRAST)
    video_capture.set(cv2.CAP_PROP_SATURATION, SATURATION)
    video_capture.set(cv2.CAP_PROP_HUE, HUE)
    video_capture.set(cv2.CAP_PROP_AUTO_WB, AUTO_WB)
    video_capture.set(cv2.CAP_PROP_GAMMA, GAMMA)
    video_capture.set(cv2.CAP_PROP_GAIN, GAIN)
    video_capture.set(cv2.CAP_PROP_SHARPNESS, SHARPNESS)
    video_capture.set(cv2.CAP_PROP_BACKLIGHT, BACKLIGHT_COMPENSATION)
    video_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, EXPOSURE_AUTO)
    video_capture.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE_ABSOLUTE)

    if video_capture.isOpened():
        try:
            if VIEW_IMG:
                window_handle = cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
            savenum = 0
            img = Image()
            while True:
                ret_val, frame = video_capture.read()
                # frame = cv2.flip(frame, 1)
                # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                if ret_val:
                    i += 1
                    

                    if SAVE_IMG and save_image:
                        print("Saving Images!")
                        saving_stamp = (f"{tmp.month:02.0f}{tmp.day:02.0f}{tmp.year:04.0f}")
                        cv2.imwrite(str(savedir.joinpath(f'person1-{saving_stamp}-{savenum:06d}.png')), frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])

                    (target_width, target_height) = (int(frame.shape[1]/4), int(frame.shape[0]/4))
                    resized_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)

                    img.header.stamp = rospy.Time.now()
                    img.header.seq = i
                    img.height = int(frame.shape[0]/4)
                    img.width = int(frame.shape[1]/4)
                    ros_img_msg = bridge.cv2_to_imgmsg(resized_frame, "bgr8")
                    img.data = ros_img_msg.data
                    image_pub.publish(img)
                    savenum += 1
                
                if VIEW_IMG:
                    if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(WINDOW_TITLE, frame)
                
                keyCode = cv2.waitKey(10) & 0xFF

                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break

        finally:
            video_capture.release()
            if VIEW_IMG:
                cv2.destroyAllWindows()
    else:
        print("Unable to open camera")




if __name__ == "__main__":
    try:
        init_cameranode()
    except rospy.ROSInterruptException:
        pass
