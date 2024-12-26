#!/usr/bin/env python3

import os
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

WINDOW_TITLE = "Images From Folder"

IMAGE_DIR = "/home/swarm7/Documents/3d_person/image_dataset/perfect_bright_images"
VIEW_IMG = True


def init_imagenode():
    global image_pub
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/drone7/camera/image', Image, queue_size=1)
    show_images()
    rospy.spin()


def show_images():
    bridge = CvBridge()
    i = 0

    image_files = sorted(os.listdir(IMAGE_DIR))  # Get list of image files
    num_images = len(image_files)
    num_images = len(image_files)

    rate = rospy.Rate(15)  # Publish images at 30Hz

    iter = 0

    while True:
        image_path = os.path.join(IMAGE_DIR, image_files[iter])
        frame = cv2.imread(image_path)

        if frame is not None:
            i += 1
            img = Image()
            img.header.stamp = rospy.Time.now()
            img.header.seq = i
            img.height = frame.shape[0]
            img.width = frame.shape[1]
            ros_img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            img.data = ros_img_msg.data
            image_pub.publish(img)

            if VIEW_IMG:
                cv2.imshow(WINDOW_TITLE, frame)
                cv2.waitKey(1)

            rate.sleep()
        iter += 1
        if iter == num_images - 1:
            iter = 0

if __name__ == "__main__":
    try:
        init_imagenode()
    except rospy.ROSInterruptException:
        pass

