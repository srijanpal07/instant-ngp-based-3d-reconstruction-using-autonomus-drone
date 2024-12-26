#!/usr/bin/python3.8

from re import sub
import rospy
from rospy.client import init_node
from sensor_msgs.msg import Image, TimeReference
from vision_msgs.msg import Detection2D
import numpy as np
import cv2
import os, re
import datetime
from pathlib import Path
import time
from ultralytics import YOLO




# --------------------------- OPTIONS -------------------------- #
VIEW_IMG = False
VIEW_DET = False

SAVE_IMG = True
SAVE_DET = True
# --------------------------- OPTIONS --------------------------- #



# ------------------------ YOLO PARAMETERS ------------------------ #
RATE =10
MAX_DELAY = 5          # [seconds] delay between last detection and current image after which to just drop images to catch up
CONF_THRES = 0.3       # previously 0.25  # confidence threshold
IOU_THRES = 0.6        # previously 0.7 # NMS IOU threshold
MAX_DET = 5            # maximum detections per image
IMGSZ = 640     # 640, 160            # although camera capture size is (480, 640) # 640 image size to run inference on and it needs to be a square size 
DEVICE = 'cuda:0'      # DEVICE = 'cuda:0' or 'cpu'
RETINA_MASKS = False
HALF_PRECISION = False
SHOW_CONF = True
# ------------------------ YOLO PARAMETERS ------------------------ #


global pub, box
gps_t = 0
global_img, img_numpy = None, None
last_img_header_stamp = None




# ------------------------ SAVING DIRECTORY ------------------------ #
username = os.getlogin( )
tmp = datetime.datetime.now()
stamp = ("%02d-%02d-%02d" % 
    (tmp.year, tmp.month, tmp.day))


username = os.getlogin()

maindir = Path('/home/%s/Documents/3d_person/deploy/detection' % username)

runs_today = list(maindir.glob('*%s*_detection' % stamp))
if runs_today:
    runs_today = [str(name) for name in runs_today]
    regex = 'run\d\d'
    runs_today=re.findall(regex,''.join(runs_today))
    runs_today = np.array([int(name[-2:]) for name in runs_today])
    new_run_num = max(runs_today)+1
else:
    new_run_num = 1
savedir_seg = maindir.joinpath('%s_run%02d_detection' % (stamp,new_run_num))


runs_today = list(maindir.glob('*%s*_detection_images' % stamp))
if runs_today:
    runs_today = [str(name) for name in runs_today]
    regex = 'run\d\d'
    runs_today=re.findall(regex,''.join(runs_today))
    runs_today = np.array([int(name[-2:]) for name in runs_today])
    new_run_num = max(runs_today)+1
else:
    new_run_num = 1
savedir_img = maindir.joinpath('%s_run%02d_detection_images' % (stamp,new_run_num))

if SAVE_DET:
    os.makedirs(savedir_seg)
if SAVE_IMG:
    os.makedirs(savedir_img)
# ------------------------ SAVING DIRECTORY ------------------------ #
    

# YOLO paths and importing
YOLOv8_ROOT = '/home/swarm7/Documents/3d_person/yolo/train_yolo2/runs/detect/train/weights/'





def time_callback(gpstime):
    """ gps time callback """
    global gps_t
    gps_t = float(gpstime.time_ref.to_sec())



def imagecallback(img):
    """image callback function"""
    global pub, box
    global MODEL
    global global_img, img_numpy, last_img_header_stamp

    t1 = time.time()
    global_img = img
    img_numpy = np.frombuffer(img.data,dtype=np.uint8).reshape(img.height,img.width,-1)
    # print(f"img shape: ({img.height}, {img.width})") # (480, 640)
    last_img_header_stamp = img.header.stamp

    if VIEW_IMG:
        img_view = img_numpy
        # print(f"numpy image shape: {img_view.shape}") # (480, 640, 3)
        cv2.imshow('Img->Det Node',img_view)
        cv2.waitKey(1)
    
    if SAVE_IMG:
        print("Saving rgb images without detection!")
        savenum = img.header.seq
        saving_stamp = (f"{tmp.month:02.0f}{tmp.day:02.0f}{tmp.year:04.0f}")
        cv2.imwrite(str(savedir_img.joinpath(f'CamImgsToDetNode-{saving_stamp}-{savenum:06d}.jpg')),
                    img_numpy, [cv2.IMWRITE_JPEG_QUALITY, 100])



def init_detection_node():
    global pub, box
    global MODEL
    global global_img, img_numpy, last_img_header_stamp

    pub = rospy.Publisher('/detection_box', Detection2D, queue_size=1)
    box = Detection2D()

    print('Initializing YOLOv8 detection TensorRT model')
    MODEL = YOLO(YOLOv8_ROOT + 'target_detect.engine')

    rospy.init_node('detection_node', anonymous=False)

    rospy.Subscriber('/drone7/camera/image', Image, imagecallback)
    rospy.Subscriber('/drone7/mavros/time_reference',TimeReference,time_callback)

    rate = rospy.Rate(RATE)
    segmented_sequence = 0
    savenum = 0

    while not rospy.is_shutdown():
        if last_img_header_stamp is None:
            print("Detection Node: Image not Received yet!\n")
        elif rospy.Time.now() - last_img_header_stamp > rospy.Duration(MAX_DELAY):
            print("Detection Node: waited for more than max delay\n")
        else:
            results = MODEL.predict(img_numpy, conf=CONF_THRES, iou=IOU_THRES, 
                                    imgsz=IMGSZ, half=HALF_PRECISION, device=DEVICE, 
                                    verbose=False, max_det=MAX_DET, 
                                    retina_masks=RETINA_MASKS, show_conf=SHOW_CONF,
                                    save=False)
            
            
            
            print("---------------")
            print(results[0].boxes)
            print("---------------")
            print(results[0].boxes.xywhn)
            box_instances = results[0].boxes.xywhn.cpu().numpy()
            box_info = box_instances[0]
            print("---------------")
            print(box_info)
            print("---------------")



            img = global_img
            img.data = []


            if results[0].boxes is not None:
                annotated_frame = results[0].plot()
                if VIEW_DET:
                    cv2.imshow('Target Detection', annotated_frame)
                    cv2.waitKey(1)
                
                if SAVE_IMG:
                    print("Saving Detection")
                    savenum = savenum + 1
                    saving_stamp = (f"{tmp.month:02.0f}{tmp.day:02.0f}{tmp.year:04.0f}")
                    cv2.imwrite(str(savedir_seg.joinpath(f'person1_detect-{saving_stamp}-{savenum:06d}.png')), annotated_frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])

            
                box.header.seq = img.header.seq
                box.header.stamp = img.header.stamp
                box.header.frame_id = ''
                box.source_img = img
                box.bbox.center.x = box_info[0]
                box.bbox.center.y = box_info[1]
                box.bbox.center.theta = 0
                box.bbox.size_x = box_info[2]
                box.bbox.size_y = box_info[3]
                pub.publish(box)

            else:
                print("No Target Detected!!!")
                box.bbox.center.x = -1
                box.bbox.center.y = -1
                box.bbox.center.theta = -1
                box.bbox.size_x = -1
                box.bbox.size_y = -1
                pub.publish(box)

            # print("---------------")
            # if results[0].masks is not None:
            #     x_mean, y_mean = -1, -1
            #     max_pixel_count = 0

            #     for i, instantnce in enumerate(results[0].masks.data):
            #         mask = instantnce.cpu().numpy()
            #         # print(f"Mask Len: {mask.shape}")
            #         # (640, 640) changed from the numpy image (480, 640)

            #         # Pixel belonging to a segmented class is having a value of 1, rest 0
            #         smoke_indices = np.argwhere(mask == 1)
            #         smoke_pixel_count = len(smoke_indices)

            #         # Selecting the instance which has the maximum number of segemented pixels
            #         if max_pixel_count < smoke_pixel_count:
            #             max_pixel_count = smoke_pixel_count
            #             best_smoke_instance = i
            #             indices_mean = np.mean(smoke_indices, axis=0)
            #             # print(f"\n0: {indices_mean[0]} | 1: {indices_mean[1]}")
            #             # 0 : inverted y axis | 1: x axis

            #             x_mean, y_mean = indices_mean[1] , indices_mean[0]
            #             x_mean_norm = indices_mean[1] / IMGSZ
            #             y_mean_norm = indices_mean[0] / IMGSZ

            #         # print(f"x_mean_norm, y_mean_norm: {(x_mean_norm - 0.5): .3f},",
            #         #     f"{(0.5 - y_mean_norm): .3f}",
            #         #     f"| Segmented area: {smoke_pixel_count}\n")
            #         # print(f"Mask shape: {mask.shape}")
            #         # mask shape: (640, 640) | image length along (y, x)



            #     if x_mean == -1 and y_mean == -1:
            #         # No segmnetation data received
            #         print("No segmnetation data received!")
            #         box.bbox.center.x = box.bbox.center.y = -1
            #         box.bbox.center.theta = -1
            #         box.bbox.size_x = box.bbox.size_y = -1
            #         pub.publish(box)

            #     else:
            #         # Smoke Segmentation detected
            #         img_mask = results[0].masks.data[best_smoke_instance].cpu().numpy()
            #         img_mask = (img_mask * 255).astype("uint8")
            #         img_mask = cv2.resize(img_mask, (640, 480), interpolation=cv2.INTER_AREA)
            #         # print(f"Mask shape: {img_mask.shape}") # (480, 640)

            #         # centroid of smoke normalized to the size of the mask
            #         # x_mean_norm = x_mean / img_mask.shape[1]
            #         # y_mean_norm = y_mean / img_mask.shape[0]

            #         if VIEW_MASK:
            #             cv2.imshow('Smoke Mask', img_mask)
            #             cv2.waitKey(1)
                    
            #         img = global_img
            #         img.data = img_mask.flatten().tolist()
            #         img.height, img.width = img_mask.shape[0], img_mask.shape[1]

            #         annotated_frame = results[0].plot()
            #         # print(f"Plot shape: {annotated_frame.shape}")
            #         # (480, 640, 3) different from the mask shape (640, 640)
            #         if VIEW_SEGMENTATION:
            #             annotated_frame = cv2.circle(annotated_frame,
            #                                          (int(x_mean), int(y_mean*(480/640))),
            #                                         20, (255, 0, 0), -1)
            #             cv2.imshow('Smoke Segmentation', annotated_frame)
            #             cv2.waitKey(1)

            #         box.header.seq = img.header.seq
            #         box.header.stamp = img.header.stamp
            #         box.header.frame_id = ''
            #         box.source_img = img
            #         box.bbox.center.x = x_mean_norm
            #         box.bbox.center.y = y_mean_norm
            #         box.bbox.center.theta = 0
            #         box.bbox.size_x = smoke_pixel_count
            #         pub.publish(box)

            # else:
            #     print("No Smoke Segmented!!!")
            #     box.bbox.center.x = -1
            #     box.bbox.center.y = -1
            #     box.bbox.center.theta = -1
            #     box.bbox.size_x = -1
            #     box.bbox.size_y = -1
            #     pub.publish(box)


        rate.sleep()



if __name__ == '__main__':
    try:
        init_detection_node()
    except rospy.ROSInterruptException:
        pass
