#!/usr/bin/python3.8

import rospy
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import TimeReference,NavSatFix
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import SetMode, SetModeRequest, CommandTOL, ParamSet
from std_msgs.msg import Float64
import math
from math import atan2
import os,re
import sys
import datetime, time
from pathlib import Path



#------------------------------ SPEED CONTROLS -----------------------------#
FORWARD_SPEED = 0.5

LIMIT_SPEED = 1
LIMIT_VER_SPEED = 1
LIMIT_YAWRATE = 5
LIMIT_YAWSPEED = 1
#------------------------------ SPEED CONTROLS -----------------------------#



#------------------------------ PRINT OPTIONS ------------------------------#
print_state = True
#------------------------------ PRINT OPTIONS ------------------------------#



#------------------------------ INITILIZATION ------------------------------#
armed = False
guided_mode = False
vspeed = 0 # positive is upwards
hspeed = 0 # positive is to the left
fspeed = 0 # positive is forwards
yaw_speed = 0
yaw = 0
yawrate = 0
gps_x = gps_y = gps_t = 0
local_x = local_y = local_z = local_yaw = 0 
gps_lat = gps_long = gps_alt = gps_alt_rel = 0
drone_gps, source_gps = [0,0,0], [0,0,0]
time_last_det = None
h_det = w_det = 0
home_local_z = 0
home_local_z_received = False
#------------------------------ INITILIZATION ------------------------------#





def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians



def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion and 
    calculate quaternion components w, x, y, z
    """
    # Convert Euler angles to quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Calculate quaternion components
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return x, y, z, w



def pose_callback(pose):
    """
    x, y, z orientation of the drone
    """
    global yaw, alt, gps_x, gps_y
    global local_x, local_y, local_z, home_local_z, home_local_z_received, local_yaw
    global armed

    q = pose.pose.orientation
    _, _, y = euler_from_quaternion(q.x,q.y,q.z,q.w)
    local_yaw = y

    local_x = pose.pose.position.x
    local_y = pose.pose.position.y
    local_z = pose.pose.position.z

    if home_local_z_received == False and armed:
        home_local_z = local_z
        home_local_z_received = True
        print(f"Home height (local_z) set at arming: {home_local_z: .5f}")



def heading_callback(heading):
    """
    Returns drone's heading with respect to North
    """
    global head
    head = heading.data



def compass_hdg_callback(heading):
    """
    Returns drone's heading with respect to North
    """
    global head
    head = heading.data



def state_callback(state):
    """
    check if drone FCU is in LOITER or GUIDED mode
    """
    global guided_mode
    global print_state

    if state.mode == 'GUIDED':
        guided_mode = True
        if print_state:
            print("state.mode == 'GUIDED' -> guided_mode = True")
            print_state = False
    else:
        # print(f"{state.mode}:!!!!!!!!!!!!!!!! NOT OFFBOARD")
        guided_mode = False



def time_callback(gpstime):
    """
    returns the gps time
    """
    global gps_t
    gps_t = float(gpstime.time_ref.to_sec())



def gps_callback(gpsglobal):
    """
    returns gps loaction of the drone
    gps_lat, gps_long, gps_alt and 
    drone_gps = [gps_lat, gpa_long, gps_alt]
    """
    global gps_lat, gps_long, gps_alt, drone_gps

    gps_lat = gpsglobal.latitude
    gps_long = gpsglobal.longitude
    gps_alt = gpsglobal.altitude
    drone_gps[0], drone_gps[1], drone_gps[2] = gps_lat, gps_long, gps_alt



def rel_alt_callback(altrel):
    """
    returns relative gps altitude
    """
    global gps_alt_rel
    gps_alt_rel = altrel.data



def detection_callback(box):
    """
    target detection callback function
    """
    global horizontalerror_det, verticalerror_det, w_det, h_det
    global time_last_det

    w_det = box.bbox.size_x
    h_det = box.bbox.size_y

    # positive errors give right, up
    if box.bbox.center.x != -1:
        time_last_det = rospy.Time.now()
        # if box center is on LHS of image, error is positive
        # if box center is on upper half of image, error is positive
        horizontalerror_det = box.bbox.center.x - 0.5
        verticalerror_det = box.bbox.center.y - 0.5
        print(f"Horizontal Error: {horizontalerror_det: 0.3f} | Vertical Error: {verticalerror_det: 0.3f}")
        print(f"Width: {w_det: 0.3f} | Height: {h_det: 0.3f}")
    else:
        horizontalerror_det = verticalerror_det = size_det = 0

    return



def set_mode(mode):
    rospy.wait_for_service('/drone7/mavros/set_mode')
    print("Setmode")
    try:
        set_mode_service = rospy.ServiceProxy('/drone7/mavros/set_mode', SetMode)
        response = set_mode_service(custom_mode=mode)
        if response.mode_sent:
            rospy.loginfo(f"Mode {mode} set successfully.")
        else:
            rospy.logerr("Failed to set mode.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")



def set_param(param_name, value):
    rospy.wait_for_service('/drone7/mavros/param/set')
    try:
        param_set_service = rospy.ServiceProxy('/drone7/mavros/param/set', ParamSet)
        param_value = ParamValue(integer=0, real=value)
        response = param_set_service(param_id=param_name, value=param_value)
        if response.success:
            rospy.loginfo(f"Parameter {param_name} set to {value}.")
        else:
            rospy.logerr(f"Failed to set parameter {param_name}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")









def doautonomouscontrol():
    """
    main control loop
    """
    global fspeed, hspeed, vspeed, yaw_speed
    global twistpub, twistmsg
    global yaw
    global time_last_det

    # Initialize subscribers to mavros topics
    rospy.Subscriber('/drone7/mavros/state', State, state_callback)
    rospy.Subscriber('/drone7/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/drone7/mavros/time_reference', TimeReference, time_callback)
    rospy.Subscriber('/drone7/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber('/drone7/mavros/global_position/rel_alt', Float64, rel_alt_callback)
    rospy.Subscriber('/drone7/mavros/global_position/compass_hdg', Float64, compass_hdg_callback)

    # Initialize subscribers to bounding box, flow, keypoints topics
    rospy.Subscriber('/detection_box', Detection2D, detection_callback)

    # Initialize Publishers
    twistpub = rospy.Publisher('/drone7/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)

    twistmsg = Twist()
    rate = rospy.Rate(5)

    circle_radius = 200  # meters
    circle_speed = 10    # meters/second
    set_param('CIRCLE_RADIUS', circle_radius)
    set_param('CIRCLE_RATE', circle_speed)


    # feedback control loop
    while not rospy.is_shutdown():
        # Initiliazing all the speeds at 0
        fspeed, vspeed, hspeed, yaw_speed = 0, 0, 0, 0

        if not guided_mode:
            print("Not guided mode !!!!")
        else:
            if (time_last_det is not None and ((rospy.Time.now() - time_last_det) < rospy.Duration(5))):
                if h_det > 0.35:
                    print("setting mode to circle")
                    set_mode('CIRCLE')
                    break
                else:
                    fspeed = 0.5
                    hspeed = -3*horizontalerror_det
                    vspeed, yaw_speed = 0, 0
            else:
                fspeed, hspeed, vspeed, yaw_speed = 0, 0, 0, 0

            # Bound controls to ranges
            # lower bound first, upper bound second
            fspeed = min(max(fspeed,-LIMIT_SPEED), LIMIT_SPEED)
            hspeed = min(max(hspeed,-LIMIT_SPEED), LIMIT_SPEED)
            vspeed = min(max(vspeed,-LIMIT_VER_SPEED), LIMIT_VER_SPEED)
            yaw_speed = min(max(yaw_speed,-LIMIT_YAWSPEED), LIMIT_YAWSPEED)


            print(f'fspeed: {fspeed: 0.1f} | hspeed: {hspeed: 0.5f} | vspeed: {vspeed: 0.5f} | yaw_speed: {yaw_speed: 0.5f}')

            twistmsg.linear.x = fspeed
            twistmsg.linear.y = hspeed
            twistmsg.linear.z = vspeed
            twistmsg.angular.z = 0

            # Publishing
            twistpub.publish(twistmsg)
            rate.sleep()


    
    # while not rospy.is_shutdown():
    #         print("Giong in circle")


if __name__ == '__main__':

    print("Initializing autonomous control node...")
    rospy.init_node('autonomous_control_node', anonymous=False)
    twist_pub = rospy.Publisher('/drone7/mavros/setpoint_position/local', PoseStamped, queue_size=1)

    try:
        doautonomouscontrol()
    except rospy.ROSInterruptException:
        pass
