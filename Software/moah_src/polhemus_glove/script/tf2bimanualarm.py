#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def map_values(x, in_min,  in_max,  out_min, out_max):
    """
    limits input and scales the value from input range to output range
    input value x
    in_min and in_max is input range
    out_min and out_max is output range
    """
    if (x > in_max):
        return out_max
    elif (x < in_min):
        return out_min
    elif (in_max-in_min == 0):
        return x
    else:
        return ((x - in_min) * (out_max - out_min) / ((in_max - in_min) + out_min))

if __name__ == '__main__':
    if sys.version_info < (3, 0):
        input = raw_input

    rospy.init_node('tf2_polhemus_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    turtle_vel = rospy.Publisher('%s/cmd_vel' % 'testjoint', geometry_msgs.msg.Twist, queue_size=1)    
    pub = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
    handMsg = Float32MultiArray()
    handMsg.data = 15*[0]

    rate = rospy.Rate(10.0)
    print("Calibration Process : ")
    # this will wait for any user input
    input("Please put glove on and place open hand on table. Then press Enter to continue>>>")
    i=0
    cal_open = True
    cal_closed = False
    while not rospy.is_shutdown():
        try:

            right_arm_elbow_link = tfBuffer.lookup_transform('polhemus_station_1', 'polhemus_station_2', rospy.Time())
            right_elbow_wrist_link = tfBuffer.lookup_transform('polhemus_station_1', 'polhemus_station_2', rospy.Time())

            left_arm_elbow_link = tfBuffer.lookup_transform('polhemus_station_1', 'polhemus_station_2', rospy.Time())
            left_elbow_wrist_link = tfBuffer.lookup_transform('polhemus_station_1', 'polhemus_station_2', rospy.Time())
            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        wristFlexRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[1]))
        wristPivRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[2]))
        wristRotRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[0]))
        thumbMetaRaw = round(math.degrees(euler_from_quaternion(thumb_link.transform.rotation)[1]))
        thumbPivRaw  = round(math.degrees(euler_from_quaternion(thumb_link.transform.rotation)[0]))
        indexDisRaw = round(math.degrees(euler_from_quaternion(distal_index.transform.rotation)[1]))
        indexMetaRaw = round(math.degrees(euler_from_quaternion(meta_index.transform.rotation)[1]))
        indexPivRaw = round(math.degrees(euler_from_quaternion(meta_index.transform.rotation)[2]))
        middleDisRaw = round(math.degrees(euler_from_quaternion(distal_middle.transform.rotation)[1]))
        middleMetaRaw = round(math.degrees(euler_from_quaternion(meta_middle.transform.rotation)[1]))
        middlePivRaw = round(math.degrees(euler_from_quaternion(meta_middle.transform.rotation)[2]))
        ringDisRaw   = round(math.degrees(euler_from_quaternion(distal_ring.transform.rotation)[1]))
        ringMetaRaw  = round(math.degrees(euler_from_quaternion(meta_ring.transform.rotation)[1]))
        pinkyDisRaw  = round(math.degrees(euler_from_quaternion(distal_pinky.transform.rotation)[1]))
        pinkyMetaRaw = round(math.degrees(euler_from_quaternion(meta_pinky.transform.rotation)[1]))

        if (cal_open == True):
            # take 10 samples of opened palm position            
            if i > 9:
                cal_open = False
                cal_closed = True
                i = 0
                wristFlexOpen = round(np.mean(wristFlexTemp))
                wristPivOpen = round(np.mean(wristPivTemp))
                wristRotOpen = round(np.mean(wristRotTemp))
                thumbMetaOpen = round(np.mean(thumbMetaTemp))
                thumbPivOpen = round(np.mean(thumbPivTemp))
                indexDisOpen = round(np.mean(indexDisTemp))
                indexMetaOpen = round(np.mean(indexMetaTemp))
                indexPivOpen = round(np.mean(indexPivTemp))
                middleDisOpen= round(np.mean(middleDisTemp))
                middleMetaOpen= round(np.mean(middleMetaTemp))
                middlePivOpen = round(np.mean(middlePivTemp))
                ringDisOpen = round(np.mean(ringDisTemp))
                ringMetaOpen = round(np.mean(ringMetaTemp))
                pinkyDisOpen = round(np.mean(pinkyDisTemp))
                pinkyMetaOpen = round(np.mean(pinkyMetaTemp))
                print(thumbMetaOpen)
                print(indexDisOpen)
                print("Open Hand Samples Collected.")
                # this will wait for any user input
                input("Please grip hand while placed on table. Then press Enter to continue>>>")
            else:
                wristFlexTemp[i] = wristFlexRaw
                wristPivTemp[i] =  wristPivRaw
                wristRotTemp[i] =  wristRotRaw
                thumbMetaTemp[i] = thumbMetaRaw
                thumbPivTemp[i]  = thumbPivRaw
                indexDisTemp[i] =  indexDisRaw
                indexMetaTemp[i] = indexMetaRaw
                indexPivTemp[i] =  indexPivRaw
                middleDisTemp[i] = middleDisRaw
                middleMetaTemp[i] = middleMetaRaw
                middlePivTemp[i] = middlePivRaw
                ringDisTemp[i]   = ringDisRaw
                ringMetaTemp[i]  = ringMetaRaw
                pinkyDisTemp[i]  = pinkyDisRaw
                pinkyMetaTemp[i] = pinkyMetaRaw
                i=i+1
        elif (cal_closed == True):
            if i > 9:
                cal_open = False
                cal_closed = False
                i = 0
                wristFlexClose = round(np.mean(wristFlexTemp))
                wristPivClose = round(np.mean(wristPivTemp))
                wristRotClose = round(np.mean(wristRotTemp))
                thumbMetaClose = round(np.mean(thumbMetaTemp))
                thumbPivClose = round(np.mean(thumbPivTemp))
                indexDisClose = round(np.mean(indexDisTemp))
                indexMetaClose = round(np.mean(indexMetaTemp))
                indexPivClose = round(np.mean(indexPivTemp))
                middleDisClose= round(np.mean(middleDisTemp))
                middleMetaClose= round(np.mean(middleMetaTemp))
                middlePivClose = round(np.mean(middlePivTemp))
                ringDisClose = round(np.mean(ringDisTemp))
                ringMetaClose = round(np.mean(ringMetaTemp))
                pinkyDisClose = round(np.mean(pinkyDisTemp))
                pinkyMetaClose = round(np.mean(pinkyMetaTemp))
                print(thumbMetaClose)
                print(indexDisClose)
                print("Closed Hand Samples Collected.")
                # this will wait for any user input
                input("Please Relax Hand. Then press Enter to BEGIN TELEOPERATION >>>")
            else:
                wristFlexTemp[i] = wristFlexRaw
                wristPivTemp[i] =  wristPivRaw
                wristRotTemp[i] =  wristRotRaw
                thumbMetaTemp[i] = thumbMetaRaw
                thumbPivTemp[i]  = thumbPivRaw
                indexDisTemp[i] =  indexDisRaw
                indexMetaTemp[i] = indexMetaRaw
                indexPivTemp[i] =  indexPivRaw
                middleDisTemp[i] = middleDisRaw
                middleMetaTemp[i] = middleMetaRaw
                middlePivTemp[i] = middlePivRaw
                ringDisTemp[i]   = ringDisRaw
                ringMetaTemp[i]  = ringMetaRaw
                pinkyDisTemp[i]  = pinkyDisRaw
                pinkyMetaTemp[i] = pinkyMetaRaw
                i=i+1
        else:
            # wristFlex = map(wristFlexRaw, wristFlexClose, wristFlexOpen, -15, 70)
            wristFlex = wristFlexRaw
            wristPiv = wristPivRaw
            wristRot = wristRotRaw
            thumbMeta = map_values(thumbMetaRaw, thumbMetaOpen, thumbMetaClose,  0, 90)
            thumbPiv = map_values(thumbPivRaw, thumbPivOpen, thumbPivClose,  0, 90)
            indexDis = map_values(indexDisRaw, indexDisOpen, indexDisClose,  0, 180)
            indexMeta = map_values(indexMetaRaw, indexMetaOpen, indexMetaClose,  0, 90)
            # indexPiv = round(map_values(indexPivRaw, indexPivOpen+10, indexPivClose-10,  -10, 10))
            indexPiv = round(map_values(indexPivRaw, -10, 10,  -10, 10))
            middleDis = map_values(middleDisRaw, middleDisOpen, middleDisClose,  0, 180)
            middleMeta = map_values(middleMetaRaw, middleMetaOpen, middleMetaClose,  0, 90)
            # middlePiv = round(map_values(middlePivRaw, middlePivOpen+20, middlePivClose-20,  -20, 20))
            middlePiv = round(map_values(middlePivRaw, -20, 20,  -20, 20))
            ringDis   = map_values(ringDisRaw, ringDisOpen, ringDisClose,  0, 180)
            ringMeta  = map_values(ringMetaRaw, ringMetaOpen, ringMetaClose, 0, 90)
            pinkyDis  = map_values(pinkyDisRaw,  pinkyDisOpen, pinkyDisClose, 0, 180)
            pinkyMeta = map_values(pinkyMetaRaw,  pinkyMetaOpen, pinkyMetaClose,0, 90)

            handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]

            # msg = geometry_msgs.msg.Twist()
            # angles = euler_from_quaternion(thumb_link.transform.rotation)
            # msg.angular.x = math.degrees(angles[0])
            # msg.angular.y = math.degrees(angles[1])
            # msg.angular.z = math.degrees(angles[2])
            # msg.linear.x = meta_index.transform.translation.x
            # msg.linear.y = meta_index.transform.translation.y
            # msg.linear.z = meta_index.transform.translation.z
            # turtle_vel.publish(msg)
            pub.publish(handMsg)
        rate.sleep()

