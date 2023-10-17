#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

right_handMsg = Float32MultiArray()
right_handMsg.data = 15*[0]
left_handMsg = Float32MultiArray()
left_handMsg.data = 15*[0]



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

def update_left_hand():
    pub_left_hand = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]
    for i in range(4,15):
        if data.data[i]>5:
            scaled_joint=10*data.data[i]
            if scaled_joint >90:
                left_handMsg.data[i] = 90
            else:
                left_handMsg.data[i] = scaled_joint
        else:
            left_handMsg.data[i] = data.data[i]    
        # left_handMsg.data[i] = data.data[i]

    pub_left_hand.publish(left_handMsg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('left_hand_listener', anonymous=True)

    rospy.Subscriber("/leftservo_recorded",  Float32MultiArray, callback)
    rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    if sys.version_info < (3, 0):
        input = raw_input

    rospy.init_node('tf2_polhemus_listener')

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)

    turtle_vel = rospy.Publisher('%s/cmd_vel' % 'testjoint', geometry_msgs.msg.Twist, queue_size=1) 
    # setup output streams to robot   
    pub_right_hand = rospy.Publisher('/rightservo', Float32MultiArray, queue_size=10)
    pub_left_hand = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
    
    rate = rospy.Rate(1)
    print("Calibration Process : ")
    right_elbow_base = rospy.get_param('right_elbow_base')
    right_wrist_base = rospy.get_param('right_wrist_base')
    right_palm_base = rospy.get_param('right_palm_base')
    right_thumb = rospy.get_param('right_thumb')
    right_index_dist = rospy.get_param('right_index_dist')
    right_middle_dist = rospy.get_param('right_middle_dist')
    right_ring_dist = rospy.get_param('right_ring_dist')
    right_pinky_dist = rospy.get_param('right_pinky_dist')
    right_index_meta = rospy.get_param('right_index_meta')
    right_middle_meta = rospy.get_param('right_middle_meta')
    right_ring_meta = rospy.get_param('right_ring_meta')
    right_pinky_meta = rospy.get_param('right_pinky_meta')
    wristFlexTemp = 10*[0]
    wristPivTemp = 10*[0]
    wristRotTemp = 10*[0]
    thumbMetaTemp = 10*[0]
    thumbPivTemp = 10*[0]
    indexDisTemp = 10*[0]
    indexMetaTemp =10*[0]
    indexPivTemp = 10*[0]
    middleDisTemp = 10*[0]
    middleMetaTemp = 10*[0]
    middlePivTemp = 10*[0]
    ringDisTemp = 10*[0]
    ringMetaTemp = 10*[0]
    pinkyDisTemp = 10*[0]
    pinkyMetaTemp = 10*[0]

    while not rospy.is_shutdown():
        try:
            #target, parent
            palm_link = tfBuffer.lookup_transform(right_elbow_base, right_palm_base, rospy.Time())
            thumb_link = tfBuffer.lookup_transform(right_thumb, right_palm_base, rospy.Time())

            distal_index  = tfBuffer.lookup_transform(right_index_dist, right_index_meta, rospy.Time())
            meta_index = tfBuffer.lookup_transform(right_index_meta, right_palm_base, rospy.Time())

            distal_middle = tfBuffer.lookup_transform(right_middle_dist, right_middle_meta, rospy.Time())
            meta_middle = tfBuffer.lookup_transform(right_middle_meta, right_palm_base, rospy.Time())

            distal_ring = tfBuffer.lookup_transform(right_ring_dist, right_ring_meta, rospy.Time())
            meta_ring = tfBuffer.lookup_transform(right_ring_meta, right_palm_base, rospy.Time())

            distal_pinky = tfBuffer.lookup_transform(right_pinky_dist, right_pinky_meta, rospy.Time())
            meta_pinky = tfBuffer.lookup_transform(right_pinky_meta, right_palm_base, rospy.Time())
            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # wristFlexRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[1]))
        # wristPivRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[2]))
        # wristRotRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[0]))
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

        # wristFlex = map(wristFlexRaw, wristFlexClose, wristFlexOpen, -15, 70)
        wristFlex = 0    # wristFlexRaw
        wristPiv = 0     # wristPivRaw
        wristRot = 0     # wristRotRaw
        thumbMeta = map_values(thumbMetaRaw, 0, 15,  0, 90)
        thumbPiv = map_values(thumbPivRaw, 0, 15,  0, 90)
        indexDis = map_values(indexDisRaw, 0, 10,  0, 180)
        indexMeta = map_values(indexMetaRaw, 0, 15,  0, 90)
        # indexPiv = round(map_values(indexPivRaw, indexPivOpen+10, indexPivClose-10,  -10, 10))
        indexPiv = round(map_values(indexPivRaw, -20, 20,  -10, 10))
        middleDis = map_values(middleDisRaw, 0, 10,  0, 180)
        middleMeta = map_values(middleMetaRaw, 0, 15,  0, 90)
        # middlePiv = round(map_values(middlePivRaw, middlePivOpen+20, middlePivClose-20,  -20, 20))
        middlePiv = round(map_values(middlePivRaw, -20, 20,  -20, 20))
        ringDis   = map_values(ringDisRaw, 0, 10,  0, 180)
        ringMeta  = map_values(ringMetaRaw, 0, 15, 0, 90)
        pinkyDis  = map_values(pinkyDisRaw,  0, 10, 0, 180)
        pinkyMeta = map_values(pinkyMetaRaw,  0, 15,0, 90)

        right_handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]

        pub_right_hand.publish(right_handMsg)
        # listener()
        rate.sleep()

