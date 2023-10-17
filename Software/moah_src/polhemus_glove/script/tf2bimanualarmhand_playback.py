#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState





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

class MoahJointRemapper():
    def __init__(self):

        self.rate = rospy.Rate(100) # 10hz
        rospy.Subscriber("/moa_left/joint_states", JointState, self.callbackleft)
        rospy.Subscriber("/moa_right/joint_states", JointState, self.callbackright)
        
        #INITIALIZE ROS PUBLISHER TOPICS FOR HAND
        
        #INITIALIZE ROS PUBLISHER TOPICS FOR ARM
        self.pub_right_arm = rospy.Publisher('/rightarm/position_controller/command', Float64MultiArray, queue_size=1)
        self.pub_left_arm = rospy.Publisher('/leftarm/position_controller/command', Float64MultiArray, queue_size=1)
        
        #INITIALIZE MESSAGE CONTAINERS        
        self.rarmMsg = Float64MultiArray()
        self.larmMsg = Float64MultiArray()        
        self.rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.larmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.lhandMsg = Float32MultiArray()
        self.lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.rhandMsg = Float32MultiArray()
        self.rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rospy.Subscriber("/leftservo_recorded", Float32MultiArray, self.callbacklefthand)
        self.pub_left_hand = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=5)
        self.pub_right_hand = rospy.Publisher('/rightservo', Float32MultiArray, queue_size=5)

        self.rwristFlexTemp = 0
        self.rwristPivTemp = 0
        self.rwristRotTemp = 0
        self.lwristFlexTemp = 0
        self.lwristPivTemp = 0
        self.lwristRotTemp = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.tfBuffertflistener = tf2_ros.TransformListener(self.tfBuffer)

        self.right_elbow_base = rospy.get_param('right_elbow_base')
        self.right_wrist_base = rospy.get_param('right_wrist_base')
        self.right_palm_base = rospy.get_param('right_palm_base')
        self.right_thumb = rospy.get_param('right_thumb')
        self.right_index_dist = rospy.get_param('right_index_dist')
        self.right_middle_dist = rospy.get_param('right_middle_dist')
        self.right_ring_dist = rospy.get_param('right_ring_dist')
        self.right_pinky_dist = rospy.get_param('right_pinky_dist')
        self.right_index_meta = rospy.get_param('right_index_meta')
        self.right_middle_meta = rospy.get_param('right_middle_meta')
        self.right_ring_meta = rospy.get_param('right_ring_meta')
        self.right_pinky_meta = rospy.get_param('right_pinky_meta')
        self.run()

    def callbackright(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
        self.rwristFlexTemp = math.ceil(np.rad2deg(data.position[7]))
        self.rwristPivTemp = math.ceil(np.rad2deg(data.position[6]))
        self.rwristRotTemp = math.ceil(np.rad2deg(data.position[0]))

        # self.pub_right_hand.publish(self.rhandMsg)

        self.rarmMsg.data[0] = np.rad2deg(data.position[5])
        self.rarmMsg.data[1] = np.rad2deg(data.position[2])
        self.rarmMsg.data[2] = np.rad2deg(data.position[3])
        self.rarmMsg.data[3] = np.rad2deg(data.position[4])
        self.rarmMsg.data[4] = (np.rad2deg(data.position[1])-90)

        self.pub_right_arm.publish(self.rarmMsg)

        self.callbackrighthand()
        #[left_7_prona_supin, left_elbow_4_flex_exte, left_shoulder_1_flex_exte, left_shoulder_2_abdu_addu, 
        # left_shoulder_3_rotation, left_shoulder_ref_frame_joint, left_wrist_5_abdu_addu, left_wrist_6_flex_exte]


        # wristFlex = wristFlexRaw
        # wristPiv = wristPivRaw
        # wristRot = wristRotRaw
        # handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]

        # rarmMsg.data = [torso_bend, shoulder_flex, shoulder_abd, shoulder_rot, arm_elbow]

    def callbackleft(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
        self.lwristFlexTemp = math.ceil(np.rad2deg(data.position[7]))
        self.lwristPivTemp = math.ceil(np.rad2deg(data.position[6]))
        self.lwristRotTemp = math.ceil(np.rad2deg(data.position[0]))

        # self.pub_left_hand.publish(self.rhandMsg)

        self.larmMsg.data[0] = np.rad2deg(data.position[5])
        self.larmMsg.data[1] = np.rad2deg(data.position[2])
        self.larmMsg.data[2] = np.rad2deg(data.position[3])
        self.larmMsg.data[3] = np.rad2deg(data.position[4])
        self.larmMsg.data[4] = (np.rad2deg(data.position[1])-90)

        self.pub_left_arm.publish(self.larmMsg)

    def callbacklefthand(self,data):
        #directly update the /leftservo hand values
        self.lhandMsg.data[0] = self.lwristFlexTemp
        self.lhandMsg.data[1] = self.lwristPivTemp
        self.lhandMsg.data[2] = self.lwristRotTemp

        for i in range(3,14):
            # if self.lhandMsg.data[i] >20:
            #     self.lhandMsg.data[i] = 10*data.data[i]
            # else:
            self.lhandMsg.data[i] = data.data[i]
        self.lhandMsg.data[5] = 2*data.data[6]
        self.lhandMsg.data[8] = 2*data.data[9]
        self.lhandMsg.data[11] = 2*data.data[12]
        self.lhandMsg.data[13] = 2*data.data[14]

        #self.rhandMsg.data = self.lhandMsg.data


    def callbackrighthand(self):
        #directly update the /rightservo hand values
        try:
            #target, parent
            palm_link = self.tfBuffer.lookup_transform(self.right_elbow_base, self.right_palm_base, rospy.Time())
            thumb_link = self.tfBuffer.lookup_transform(self.right_thumb, self.right_palm_base, rospy.Time())
            distal_index  = self.tfBuffer.lookup_transform(self.right_index_dist, self.right_index_meta, rospy.Time())
            meta_index = self.tfBuffer.lookup_transform(self.right_index_meta, self.right_palm_base, rospy.Time())
            distal_middle = self.tfBuffer.lookup_transform(self.right_middle_dist, self.right_middle_meta, rospy.Time())
            meta_middle = self.tfBuffer.lookup_transform(self.right_middle_meta, self.right_palm_base, rospy.Time())
            distal_ring = self.tfBuffer.lookup_transform(self.right_ring_dist, self.right_ring_meta, rospy.Time())
            meta_ring = self.tfBuffer.lookup_transform(self.right_ring_meta, self.right_palm_base, rospy.Time())
            distal_pinky = self.tfBuffer.lookup_transform(self.right_pinky_dist, self.right_pinky_meta, rospy.Time())
            meta_pinky = self.tfBuffer.lookup_transform(self.right_pinky_meta, self.right_palm_base, rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()
            return
        # wristFlexRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[1]))
        # wristPivRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[2]))
        # wristRotRaw = round(math.degrees(euler_from_quaternion(palm_link.transform.rotation)[0]))
        thumbMetaRaw = round(math.degrees(euler_from_quaternion(thumb_link.transform.rotation)[2]))
        thumbPivRaw  = round(math.degrees(euler_from_quaternion(thumb_link.transform.rotation)[0]))
        indexDisRaw = round(math.degrees(euler_from_quaternion(distal_index.transform.rotation)[2]))
        indexMetaRaw = round(math.degrees(euler_from_quaternion(meta_index.transform.rotation)[2]))
        indexPivRaw = round(math.degrees(euler_from_quaternion(meta_index.transform.rotation)[0]))
        middleDisRaw = round(math.degrees(euler_from_quaternion(distal_middle.transform.rotation)[2]))
        middleMetaRaw = round(math.degrees(euler_from_quaternion(meta_middle.transform.rotation)[2]))
        middlePivRaw = round(math.degrees(euler_from_quaternion(meta_middle.transform.rotation)[0]))
        ringDisRaw   = round(math.degrees(euler_from_quaternion(distal_ring.transform.rotation)[2]))
        ringMetaRaw  = round(math.degrees(euler_from_quaternion(meta_ring.transform.rotation)[2]))
        pinkyDisRaw  = round(math.degrees(euler_from_quaternion(distal_pinky.transform.rotation)[2]))
        pinkyMetaRaw = round(math.degrees(euler_from_quaternion(meta_pinky.transform.rotation)[2]))

        # wristFlex = map(wristFlexRaw, wristFlexClose, wristFlexOpen, -15, 70)
        thumbMeta = map_values(thumbMetaRaw, 0, 90,  0, 90)
        thumbPiv = map_values(thumbPivRaw, 0, 90,  0, 90)
        indexDis = map_values(indexDisRaw, 0, 100,  10, 180)
        indexMeta = map_values(indexMetaRaw, -10, 51,  10, 90)
        # indexPiv = round(map_values(indexPivRaw, indexPivOpen+10, indexPivClose-10,  -10, 10))
        indexPiv = round(map_values(indexPivRaw, -20, 20,  -10, 10))
        middleDis = map_values(middleDisRaw, 0, 100,  10, 180)
        middleMeta = map_values(middleMetaRaw, 0, 50,  10, 90)
        # middlePiv = round(map_values(middlePivRaw, middlePivOpen+20, middlePivClose-20,  -20, 20))
        middlePiv = round(map_values(middlePivRaw, -20, 20,  -20, 20))
        ringDis   = map_values(ringDisRaw, 0, 100,  10, 180)
        ringMeta  = map_values(ringMetaRaw, 0, 50, 10, 90)
        pinkyDis  = map_values(pinkyDisRaw,  0, 100, 10, 180)
        pinkyMeta = map_values(pinkyMetaRaw,  0, 50,10, 90)

        self.rhandMsg.data = [self.rwristFlexTemp,self.rwristPivTemp,self.rwristRotTemp,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]


    def run(self):
        while not rospy.is_shutdown():
            self.pub_left_hand.publish(self.lhandMsg)
            self.pub_right_hand.publish(self.rhandMsg)
            self.rate.sleep()

def main():
    rospy.init_node('moah_arm_joint_listener')
    majl = MoahJointRemapper()


if __name__ == "__main__":
    if sys.version_info < (3, 0):
        input = raw_input
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    # pub = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
    # handMsg = Float32MultiArray()
    # handMsg.data = 15*[0]
