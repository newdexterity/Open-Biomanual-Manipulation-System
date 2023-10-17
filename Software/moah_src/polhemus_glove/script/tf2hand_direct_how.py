#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
import numpy as np

def talker():
    #INITIALIZE ROS PUBLISHER TOPICS FOR HAND
    pub_right_hand = rospy.Publisher('/rightservo', Float32MultiArray, queue_size=10)
    pub = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
    #INITIALIZE ROS PUBLISHER TOPICS FOR ARM
    pub_right_arm = rospy.Publisher('/rightarm/position_controller/command', Float64MultiArray, queue_size=10)
    pub_left_arm = rospy.Publisher('/leftarm/position_controller/command', Float64MultiArray, queue_size=10)
    #INITIALIZE ROS NODE
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #INITIALIZE MESSAGE CONTAINERS
    rhandMsg = Float32MultiArray()
    rarmMsg = Float64MultiArray()
    lhandMsg = Float32MultiArray()
    larmMsg = Float64MultiArray()

    while not rospy.is_shutdown():
        #initial pose
        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.02), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.02), np.rad2deg(0.0), 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        rarmMsg.data = [np.rad2deg(-0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.57-1.57)]
        larmMsg.data = [np.rad2deg(-0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.57-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        #handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]
        #move to opposed thumb in
        rhandMsg.data = [np.rad2deg(0), np.rad2deg(0.0), np.rad2deg(0.7), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        lhandMsg.data = [np.rad2deg(0), np.rad2deg(-0.02), np.rad2deg(-0.67), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        # rhandMsg.data = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        # lhandMsg.data = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.9), np.rad2deg(0.4), np.rad2deg(1.3), np.rad2deg(1.5-1.57)]
        #rarmMsg.data = [np.rad2deg(-0.25), np.rad2deg(0.9), np.rad2deg(0.4), np.rad2deg(1.3), np.rad2deg(1.5-1.57)]
        larmMsg.data = [np.rad2deg(-0.25), np.rad2deg(0.9), np.rad2deg(0.4), np.rad2deg(-1.3), np.rad2deg(1.7-1.57)]

#[left_shoulder_ref_frame_joint, left_shoulder_1_flex_exte, left_shoulder_2_abdu_addu, left_shoulder_3_rotation,left_elbow_4_flex_exte,  
        #   left_wrist_5_abdu_addu, left_wrist_6_flex_exte,  left_7_prona_supin, ]

        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        #move to opposed thumb up
        rhandMsg.data = [np.rad2deg(0.37), np.rad2deg(0.0), np.rad2deg(-1.02), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        lhandMsg.data = [np.rad2deg(-0.37), np.rad2deg(-0.02), np.rad2deg(1.05), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(1.0), np.rad2deg(0.4), np.rad2deg(1.1), np.rad2deg(1.5-1.57)]
        larmMsg.data = [np.rad2deg(-0.25), np.rad2deg(1.0), np.rad2deg(0.4), np.rad2deg(-1.1), np.rad2deg(1.75-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)
    # 1st value -ve = arm flexion
    # 2nd value +ve = arm ABD(outwards)
    # 3rd value +ve = outward rot
    # 4th value +ve = flex inward

        #move to opposed thumb out
        rhandMsg.data = [np.rad2deg(0.37), np.rad2deg(0.0), np.rad2deg(2.57), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        lhandMsg.data = [np.rad2deg(-0.37), np.rad2deg(-0.02), np.rad2deg(-2.57), 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.90), np.rad2deg(0.4), np.rad2deg(1.43), np.rad2deg(1.6-1.57)]
        larmMsg.data = [np.rad2deg(-0.25), np.rad2deg(0.90), np.rad2deg(0.4), np.rad2deg(-1.41), np.rad2deg(1.75-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        #start you motion 
        rhandMsg.data = [np.rad2deg(-0.4), np.rad2deg(0.2), np.rad2deg(1.57), 0.0, 0.0, 0.0, 0.0, 0.0, 180.0, 90.0, 0.0, 180.0, 90.0, 180.0, 90.0]
        lhandMsg.data = [np.rad2deg(-0.2), np.rad2deg(-0.0), np.rad2deg(0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.90), np.rad2deg(0.2), np.rad2deg(0.83), np.rad2deg(1.5-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.70), np.rad2deg(0.4), np.rad2deg(-0.81), np.rad2deg(1.6-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)
        # point to you end
        rhandMsg.data = [np.rad2deg(-0.4), np.rad2deg(0.2), np.rad2deg(0.57), 0.0, 0.0, 0.0, 0.0, 0.0, 180.0, 90.0, 0.0, 180.0, 90.0, 180.0, 90.0]
        lhandMsg.data = [np.rad2deg(-0.2), np.rad2deg(-0.0), np.rad2deg(0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.90), np.rad2deg(0.2), np.rad2deg(0.83), np.rad2deg(1.4-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.70), np.rad2deg(0.4), np.rad2deg(-0.81), np.rad2deg(1.5-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        #start me motion 
        rhandMsg.data = [np.rad2deg(0.8), np.rad2deg(0.0), np.rad2deg(0.37), 0.0, 0.0, 0.0, 0.0, 0.0, 180.0, 90.0, 0.0, 180.0, 90.0, 180.0, 90.0]
        lhandMsg.data = [np.rad2deg(-0.2), np.rad2deg(-0.0), np.rad2deg(0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.90), np.rad2deg(0.3), np.rad2deg(0.93), np.rad2deg(1.7-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.70), np.rad2deg(0.4), np.rad2deg(-0.81), np.rad2deg(1.5-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        #start good motion 
        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.8), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.0), np.rad2deg(-1.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(1.0), np.rad2deg(0.0), np.rad2deg(0.93), np.rad2deg(1.7-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.80), np.rad2deg(0.3), np.rad2deg(-0.95), np.rad2deg(1.7-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.8), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.0), np.rad2deg(-1.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(-0.20), np.rad2deg(0.80), np.rad2deg(0.0), np.rad2deg(0.93), np.rad2deg(1.4-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.80), np.rad2deg(0.3), np.rad2deg(-0.95), np.rad2deg(1.7-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.8), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.0), np.rad2deg(-1.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(1.0), np.rad2deg(0.0), np.rad2deg(0.93), np.rad2deg(1.7-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.80), np.rad2deg(0.3), np.rad2deg(-0.95), np.rad2deg(1.7-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.8), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.0), np.rad2deg(-1.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(-0.20), np.rad2deg(0.80), np.rad2deg(0.0), np.rad2deg(0.93), np.rad2deg(1.4-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.80), np.rad2deg(0.3), np.rad2deg(-0.95), np.rad2deg(1.7-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)

        rhandMsg.data = [np.rad2deg(0.0), np.rad2deg(0.0), np.rad2deg(1.8), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [np.rad2deg(-0.0), np.rad2deg(-0.0), np.rad2deg(-1.5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        #rarmMsg.data = [np.rad2deg(0.25), np.rad2deg(0.1), np.rad2deg(0.2), np.rad2deg(0.3), np.rad2deg(1.5-1.57)]
        rarmMsg.data = [np.rad2deg(0.0), np.rad2deg(1.0), np.rad2deg(0.0), np.rad2deg(0.93), np.rad2deg(1.7-1.57)]
        larmMsg.data = [np.rad2deg(0.0), np.rad2deg(0.80), np.rad2deg(0.3), np.rad2deg(-0.95), np.rad2deg(1.7-1.57)]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue")
        rospy.sleep(0.5)
        
    # wristFlex 0~90 (1:1) 
    # wristPiv 0~45 (1:1)
    # wrist rot (0 pos : 37, 180 pos :157 difference = 120
    # thumbMeta 90 = meta bend 45, 180 = meta+distal both bend 45
    # thumbPiv 
    # indexDis;
    # indexMeta;
    # indexPiv;
    # middleDis;
    # middleMeta;
    # middlePiv;
    # ringDis;
    # ringMeta;
    # pinkyDis;
    # pinkyMeta;

        


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass