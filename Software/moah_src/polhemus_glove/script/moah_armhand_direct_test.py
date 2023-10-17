#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

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
        #handMsg.data = [wristFlex,wristPiv,wristRot,thumbMeta,thumbPiv,indexDis,indexMeta,indexPiv,middleDis,middleMeta,middlePiv,ringDis,ringMeta,pinkyDis,pinkyMeta]
        rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0]
        lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0, 90.0]
        # rhandMsg.data = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        # lhandMsg.data = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 20.0]
        larmMsg.data = [0.0, 0.0, 0.0, 0.0, 20.0]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)

        rospy.sleep(1)
########
        
        rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        larmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)

        rospy.sleep(1)
    # 1st value -ve = arm flexion
    # 2nd value +ve = arm ABD(outwards)
    # 3rd value +ve = outward rot
    # 4th value +ve = flex inward

        
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