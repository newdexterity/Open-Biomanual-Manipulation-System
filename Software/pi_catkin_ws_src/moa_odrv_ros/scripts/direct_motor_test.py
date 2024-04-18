#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
import sys, os
__location__ = os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__)))


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
        input("Press Space to Continue...")
            	
        rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(lhandMsg)
        pub_right_hand.publish(rhandMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(rhandMsg)
        rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 10.0]
        larmMsg.data = [10.0, 10.0, 10.0, 10.0, 10.0]
        pub_right_arm.publish(rarmMsg)
        pub_left_arm.publish(larmMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(larmMsg)
        input("Press Space to Continue...")

        # rightarm values

        # max -22 or -22.2222 (79.2 or 80 degrees?)
        # max 10 (physically possible to goto 22 but don't) ( 36 degrees)
        # 1st value +ve = backward torso bend 

        # max 60       (216) 
        # max -25      (90)
        # 2nd value +ve = arm flexion forward
        # max 10      (36)
        # max -25     (90)
        # 3rd value -ve = arm ABD(outwards)
        # max 25
        # max -25
        # 4th value +ve = inward rot (to torso)
        # max 5
        # max -7
        # 5th value +ve = flex inward (up)


        
        # leftarm values

        # max -22 or -22.2222 (79.2 or 80 degrees?)
        # max 10 (physically possible to goto 22 but don't) ( 36 degrees)
        # 1st value +ve = backward torso bend 

        # max -60 (216)  
        # max 25 (90)
        # 2nd value -ve = arm flexion forward

        # max 10 (36)
        # max -25 (90)
        # 3rd value -ve = arm ABD(outwards)
        # max -25 (90)
        # max 25 (90)
        # 4th value -ve = inward rot (to torso)

        # max 13 (touching)
        # max -15 (almost straight)
        # 5th value +ve = flex inward (up)

		
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
