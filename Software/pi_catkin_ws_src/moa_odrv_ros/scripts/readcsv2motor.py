#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
import pandas as pd
import sys, os
__location__ = os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__)))
            	

def talker():
    #INITIALIZE ROS PUBLISHER TOPICS FOR HAND
    pub_right_hand = rospy.Publisher('/rightservo', Float32MultiArray, queue_size=10)
    pub_left_hand = rospy.Publisher('/leftservo', Float32MultiArray, queue_size=10)
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

    #bimanual_demo.csv')
    
    df = pd.read_csv(os.path.join(__location__,"test.csv"))
    print(df)
    df = df.reset_index()

    for i,row in df.iterrows():
        rhdata = row.iloc[2:17].values
        radata = row.iloc[17:22].values
        lhdata = row.iloc[22:37].values
        ladata = row.iloc[37:42].values

        rhandMsg.data = rhdata
        rarmMsg.data = radata
        lhandMsg.data = lhdata
        larmMsg.data = ladata

        rospy.loginfo(rhandMsg)
        rospy.loginfo(rarmMsg)
        rospy.loginfo(lhandMsg)
        rospy.loginfo(larmMsg)
        
        input("Press Space to Continue...")

    rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
    larmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
    pub_right_hand.publish(rhandMsg)
    pub_right_arm.publish(rarmMsg)
    pub_left_hand.publish(lhandMsg)
    pub_left_arm.publish(larmMsg)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
