#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
import pandas as pd
import sys, os
__location__ = os.path.realpath(os.path.join(os.getcwd(),os.path.dirname(__file__)))
            	

if __name__ == '__main__':

    #INITIALIZE MESSAGE CONTAINERS
    rhandMsg = Float32MultiArray()
    rarmMsg = Float64MultiArray()
    lhandMsg = Float32MultiArray()
    larmMsg = Float64MultiArray()

    #bimanual_demo.csv')
    rhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lhandMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rarmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
    larmMsg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
    df = pd.read_csv(os.path.join(__location__,"test.csv"))
    print(df)
    df = df.reset_index()

    for i,row in df.iterrows():
        rhdata = row.iloc[2:17].values
        radata = row.iloc[17:22].values
        lhdata = row.iloc[22:37].values
        ladata = row.iloc[37:42].values
        #print(rhdata)
        rhandMsg.data = rhdata
        rarmMsg.data = radata
        lhandMsg.data = lhdata
        larmMsg.data = ladata
        print(rhandMsg)
        print(rarmMsg)
        print(lhandMsg)
        print(larmMsg)
        input("Press Space to Continue...")