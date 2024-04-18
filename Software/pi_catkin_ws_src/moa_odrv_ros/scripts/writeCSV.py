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
    name_dict = {
                'rh_wrist_flex': [0.0,0.1,0.2,0.3],
                'rh_wrist_piv': [1.0,1.1,1.2,1.3],
                'rh_wrist_rot': [2.0,2.1,2.2,2.3],
                'rh_thumb_meta': [3.0,3.1,3.2,3.3],
                'rh_thumb_piv': [4.0,4.1,4.2,4.3],
                'rh_index_dist': [5.0,0.0,0.0,0.0],
                'rh_index_meta': [6.0,0.0,0.0,0.0],
                'rh_index_piv': [7.0,0.0,0.0,0.0],
                'rh_middle_dist': [8.0,0.0,0.0,0.0],
                'rh_middle_meta': [9.0,0.0,0.0,0.0],
                'rh_middle_piv': [10.0,0.0,0.0,0.0],
                'rh_ring_dist': [11.0,0.0,0.0,0.0],
                'rh_ring_meta': [12.0,0.0,0.0,0.0],
                'rh_pinky_dist': [13.0,0.0,0.0,0.0],
                'rh_pinky_meta': [14.0,0.0,0.0,0.0],
                
                'ra_torso_bend': [15.0,0.0,0.0,0.0],
                'ra_flex': [16.0,0.0,0.0,0.0],
                'ra_abd': [17.0,0.0,0.0,0.0],
                'ra_rot': [18.0,0.0,0.0,0.0],
                'ra_elbow': [19.0,0.0,0.0,0.0],

                'lh_wrist_flex': [20.0,0.0,0.0,0.0],
                'lh_wrist_piv': [21.0,0.0,0.0,0.0],
                'lh_wrist_rot': [22.0,0.0,0.0,0.0],
                'lh_thumb_meta': [23.0,0.0,0.0,0.0],
                'lh_thumb_piv': [24.0,0.0,0.0,0.0],
                'lh_index_dist': [25.0,0.0,0.0,0.0],
                'lh_index_meta': [26.0,0.0,0.0,0.0],
                'lh_index_piv': [27.0,0.0,0.0,0.0],
                'lh_middle_dist': [28.0,0.0,0.0,0.0],
                'lh_middle_meta': [29.0,0.0,0.0,0.0],
                'lh_middle_piv': [30.0,0.0,0.0,0.0],
                'lh_ring_dist': [31.0,0.0,0.0,0.0],
                'lh_ring_meta': [32.0,0.0,0.0,0.0],
                'lh_pinky_dist': [33.0,0.0,0.0,0.0],
                'lh_pinky_meta': [34.0,0.0,0.0,0.0],

                'la_torso_bend': [35.0,0.0,0.0,0.0],
                'la_flex': [36.0,0.0,0.0,0.0],
                'la_abd': [37.0,0.0,0.0,0.0],
                'la_rot': [38.0,0.0,0.0,0.0],
                'la_elbow': [39.0,0.0,0.0,0.0]
    }

    df = pd.DataFrame(name_dict)
    print(df)
    df.to_csv('test.csv')

    input("Press Space to Continue...")