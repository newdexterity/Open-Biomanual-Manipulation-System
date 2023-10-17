#!/usr/bin/env python
import pandas as pd
import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


if __name__ == "__main__":
    rospy.init_node("csv_playback")
    br = TransformBroadcaster()

    rate = rospy.Rate(60)

    #left_csv_file = "/home/aroa/moah_ws/src/polhemus_glove/data/trial_left.csv"
    #right_csv_file = "/home/aroa/moah_ws/src/polhemus_glove/data/trial_right.csv"
    both_csv_file = "/home/aroa/moah_ws/src/polhemus_glove/data/both_hands.csv"
    
    df = pd.read_csv(both_csv_file)

    for index, row in df.iterrows():
        lt = TransformStamped()
        lt_e = TransformStamped()
        rt = TransformStamped()
        rt_e = TransformStamped()

        # Name of the parent frame for the transforms note:need static transform from temp to map
        lt.header.frame_id = "polhemus_base"
        lt_e.header.frame_id = "polhemus_base"
        rt.header.frame_id = "polhemus_base"
        rt_e.header.frame_id = "polhemus_base"

        # Name of the frame
        lt.child_frame_id = "polhemus_station_3"
        lt_e.child_frame_id = "polhemus_station_2"
        rt.child_frame_id = "polhemus_station_5"
        rt_e.child_frame_id = "polhemus_station_4"

        lt.header.stamp = rospy.Time.now()
        lt_e.header.stamp = rospy.Time.now()
        rt.header.stamp = rospy.Time.now()
        rt_e.header.stamp = rospy.Time.now()

        lt.transform.translation.x = row["lh_position_x"]
        lt.transform.translation.y = row["lh_position_y"]
        lt.transform.translation.z = row["lh_position_z"]

        lt.transform.rotation.w = row["lh_orientation_w"]
        lt.transform.rotation.x = row["lh_orientation_x"]
        lt.transform.rotation.y = row["lh_orientation_y"]
        lt.transform.rotation.z = row["lh_orientation_z"]
    
        lt_e.transform.translation.x = row["le_position_x"]
        lt_e.transform.translation.y = row["le_position_y"]
        lt_e.transform.translation.z = row["le_position_z"]

        lt_e.transform.rotation.w = row["le_orientation_w"]
        lt_e.transform.rotation.x = row["le_orientation_x"]
        lt_e.transform.rotation.y = row["le_orientation_y"]
        lt_e.transform.rotation.z = row["le_orientation_z"]
        br.sendTransform(lt)
        br.sendTransform(lt_e)
        
        rt.transform.translation.x = row["rh_position_x"]
        rt.transform.translation.y = row["rh_position_y"]
        rt.transform.translation.z = row["rh_position_z"]

        rt.transform.rotation.w = row["rh_orientation_w"]
        rt.transform.rotation.x = row["rh_orientation_x"]
        rt.transform.rotation.y = row["rh_orientation_y"]
        rt.transform.rotation.z = row["rh_orientation_z"]

        rt_e.transform.translation.x = row["re_position_x"]
        rt_e.transform.translation.y = row["re_position_y"]
        rt_e.transform.translation.z = row["re_position_z"]

        rt_e.transform.rotation.w = row["re_orientation_w"]
        rt_e.transform.rotation.x = row["re_orientation_x"]
        rt_e.transform.rotation.y = row["re_orientation_y"]
        rt_e.transform.rotation.z = row["re_orientation_z"]
        br.sendTransform(rt)
        br.sendTransform(rt_e)

        rate.sleep()
#rosrun tf static_transform_publisher 0 0 0 0 0 0 temp map 100