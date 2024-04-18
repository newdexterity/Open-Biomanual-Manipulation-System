#!/usr/bin/env python
import rospy
import yaml

import numpy as np
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class JointPositionReader():
    def __init__(self, hand_dof, arm_dof, hand_smooth, arm_smooth, delay, aroa_rviz_vizualize=False):
        # with open(arm_config) as file:
        #     self.arm_joint_map = yaml.safe_load(file)
        # with open(hand_config) as file:
        #     self.hand_joint_map = yaml.safe_load(file)
        self.arm_angles = np.zeros(arm_dof)
        self.hand_angles = np.zeros(hand_dof)
        self.aroa_rviz_vizualise = aroa_rviz_vizualize
        self.hand_sf = hand_smooth
        self.arm_sf = arm_smooth
        self.delay = delay

        self.pub_hand = rospy.Publisher('/servo', Float32MultiArray, queue_size=10)
        self.pub_arm = rospy.Publisher('/leftarm/position_controller/command', Float64MultiArray, queue_size=10)
        self.handMsg = Float32MultiArray()
        self.armMsg = Float64MultiArray()

        self.handMsg.data = 15*[0]
        self.armMsg.data = 4*[0]

        self.arm_joint_map = {
            "left_shoulder_1_flex_exte": 0,
            "left_shoulder_2_abdu_addu": 1,
            "left_shoulder_3_rotation": 2,
            "left_elbow_4_flex_exte": 3,
            "left_wrist_5_abdu_addu": 4,
            "left_wrist_6_flex_exte": 5,
            "left_7_prona_supin": 6
        }

        self.hand_joint_map = {
            "left_thumb_meta_piv": 0,
            "left_thumb_meta_flex": 1,
            "left_thumb_proximal_flex": 2,
            "left_thumb_distal_flex": 3
        }
        fingers = ["index", "middle", "ring", "pinky"]
        i = 4
        for finger in fingers:
            self.hand_joint_map["human/"+finger+"_meta_piv"] = i
            self.hand_joint_map["human/"+finger+"_meta_flex"] = i+1
            self.hand_joint_map["human/"+finger+"_proximal_flex"] = i+2
            self.hand_joint_map["human/"+finger+"_middle_flex"] = i+3
            i += 4

        rospy.Subscriber("/joint_states", JointState, self.callback)

        # To visualize in RVIZ:
        if aroa_rviz_vizualize:
            self.js_pub = rospy.Publisher('/joint_states', JointState, queue_size=1000)
            self.js = JointState()
            self.aroa_left_arm_joints = [
                "leftarm_shoulder_flexion",
                "leftarm_shoulder_abduction",
                "leftarm_shoulder_rotation",
                "leftarm_elbow_flexion",
                "leftarm_wrist_rotation",
                "leftarm_wrist_abduction",
                "leftarm_wrist_flexion"
            ]
            self.aroa_joints = [
                "rightarm_shoulder_flexion",
                "rightarm_shoulder_abduction",
                # "rightarm_shoulder_rotation",
                "rightarm_elbow_flexion",
                "rightarm_wrist_rotation",
                "rightarm_wrist_abduction",
                "rightarm_wrist_flexion",
                "aroa_torso_vertical",
                "aroa_torso_horizontal",
                "aroa_neck_yaw",
                "aroa_head_pitch",
                "right_finger_mount_joint1",
                "right_finger_mount_joint2",
                "left_finger_mount_joint1",
                "left_finger_mount_joint2"
            ]
            for joint in self.aroa_left_arm_joints:
                self.js.name.append(joint)
                self.js.position.append(0.0)
            for joint in self.aroa_joints:
                self.js.name.append(joint)
                self.js.position.append(0.0)

    def callback(self, joint_state_msg):
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            if name in self.arm_joint_map:
                index = self.arm_joint_map[name]
                if index != -1:
                    self.arm_angles[index] = position
            elif name in self.hand_joint_map:
                index = self.hand_joint_map[name]
                if index != -1:
                    self.hand_angles[index] = position
        
        self.hand_angles[0]

        hand_adjusted_values = [
            -self.arm_angles[5],
            -self.arm_angles[4],
            (math.pi/2.0)-self.arm_angles[6],

            self.hand_angles[1]+self.hand_angles[2]+self.hand_angles[3],
            1.2*self.hand_angles[0], 
            self.hand_angles[6]+self.hand_angles[7],
            self.hand_angles[5],
            self.hand_angles[4],
            self.hand_angles[10]+self.hand_angles[11],
            self.hand_angles[9],
            self.hand_angles[8],
            self.hand_angles[14]+self.hand_angles[15],
            self.hand_angles[13],
            self.hand_angles[18]+self.hand_angles[19],
            self.hand_angles[17]
        ]
        hand_adjusted_values = np.rad2deg(hand_adjusted_values)
        
        for i in range(15):
            self.handMsg.data[i] = self.handMsg.data[i]*(1.0 - self.hand_sf) + self.hand_sf*hand_adjusted_values[i]            
        self.pub_hand.publish(self.handMsg)
        # rospy.loginfo(handMsg)
        
        # Maping human joint values to robot joint values:

        arm_adjusted_values = [
            -self.arm_angles[0],
            self.arm_angles[1],
            -self.arm_angles[2],
            self.arm_angles[3]
            # map_values(-self.arm_angles[0], -1.57, 2.36, -1.6, 0.8),
            # map_values(self.arm_angles[1], -1.57, 1.57, -0.3, 1.5),
            # map_values(-self.arm_angles[2], -1.57, 1.57, -1,1.5), 
            # map_values(self.arm_angles[3], 0, 3.14, 0,2.8)
        ]
        
        self.armMsg.data = [
            self.armMsg.data[0]*(1.0 - self.arm_sf) + self.arm_sf*(arm_adjusted_values[0]),
            self.armMsg.data[1]*(1.0 - self.arm_sf) + self.arm_sf*(arm_adjusted_values[1]),
            self.armMsg.data[2]*(1.0 - self.arm_sf) + self.arm_sf*(arm_adjusted_values[2]),
            self.armMsg.data[3]*(1.0 - self.arm_sf) + self.arm_sf*(arm_adjusted_values[3])
        ]
        self.pub_arm.publish(self.armMsg)

        if self.aroa_rviz_vizualise:
            self.js.header.stamp = rospy.Time.now()
            # leftarm_shoulder_flexion,
            self.js.position[0] = self.armMsg.data[0]
            # leftarm_shoulder_abduction
            self.js.position[1] = self.armMsg.data[1]
            # leftarm_shoulder_rotation
            self.js.position[2] = self.armMsg.data[2]
            # leftarm_elbow_flexion
            self.js.position[3] = self.armMsg.data[3]

            self.js.position[4] = - (self.arm_angles[6] + math.pi/2.0)
            
            self.js.position[5] = self.arm_angles[4]
            self.js.position[6] = self.arm_angles[5]

            self.js_pub.publish(self.js)
        rospy.loginfo("Sending to hardware...")
        time.sleep(self.delay)

    def get_arm_angles(self):
        return self.arm_angles

    def get_hand_angles(self):
        return self.hand_angles

def main():
    rospy.init_node('joint_position_reader')
    delay = rospy.get_param("~delay", 0.25)
    hand_smooth = rospy.get_param("~hand_smooth", 0.2)
    arm_smooth = rospy.get_param("~arm_smooth", 0.2)
    publish_aroa_js = rospy.get_param("~publish_aroa_joint_states", False)

    jpr = JointPositionReader(20, 7, hand_smooth, arm_smooth, delay, 
            aroa_rviz_vizualize=publish_aroa_js)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     arm_angles = jpr.get_arm_angles()
    #     hand_angles = jpr.get_hand_angles()
    #     # Your logic here
    #     rospy.loginfo("Arm angles: " + str(arm_angles) + " hand angles: " + str(hand_angles))
        
        # rospy.loginfo(armMsg)
        # 1st value -ve = arm flexion
        # 2nd value +ve = arm ABD(outwards)
        # 3rd value +ve = outward rot
        # 4th value +ve = flex inward

        # if publish_aroa_js:
        #     js.header.stamp = rospy.Time.now()
        #     # leftarm_shoulder_flexion,
        #     js.position[0] = armMsg.data[0]
        #     # leftarm_shoulder_abduction
        #     js.position[1] = armMsg.data[1]
        #     # leftarm_shoulder_rotation
        #     js.position[2] = armMsg.data[2]
        #     # leftarm_elbow_flexion
        #     js.position[3] = armMsg.data[3]

        #     js.position[4] = - (arm_angles[6] + math.pi/2.0)
            
        #     js.position[5] = arm_angles[4]
        #     js.position[6] = arm_angles[5]

        #     js_pub.publish(js)

        # rate.sleep()

def lim_range(x, out_min, out_max):
    if x>out_max:
        return out_max
    elif x<out_min:
        return out_min
    else:
        return x

def map_values(x, in_min, in_max, out_min, out_max):
    if x < 0:
        in_max = 0
        out_max = 0
    else:
        in_min = 0
        out_min = 0
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
