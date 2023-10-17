#include <captoglove_ros/captoglove_ros.h>
#include <signal.h>
#include <iostream>
#include <string>

namespace captoglove_ros {

    ros_translate::ros_translate() {
    }

    ros_translate::~ros_translate() {
    }

// Fingers
    captoglove_ros_msgs::FingerFeedbackMsg ros_translate::FingerFeedbackMsg_PB2ROS(captoglove_v1::FingerFeedbackMsg pb_msg) {

        captoglove_ros_msgs::FingerFeedbackMsg ros_msg;

        ros_msg.ThumbFinger = pb_msg.thumb_finger();
        ros_msg.IndexFinger = pb_msg.index_finger();
        ros_msg.MiddleFinger = pb_msg.middle_finger();
        ros_msg.RingFinger = pb_msg.ring_finger();
        ros_msg.LittleFinger = pb_msg.little_finger();
        return ros_msg;

    }
    std_msgs::Float32MultiArray ros_translate::FingerFeedbackMsg_ROS2HAND(captoglove_v1::FingerFeedbackMsg hand_msg) {

        std_msgs::Float32MultiArray handMsg;
        // 2150 2150 2150 34640 33950 relaxed
        // 1780 1840 1750 27072 30048 flex
        // 1950 1960 1850 31000 30500

// thumbMeta.write(map(cmd_msg.data[3], 0, 90, 10, 170));
//   thumbPiv.write(map(cmd_msg.data[4], 0, 90, 10, 110));
//   indexDis.write(map(cmd_msg.data[5], 0, 180, 10, 170));
//   indexMeta.write(map(cmd_msg.data[6], 0, 90, 10, 160));
//   indexPiv.write(map(cmd_msg.data[7], 0, 10, 10, 70));
//   middleDis.write(map(cmd_msg.data[8], 0, 180, 10, 170));
//   middleMeta.write(map(cmd_msg.data[9], 0, 90, 10, 150));
//   middlePiv.write(map(cmd_msg.data[10], 0, 40, 30, 150));
//   ringDis.write(map(cmd_msg.data[11], 0, 180, 0, 170));
//   ringMeta.write(map(cmd_msg.data[12], 0, 90, 0, 170));
//   pinkyDis.write(map(cmd_msg.data[13], 0, 180, 0, 150));
//   pinkyMeta.write(map(cmd_msg.data[14], 0, 90, 0, 120));

        handMsg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        handMsg.data[3] = map(maptopercentage(hand_msg.thumb_finger(), 2150, 1950),0,100,0,90);
        handMsg.data[4] = map(maptopercentage(hand_msg.thumb_finger(), 2150, 1950),50,100,0,90);

        handMsg.data[5] = map(maptopercentage(hand_msg.index_finger(), 2150, 1960),0,100,0,90);
        handMsg.data[6] = map(maptopercentage(hand_msg.index_finger(), 2150, 1960),50,100,0,180);

        handMsg.data[8] = map(maptopercentage(hand_msg.middle_finger(), 2150, 1850),0,100,0,90);
        handMsg.data[9] = map(maptopercentage(hand_msg.middle_finger(), 2150, 1850),50,100,0,180);

        handMsg.data[11] = map(maptopercentage(hand_msg.ring_finger(), 34640, 31000),0,100,0,90);
        handMsg.data[12] = map(maptopercentage(hand_msg.ring_finger(), 34640, 31000),50,100,0,180);

        handMsg.data[13] = map(maptopercentage(hand_msg.little_finger(), 33950, 30500),0,100,0,90);
        handMsg.data[14] = map(maptopercentage(hand_msg.little_finger(), 33950, 30500),50,100,0,180);

        return handMsg;

    }

    captoglove_ros_wrapper::BatteryLevelMsg ros_translate::BatteryLevelMsg_PB2ROS(captoglove_v1::BatteryLevelMsg pb_msg) {

        captoglove_ros_wrapper::BatteryLevelMsg ros_msg;

        ros_msg.BatteryLevel = pb_msg.level();

        return ros_msg;

    }

    captoglove_ros_wrapper::DeviceInformationMsg ros_translate::DeviceInformationMsg_PB2ROS(captoglove_v1::DeviceInformationMsg pb_msg) {

        captoglove_ros_wrapper::DeviceInformationMsg ros_msg;

        ros_msg.DeviceName = pb_msg.device_name();
        //ros_msg.DeviceAddress = pb_msg.device_address();
        // TODO: Maybe add preffered connection parameters

        return ros_msg;

    }
    float ros_translate::maptopercentage(int value, int upper, int lower) {
        float percentage;
        percentage = 100.0*(upper-value)/(upper-lower);
        return percentage;
    }
    int ros_translate::map(float x, float in_min, float in_max, float out_min, float out_max) {
        if (x > in_max)
        {
            return out_max;
        }
        else if (x < in_min)
        {
            return out_min;
        }
        else
        {
            float val = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            return roundf(val * 100) / 100;
        }
    }

}

