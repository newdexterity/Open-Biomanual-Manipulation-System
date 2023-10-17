#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <string.h>

#include "captogloveapi.h"

#include <captoglove_ros_msgs/FingerFeedbackMsg.h>
#include <captoglove_ros_wrapper/BatteryLevelMsg.h>
#include <captoglove_ros_wrapper/DeviceInformationMsg.h>

#include "std_msgs/Float32MultiArray.h"

namespace captoglove_ros{

    class ros_translate: public QObject
{
    Q_OBJECT
public:
    ros_translate();
    ~ros_translate();

    typedef enum{
        log_info = 0,
        log_debug = 1,
        log_warning = 2,
        log_error = 3,
        log_fatal = 4
    } LogType;

    static captoglove_ros_msgs::FingerFeedbackMsg              FingerFeedbackMsg_PB2ROS           (captoglove_v1::FingerFeedbackMsg);
    static std_msgs::Float32MultiArray              FingerFeedbackMsg_ROS2HAND         (captoglove_v1::FingerFeedbackMsg);
    static captoglove_ros_wrapper::BatteryLevelMsg                BatteryLevelMsg_PB2ROS             (captoglove_v1::BatteryLevelMsg);
    static captoglove_ros_wrapper::DeviceInformationMsg           DeviceInformationMsg_PB2ROS        (captoglove_v1::DeviceInformationMsg);
    static float maptopercentage (int value, int upper, int lower);
    static int map(float x, float in_min, float in_max, float out_min, float out_max);
    private:
};
}
