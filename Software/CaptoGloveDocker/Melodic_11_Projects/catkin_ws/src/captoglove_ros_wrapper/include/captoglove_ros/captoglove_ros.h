#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

#include <captogloveapi.h>
#include <logger.h>

#include <QtCore>
#include <QObject>
#include <QDebug>
#include <QString>
#include <time.h>
#include <math.h>
#include <string>
#include <sensor_msgs/BatteryState.h>
#include <stdio.h>

#include "ros_translate.h"
#include "std_msgs/Float32MultiArray.h"

namespace captoglove_ros{

class captoglove_ros : public QThread
{
    Q_OBJECT
    public:
        explicit captoglove_ros(int argc, char** argv);
        ~captoglove_ros();
        bool init();
        void run();
        static void shutdownHandler(int signum);



private slots:
        void publishROSInfoToTerminal                   (QString info);
        void publishROSErrorToTerminal                  (QString error);

        //void publishAPIInfoToTerminal                   (QString info);
        //void publishAPIErrorToTerminal                  (QString error);

        void publishInfoToTerminal                      (QString info);
        void publishWarningToTerminal                   (QString warning);
        void publishErrorToTerminal                     (QString error);
        void publishFatalToTerminal                     (QString fatal);

        //void publishToROS                               (ros_translate::LogType type, QString Text);

        void on_fingerStatesUpdated                     (captoglove_v1::FingerFeedbackMsg);
        void on_batteryLevelUpdated                     (captoglove_v1::BatteryLevelMsg);
        //void on_deviceInfoUpdated                       (captoglove_v1::DeviceInformationMsg);

private:

        int m_init_argc;
        char** m_init_argv;

        CaptoGloveAPI                                   *m_captogloveAPI;
        QString                                          m_log_path;

        // Publishers
        ros::Publisher m_fingerFeedback_Publisher;
        ros::Publisher m_batteryLevel_Publisher;
        ros::Publisher m_deviceInfo_Publisher;
        ros::Publisher pub_hand;
        
        };

}
