#include <captoglove_ros/captoglove_ros.h>
#include <signal.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace captoglove_ros{

bool m_request_shutdown;

captoglove_ros::captoglove_ros(int argc, char** argv){
    m_init_argc = argc;
    m_init_argv = argv;
    m_request_shutdown = false;

    QCoreApplication::setApplicationName("CaptogloveROS");
    QCoreApplication::setApplicationVersion("0.8");

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addOptions({
            {{"c", "config path"},
                QCoreApplication::translate("main", "Location of the config .ini file"),
                QCoreApplication::translate("main", "path")}
        });

     parser.process(QCoreApplication::arguments());

    QString configFilePath = "";

    if (parser.isSet("c")){
        configFilePath = parser.values("c").at(0);

    }else{
        publishROSErrorToTerminal("No config path set. Using default values.");
    }

    publishROSInfoToTerminal("Starting CaptoGloveAPI!");

    m_captogloveAPI = new CaptoGloveAPI(this, configFilePath);


    if(!this->init()){
        publishROSErrorToTerminal("Master not found. Please restart node.");
        QCoreApplication::exit();
        return;
    }else{
        publishROSInfoToTerminal("Started CaptoGloveAPI!");
    }
    this->start();
}

captoglove_ros::~captoglove_ros(){

}


void captoglove_ros::shutdownHandler(int signum){
    m_request_shutdown = true;
}

bool captoglove_ros::init(){

    publishROSInfoToTerminal("Entered initialization!");


    // ros::init(<command line or remmaping arguments>, std::string node_name, uint32_t options
    ros::init(m_init_argc, m_init_argv, "captoglove_ros", ros::init_options::NoSigintHandler);

    if (! ros::master::check()){
        return false;
    }


    ros::start();
    ros::NodeHandle nh("/");
    //ros::NodeHandle pn("~");
    
    

    std::string ns = ros::this_node::getNamespace();


    signal(SIGINT, captoglove_ros::shutdownHandler);

    m_captogloveAPI->run();


    // TODO:: Add while not initialized / Check CaptoGlove initialization status
    // TODO:: Compare with systemprocess_controller architecture and how does getFingers work

    /*while (!m_captogloveAPI->getInit())
    {
        publishROSInfoToTerminal("Waiting for CaptoGloveAPI initialization...");
        sleep(1);
        if (m_captogloveAPI->getInit()){
            break;
        }else {
            publishROSInfoToTerminal("Still boolean negative!");
        }

    }
    if (m_captogloveAPI->getFingers()){


    */

    // TODO: Implement battery feedback and finger feedback to publishers

    connect(m_captogloveAPI, SIGNAL(updateBatteryState(captoglove_v1::BatteryLevelMsg)),
           SLOT(on_batteryLevelUpdated(captoglove_v1::BatteryLevelMsg)));

    connect(m_captogloveAPI, SIGNAL(updateFingerState(captoglove_v1::FingerFeedbackMsg)),
            SLOT(on_fingerStatesUpdated(captoglove_v1::FingerFeedbackMsg)));

    m_fingerFeedback_Publisher = nh.advertise<captoglove_ros_msgs::FingerFeedbackMsg>(ns + "/fingers/states", 10);
    m_batteryLevel_Publisher = nh.advertise<sensor_msgs::BatteryState>(ns + "/battery/status", 10);
    pub_hand = nh.advertise<std_msgs::Float32MultiArray>(ns + "/servo", 10);
    publishROSInfoToTerminal("Exiting initialization!");

    return true;

}

void captoglove_ros::run(){
    int i = 0;
    ros::Rate r(50);


    ros::Duration(5).sleep();

    publishROSInfoToTerminal("Entered run loop!");

    while(!m_request_shutdown){
        ros::spinOnce();

        i++;

        r.sleep();
    }

    // TODO: Add stop to CaptoGloveAPI method
    // m_captogloveAPI->stop(); --> commented it out to successfully build ros workspace

    delete m_captogloveAPI;
    ros::Duration(0.1).sleep();
    ros::shutdown();
    QCoreApplication::exit();

    return;
}

void captoglove_ros::on_fingerStatesUpdated(captoglove_v1::FingerFeedbackMsg msg){
    // printf("on_fingerstateupdated1 \n");
    m_fingerFeedback_Publisher.publish(ros_translate::FingerFeedbackMsg_PB2ROS(msg));
    pub_hand.publish(ros_translate::FingerFeedbackMsg_ROS2HAND(msg));
    // ROS_INFO("%d\n", ros_translate::FingerFeedbackMsg_PB2ROS(msg).ThumbFinger);
    // printf("end_on_fingerstateupdated \n");
}


void captoglove_ros::on_batteryLevelUpdated(captoglove_v1::BatteryLevelMsg msg){
    m_batteryLevel_Publisher.publish(ros_translate::BatteryLevelMsg_PB2ROS(msg));
}

//void captoglove_ros::on_deviceInfoUpdated(captoglove_v1::DeviceinformationMsg msg){
//    m_deviceInformation_Publisher.publish(ros_translate::DeviceInformationMsg_PB2ROS(msg));
//}

void captoglove_ros::publishROSInfoToTerminal(QString info){
    QString Text = QString("%1 [INFO]" + info).arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
    std::cout << "\033[34m" << Text.toStdString() << "\033[0m" << std::endl;

}

void captoglove_ros::publishROSErrorToTerminal(QString error){
    QString Text = QString("[%1] Error log: [ROS] " + error).arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
    std::cout << "\033[31m" << Text.toStdString() << "\033[0m" << std::endl;
}


void captoglove_ros::publishInfoToTerminal(QString info){
    std::cout <<"\033[33m" << info.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_info, info);
}


void captoglove_ros::publishWarningToTerminal(QString warning){
    std::cout <<"\033[33m" << warning.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_warning, warning);
}

void captoglove_ros::publishErrorToTerminal(QString error){
    std::cout <<"\033[31m" << error.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_error, error);
}

void captoglove_ros::publishFatalToTerminal(QString fatal){
    std::cout <<"\033[1m\033[31m" << fatal.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_fatal, fatal);
}

//void captoglove_ros::publishToROS(ros_translate::LogType type, QString Text){
    //m_robotLogPublisher.publish()
//}

/*
    void captoglove_ros::publishAPIInfoToTerminal(QString info){
        QString Text = QString("[%1] Info log: [API] "+info).arg(QDateTime::currentDateTime().toString("hh:mm:ss dd/MM/yyyy"));
        std::cout <<"\033[34m"<< Text.toStdString() << "\033[0m" << std::endl;
        publishToROS(ros_translate::log_info, Text);
    }

    void captoglove_ros::publishAPIErrorToTerminal(QString error){
        QString Text = QString("[%1] Error log: [API] "+error).arg(QDateTime::currentDateTime().toString("hh:mm:ss dd/MM/yyyy"));
        std::cout <<"\033[31m"<< Text.toStdString()<< "\033[0m" << std::endl;
        publishToROS(ros_translate::log_error, Text);
    }


    void captoglove_ros::publishInfoToTerminal(QString info){
        std::cout <<"\033[34m" << info.toStdString() << "\033[0m" << std::endl;
        publishToROS(ros_translate::log_info, info);
    }

*/

}