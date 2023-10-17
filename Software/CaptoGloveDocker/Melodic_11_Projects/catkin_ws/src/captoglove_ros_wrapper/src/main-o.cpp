#include "../include/captoglove_ros/captoglove_ros.h"
#include "signal.h"

int main(int argc, char** argv){
    QCoreApplication a(argc, argv);

    captoglove_ros::captoglove_ros *captoglove_ros = new captoglove_ros::captoglove_ros(argc, argv);

    return a.exec();
}
#
