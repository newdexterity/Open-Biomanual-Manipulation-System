#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <moa_driver/hardware_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "moa_driver_node");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    moa_driver::MoaHWInterface robot;

    robot.init(nh, nh_priv);
    controller_manager::ControllerManager cm(&robot);
    ros::Rate ctrl_rate(200);
    ROS_INFO_STREAM("Fake driver initialized 321");
    while (ros::ok()) {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        ctrl_rate.sleep();
    }
    spinner.stop();
    ROS_INFO_STREAM("Shutting down.");
    return 0;
}