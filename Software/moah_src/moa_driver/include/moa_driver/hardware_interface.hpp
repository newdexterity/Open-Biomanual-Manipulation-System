#ifndef MOA_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
#define MOA_DRIVER_HARDWARE_INTERFACE_H_INCLUDED

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <pluginlib/class_list_macros.hpp>
// #include <std_msgs/Float64.h>

namespace moa_driver {
class MoaHWInterface : public hardware_interface::RobotHW {
   public:
    MoaHWInterface();
    ~MoaHWInterface();

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    void read();

    void write();

    const ros::Time get_time();

    const ros::Duration get_period();

   protected:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    double *cmd_;
    double *pos_;
    double *vel_;
    double *eff_;

    ros::Time last_read_;
    std::vector<std::string> joint_names_;
};
}  // namespace moa_driver
#endif  // ifndef MOA_DRIVER_HARDWARE_INTERFACE_H_INCLUDED