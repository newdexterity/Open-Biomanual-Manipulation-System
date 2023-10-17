#include <moa_driver/hardware_interface.hpp>

namespace moa_driver {

MoaHWInterface::MoaHWInterface() {}
MoaHWInterface::~MoaHWInterface() {
    delete cmd_;
    delete pos_;
    delete vel_;
    delete eff_;
}

bool MoaHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    // Names of the joints. Usually, this is given in the controller config file.
    if (!robot_hw_nh.getParam("joints", joint_names_)) {
        ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                           << " on the parameter server.");
        throw std::runtime_error(
            "Cannot find required parameter "
            "'joints' on the parameter server.");
        return false;
    }
    cmd_ = new double[joint_names_.size()];
    pos_ = new double[joint_names_.size()];
    vel_ = new double[joint_names_.size()];
    eff_ = new double[joint_names_.size()];
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        cmd_[i] = 0.0;
        pos_[i] = 0.0;
        vel_[i] = 0.0;
        eff_[i] = 0.0;

        // connect and register the joint state interface
        jnt_state_interface.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[i], &(pos_[i]), &(vel_[i]), &(eff_[i])));

        // connect and register the joint position interface
        jnt_pos_interface.registerHandle(hardware_interface::JointHandle(
            jnt_state_interface.getHandle(joint_names_[i]), &(cmd_[i])));
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);

    last_read_ = ros::Time::now();
    return true;
}

void MoaHWInterface::read() {}

void MoaHWInterface::write() {
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        pos_[i] = 0.8 * pos_[i] + 0.2 * cmd_[i];
    }
}

const ros::Time MoaHWInterface::get_time() { return ros::Time::now(); }

const ros::Duration MoaHWInterface::get_period() {
    const ros::Duration ret = ros::Time::now() - last_read_;
    last_read_ = ros::Time::now();
    return ret;
}
};  // namespace moa_driver
PLUGINLIB_EXPORT_CLASS(moa_driver::MoaHWInterface, hardware_interface::RobotHW)