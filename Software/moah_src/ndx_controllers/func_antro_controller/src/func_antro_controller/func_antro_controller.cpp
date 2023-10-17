#include <controller_interface/controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.hpp>
#include <vector>

// KDL
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>
#include <urdf_model/joint.h>

// NLOPT
#include <nlopt.hpp>

// MISC
#include <string.h>

#include <cmath> /* acos */
#include <iostream>

namespace ndx_controllers {

double sqrd_mag(KDL::Vector& vec) { return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]; }

double get_magnitude(KDL::Frame frame, bool use_angles) {
    KDL::Vector rot_axis;

    double yaw, pitch, roll;
    double sqrd_p_mag = sqrd_mag(frame.p);
    double error_angle = frame.M.GetRotAngle(rot_axis);
    rot_axis = rot_axis * error_angle;
    if (!use_angles) {
        return sqrd_p_mag;
    }
    return sqrd_p_mag;
}

double obj_wrapper(const std::vector<double>& x, std::vector<double>& grad, void* data);

class FuncAntroController
    : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
   public:
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n) {
        // Get joint names from the parameter server

        std::vector<std::string> joint_names;
        if (!n.getParam("joints", joint_names)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/joints" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        // Get joint handles to be used in the control loop
        for (auto name : joint_names) {
            joint_handles_.push_back(hw->getHandle(name));
        }

        KDL::Tree robot_tree;
        urdf::Model robot_model;
        // KDL::Chain ee_chain, elbow_chain;

        std::string robot_description;
        std::string target_pose_topic;
        std::string target_elbow_pose_topic;
        std::string palm_pose_topic;
        // Get controller specific configuration
        if (!ros::param::search("robot_description", robot_description)) {
            ROS_ERROR_STREAM(
                "Searched enclosing namespaces for robot_description but nothing found");
            return false;
        }

        if (!n.getParam(robot_description, robot_description)) {
            ROS_ERROR_STREAM("Failed to load " << robot_description << " from parameter server");
            return false;
        }

        if (!robot_model.initString(robot_description)) {
            ROS_ERROR("Failed to parse urdf model from 'robot_description'");
            return false;
        }

        if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {
            const std::string error = "Failed to parse KDL tree from urdf model";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("base_link", base_link_name_)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/base_link" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("ee_link", ee_link_name_)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/ee_link" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("elbow_link", elbow_link_name_)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/elbow_link" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("arm_type", arm_type_)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/arm_type" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("target_pose_topic", target_pose_topic)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/target_pose_topic" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("target_elbow_pose_topic", target_elbow_pose_topic)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/target_elbow_pose_topic" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("palm_pose_topic", palm_pose_topic)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/palm_pose_topic" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!n.getParam("elbow_pos_weight", elbow_pos_weight_)) {
            const std::string error =
                ""
                "Failed to load " +
                n.getNamespace() + "/elbow_pos_weight" + " from parameter server";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        ee_sub_ =
            n.subscribe(target_pose_topic, 100, &FuncAntroController::targetEEFrameCallback, this);
        elbow_sub_ = n.subscribe(target_elbow_pose_topic, 100,
                                 &FuncAntroController::targetElbowFrameCallback, this);
        palm_sub_ =
            n.subscribe(palm_pose_topic, 100, &FuncAntroController::palmFrameCallback, this);

        if (!robot_tree.getChain(base_link_name_, ee_link_name_, ee_chain_)) {
            const std::string error =
                "Failed to parse robot chain from urdf model. "
                "Are you sure that both your 'base_link' and 'ee_link' exist?";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        if (!robot_tree.getChain(base_link_name_, elbow_link_name_, elbow_chain_)) {
            const std::string error =
                "Failed to parse robot chain from urdf model. "
                "Are you sure that both your 'base_link' and 'elbow_link' exist?";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }

        // Parse joint limits
        KDL::JntArray upper_pos_limits(joint_names.size());
        KDL::JntArray lower_pos_limits(joint_names.size());
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (!robot_model.getJoint(joint_names[i])) {
                const std::string error =
                    ""
                    "Joint " +
                    joint_names[i] + " does not appear in " + n.getNamespace() +
                    "/robot_description";
                ROS_ERROR_STREAM(error);
                throw std::runtime_error(error);
            }
            if (robot_model.getJoint(joint_names[i])->type == urdf::Joint::CONTINUOUS) {
                upper_pos_limits(i) = std::nan("0");
                lower_pos_limits(i) = std::nan("0");
            } else {
                // Non-existent urdf limits are zero initialized
                ROS_INFO_STREAM(joint_names[i]
                                << robot_model.getJoint(joint_names[i])->limits->upper << ", "
                                << robot_model.getJoint(joint_names[i])->limits->lower);
                upper_pos_limits(i) = robot_model.getJoint(joint_names[i])->limits->upper;
                lower_pos_limits(i) = robot_model.getJoint(joint_names[i])->limits->lower;
            }
        }

        // Create solver based on kinematic chain
        ee_fksolver_.reset(new KDL::ChainFkSolverPos_recursive(ee_chain_));
        elbow_fksolver_.reset(new KDL::ChainFkSolverPos_recursive(elbow_chain_));

        KDL::Frame ee_pose;
        KDL::JntArray test_array = KDL::JntArray(4);

        ee_fksolver_->JntToCart(test_array, ee_pose);

        // Create joint array
        unsigned int nj = ee_chain_.getNrOfJoints();
        unsigned int elbow_nj = elbow_chain_.getNrOfJoints();
        ee_chain_jp_ = KDL::JntArray(nj);
        elbow_chain_jp_ = KDL::JntArray(elbow_nj);

        elbow_chain_l_ = elbow_nj;
        // Instantiating optimizer
        opt_.reset(new nlopt::opt(nlopt::LN_COBYLA, nj));
        joint_positions_.resize(nj);

        // ROS_WARN_STREAM("nj: " << nj);
        // ROS_WARN_STREAM("elbow_nj: " << elbow_nj);
        // ROS_WARN_STREAM("joint_positions_.size: " << joint_positions_.size());

        // Defining upper and lower bounds
        // std::vector<double> lb(nj), ub(nj);
        lb_.resize(nj);
        ub_.resize(nj);
        for (unsigned int i = 0; i < nj; i++) {
            lb_[i] = lower_pos_limits(i);
            ub_[i] = upper_pos_limits(i);
        }
        opt_->set_lower_bounds(lb_);
        opt_->set_upper_bounds(ub_);

        // Defining opt target. We use obj_wrapper instead of the class function directly
        // because NLOPT requires a global function pointer.
        opt_->set_min_objective(ndx_controllers::obj_wrapper, this);

        // Defining tolerance for solution. Todo: make this a parameter
        opt_->set_xtol_rel(1e-7);

        received_t_ee_pose_ = false;
        received_t_elbow_pose_ = false;
        received_palm_pose_ = false;

        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period) {
        double minf;

        for (auto i = 0; i < joint_handles_.size(); i++) {
            joint_positions_[i] = joint_handles_[i].getPosition();
        }

        if (!received_t_ee_pose_ || !received_t_elbow_pose_ || !received_palm_pose_) {
            return;
        }

        try {
            nlopt::result result = opt_->optimize(joint_positions_, minf);

            double wrist_yaw, wrist_pitch, wrist_roll;
            double palm_yaw, palm_pitch, palm_roll;

            double rad2deg = 180.0 / M_PI;
            KDL::Frame relative_ee_pose;
            KDL::Frame relative_palm_pose;

            relative_ee_pose = elbow_t_pose_.Inverse() * ee_t_pose_;
            relative_ee_pose.M.GetRPY(wrist_roll, wrist_pitch, wrist_yaw);

            relative_palm_pose = ee_t_pose_.Inverse() * palm_pose_;
            relative_palm_pose.M.GetRPY(palm_roll, palm_pitch, palm_yaw);

            if (arm_type_ == "right") {
                joint_positions_[7] = wrist_roll;
                // joint_positions_[7] = -wrist_roll - 3.1415 / 2.0;
                joint_positions_[5] = -palm_yaw;
                joint_positions_[6] = -palm_pitch;
            } else {
                joint_positions_[7] = wrist_roll;
                // joint_positions_[7] = wrist_roll - 3.1415 / 2.0;
                joint_positions_[5] = -palm_yaw;
                joint_positions_[6] = -palm_pitch;
            }

            // ROS_INFO_STREAM(relative_palm_pose.p[0] << " " << relative_palm_pose.p[1] << " "
            //                                         << relative_palm_pose.p[2]);
            // ROS_INFO_STREAM(wrist_yaw * rad2deg << " " << wrist_pitch * rad2deg << " "
            // << wrist_roll * rad2deg);

            
            for (auto i = 0; i < joint_handles_.size(); i++) {
                joint_positions_[i] = joint_positions_[i] > ub_[i] ? ub_[i] : joint_positions_[i];
                joint_positions_[i] = joint_positions_[i] < lb_[i] ? lb_[i] : joint_positions_[i];
                joint_handles_[i].setCommand(joint_positions_[i]);
            }
        } catch (std::exception& e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
    }

    void starting(const ros::Time& time) {}
    void stopping(const ros::Time& time) {}

    void targetEEFrameCallback(const geometry_msgs::PoseStamped& target) {
        if (target.header.frame_id != base_link_name_) {
            ROS_ERROR_STREAM_THROTTLE(3, "Got target pose in wrong reference frame. Expected: "
                                             << base_link_name_ << " but got "
                                             << target.header.frame_id);
            return;
        }

        received_t_ee_pose_ = true;
        ee_t_pose_ = KDL::Frame(
            KDL::Rotation::Quaternion(target.pose.orientation.x, target.pose.orientation.y,
                                      target.pose.orientation.z, target.pose.orientation.w),
            KDL::Vector(target.pose.position.x, target.pose.position.y, target.pose.position.z));
    }

    void targetElbowFrameCallback(const geometry_msgs::PoseStamped& target) {
        if (target.header.frame_id != base_link_name_) {
            ROS_ERROR_STREAM_THROTTLE(3, "Got target pose in wrong reference frame. Expected: "
                                             << base_link_name_ << " but got "
                                             << target.header.frame_id);
            return;
        }
        received_t_elbow_pose_ = true;
        elbow_t_pose_ = KDL::Frame(
            KDL::Rotation::Quaternion(target.pose.orientation.x, target.pose.orientation.y,
                                      target.pose.orientation.z, target.pose.orientation.w),
            KDL::Vector(target.pose.position.x, target.pose.position.y, target.pose.position.z));

    }

    void palmFrameCallback(const geometry_msgs::PoseStamped& target) {
        if (target.header.frame_id != base_link_name_) {
            ROS_ERROR_STREAM_THROTTLE(3, "Got target pose in wrong reference frame. Expected: "
                                             << base_link_name_ << " but got "
                                             << target.header.frame_id);
            return;
        }

        received_palm_pose_ = true;
        palm_pose_ = KDL::Frame(
            KDL::Rotation::Quaternion(target.pose.orientation.x, target.pose.orientation.y,
                                      target.pose.orientation.z, target.pose.orientation.w),
            KDL::Vector(target.pose.position.x, target.pose.position.y, target.pose.position.z));
    }

    double eval_sol(const std::vector<double>& x, std::vector<double>& grad, void* my_func_data) {
        bool kinematics_status;
        KDL::Frame ee_pose, elbow_pose, ee_error, elbow_error;

        for (size_t i = 0; i < x.size(); ++i) {
            ee_chain_jp_(i) = x[i];
        }
        for (size_t i = 0; i < elbow_chain_l_; ++i) {
            elbow_chain_jp_(i) = x[i];
        }

        ee_fksolver_->JntToCart(ee_chain_jp_, ee_pose);
        elbow_fksolver_->JntToCart(elbow_chain_jp_, elbow_pose);

        ee_error.M = ee_t_pose_.M * ee_pose.M.Inverse();

        ee_error.p = ee_t_pose_.p - ee_pose.p;

        // We don't care for the elbow's orientation
        elbow_error.M = KDL::Rotation::Identity();
        elbow_error.p = elbow_t_pose_.p - elbow_pose.p;

        return (1.0 - elbow_pos_weight_) * get_magnitude(ee_error, true) +
               elbow_pos_weight_ * get_magnitude(elbow_error, false);
    }

   private:
    KDL::JntArray ee_chain_jp_;
    KDL::JntArray elbow_chain_jp_;
    KDL::Frame ee_t_pose_;
    KDL::Frame elbow_t_pose_;
    KDL::Frame palm_pose_;
    size_t elbow_chain_l_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::string base_link_name_;
    std::string ee_link_name_;
    std::string elbow_link_name_;
    std::string arm_type_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> ee_fksolver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> elbow_fksolver_;
    std::unique_ptr<nlopt::opt> opt_;
    double elbow_pos_weight_;
    std::vector<double> joint_positions_;
    std::vector<double> lb_;
    std::vector<double> ub_;
    ros::Subscriber ee_sub_;
    ros::Subscriber elbow_sub_;
    ros::Subscriber palm_sub_;
    KDL::Chain ee_chain_;
    KDL::Chain elbow_chain_;
    bool received_t_ee_pose_;
    bool received_t_elbow_pose_;
    bool received_palm_pose_;
};

double obj_wrapper(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    FuncAntroController* obj = static_cast<FuncAntroController*>(data);
    return obj->eval_sol(x, grad, data);
}

PLUGINLIB_EXPORT_CLASS(ndx_controllers::FuncAntroController, controller_interface::ControllerBase)
}  // namespace ndx_controllers