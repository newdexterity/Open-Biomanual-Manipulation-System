#include <assert.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <nlopt.hpp>
#include <numeric>
#include <vector>

// URDF
#include <urdf/model.h>
#include <urdf_model/joint.h>

// HW interface
#include <hardware_interface/joint_command_interface.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// Misc
#include <algorithm>

// Global vars
std::unique_ptr<KDL::ChainFkSolverPos_recursive> ee_fksolver, elbow_fksolver;
KDL::JntArray full_chain_jp, elbow_chain_jp;
std::vector<double> target_pos(3), target_pos_elbow(3);
size_t elbow_chain_l = 4;
double elbow_factor = 0.4;

double sqrd_norm(std::vector<double> tp, KDL::Frame cp) {
    return pow(tp[0] - cp.p[0], 2) + pow(tp[1] - cp.p[1], 2) + pow(tp[2] - cp.p[2], 2);
}

double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    bool kinematics_status;
    KDL::Frame ee_pose, elbow_pose;

    for (size_t i = 0; i < x.size(); ++i) {
        full_chain_jp(i) = x[i];
    }
    for (size_t i = 0; i < elbow_chain_l; ++i) {
        elbow_chain_jp(i) = x[i];
    }
    ee_fksolver->JntToCart(full_chain_jp, ee_pose);
    elbow_fksolver->JntToCart(elbow_chain_jp, elbow_pose);
    // ROS_INFO_STREAM(elbow_pose.p[0] << ", " << elbow_pose.p[1] << ", " << elbow_pose.p[2]);
    // if (kinematics_status < 0) {
    //     ROS_ERROR_STREAM("KINEMATIC ERROR!");
    //     return HUGE_VAL;
    // }
    return (1.0 - elbow_factor) * sqrd_norm(target_pos, ee_pose) +
           elbow_factor * sqrd_norm(target_pos_elbow, elbow_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kdl_test");

    // ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    // ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

    ros::NodeHandle node;

    KDL::Tree robot_tree;
    urdf::Model robot_model;
    KDL::Chain ee_chain, elbow_chain;
    std::vector<std::string> joint_names;
    std::vector<hardware_interface::JointHandle> joint_handles;

    std::string robot_description;
    node.param("robot_description", robot_description, std::string());

    // Build a kinematic chain of the robot
    if (!robot_model.initString(robot_description)) {
        ROS_ERROR("Failed to parse urdf model from 'robot_description'");
        return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {
        const std::string error = "Failed to parse KDL tree from urdf model";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    if (!robot_tree.getChain("map", "human/right_arm_frame7", ee_chain)) {
        const std::string error =
            "Failed to parse robot chain from urdf model. "
            "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!robot_tree.getChain("map", "human/right_arm_frame3", elbow_chain)) {
        const std::string error =
            "Failed to parse robot chain from urdf model. "
            "Are you sure that both your 'robot_base_link' and 'elbow_link' exist?";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    if (!node.getParam("/moa_right/moa_driver_node/joints", joint_names)) {
        const std::string error =
            ""
            "Failed to load " +
            node.getNamespace() + "/joints" + " from parameter server";
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
                joint_names[i] + " does not appear in /robot_description";
            ROS_ERROR_STREAM(error);
            throw std::runtime_error(error);
        }
        if (robot_model.getJoint(joint_names[i])->type == urdf::Joint::CONTINUOUS) {
            upper_pos_limits(i) = std::nan("0");
            lower_pos_limits(i) = std::nan("0");
        } else {
            // Non-existent urdf limits are zero initialized
            ROS_INFO_STREAM(joint_names[i] << robot_model.getJoint(joint_names[i])->limits->upper
                                           << ", "
                                           << robot_model.getJoint(joint_names[i])->limits->lower);
            upper_pos_limits(i) = robot_model.getJoint(joint_names[i])->limits->upper;
            lower_pos_limits(i) = robot_model.getJoint(joint_names[i])->limits->lower;
        }
    }

    // Get the joint handles to use in the control loop
    // for (size_t i = 0; i < joint_names.size(); ++i) {
    //     joint_handles.push_back(hw->getHandle(joint_names[i]));
    // }

    // Create solver based on kinematic chain
    ee_fksolver.reset(new KDL::ChainFkSolverPos_recursive(ee_chain));
    elbow_fksolver.reset(new KDL::ChainFkSolverPos_recursive(elbow_chain));

    // Create joint array
    unsigned int nj = ee_chain.getNrOfJoints();
    unsigned int elbow_nj = elbow_chain.getNrOfJoints();
    full_chain_jp = KDL::JntArray(nj);
    elbow_chain_jp = KDL::JntArray(elbow_nj);

    // Instantiating optimizer
    nlopt::opt opt(nlopt::LN_COBYLA, nj);
    std::vector<double> x(nj);

    // Defining upper and lower bounds
    std::vector<double> lb(nj), ub(nj);
    for (unsigned int i = 0; i < nj; i++) {
        lb[i] = lower_pos_limits(i);
        ub[i] = upper_pos_limits(i);
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // Defining opt target
    opt.set_min_objective(myvfunc, NULL);

    // Defining tolerance for solution
    opt.set_xtol_rel(1e-4);

    double minf;
    // Get joint angles
    sensor_msgs::JointStateConstPtr js_msg;
    geometry_msgs::PoseStampedConstPtr ps_msg, elbow_ps_msg;
    ros::Publisher cmd_pub = node.advertise<std_msgs::Float64MultiArray>(
        "/moa_right/joint_group_position_controller/command", 1000);
    ros::Rate rate(200);

    while (ros::ok()) {
        // This should be done using a hardware interface...
        js_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", node);
        ps_msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_pose", node);
        elbow_ps_msg =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_pose_elbow", node);

        target_pos[0] = ps_msg->pose.position.x;
        target_pos[1] = ps_msg->pose.position.y;
        target_pos[2] = ps_msg->pose.position.z;

        target_pos_elbow[0] = elbow_ps_msg->pose.position.x;
        target_pos_elbow[1] = elbow_ps_msg->pose.position.y;
        target_pos_elbow[2] = elbow_ps_msg->pose.position.z;

        // js_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/moa_toy/joint_states",
        // node);
        for (unsigned int i = 0; i < nj; i++) {
            auto it = std::find(js_msg->name.begin(), js_msg->name.end(), joint_names[i]);
            if (it == js_msg->name.end()) {
                const std::string error =
                    ""
                    "Joint " +
                    joint_names[i] + " not found in joint state message";
                ROS_ERROR_STREAM(error);
                throw std::runtime_error(error);
            }
            int index = it - js_msg->name.begin();
            x[i] = js_msg->position[index];
        }

        try {
            nlopt::result result = opt.optimize(x, minf);
            std_msgs::Float64MultiArray out_msg;
            for (size_t i = 0; i < x.size(); ++i) {
                out_msg.data.push_back(x[i]);
            }
            // for (size_t i = 0; i < 3; ++i) {
            //     out_msg.data.push_back(0);
            // }
            cmd_pub.publish(out_msg);
        } catch (std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        // rate.sleep();
    }

    return 0;
}