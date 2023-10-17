#ifndef TF_UTILS
#define TF_UTILS

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/console.h>
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"

//------------------------------------------------------------------------------
namespace tf_util {

geometry_msgs::TransformStamped posemsg_to_tmsg(geometry_msgs::PoseStamped pose_msg,
                                                std::string dest_frame);
Eigen::Quaterniond posemsg_to_quaternionvector(geometry_msgs::PoseStamped pose_msg);
Eigen::Vector3d posemsg_to_transvector(geometry_msgs::PoseStamped pose_msg);
Eigen::Matrix4d posemsg_to_array(geometry_msgs::PoseStamped pose_msg);
geometry_msgs::PoseStamped tmsg_to_posemsg(geometry_msgs::TransformStamped transform_msg);
geometry_msgs::PoseStamped tarray_to_posemsg(Eigen::Matrix4d transform_array, std::string frame_id);
Eigen::Quaterniond tmsg_to_quaternionvector(geometry_msgs::TransformStamped transform_msg);
Eigen::Vector3d tmsg_to_translation(geometry_msgs::TransformStamped transform_msg);
Eigen::Matrix4d tmsg_to_rotation_matrix(geometry_msgs::TransformStamped transform_msg);
Eigen::Matrix4d tmsg_to_array(geometry_msgs::TransformStamped transform_msg);
geometry_msgs::TransformStamped tarray_to_tmsg(Eigen::Matrix4d transform_array,
                                               std::string frame_id, std::string child_frame_id);
Eigen::Vector3d tmsg_to_transvector(geometry_msgs::TransformStamped transform_msg);

// Eigen::Matrix4d align_tf_rotation_sym(Eigen::Matrix4d input_tf, Eigen::Matrix4d target_tf,
//                                       int symmetery_axis, bool allow_flipping);
Eigen::Quaterniond quaternion_conj(Eigen::Quaterniond q);
Eigen::Quaterniond quaternion_inv(Eigen::Quaterniond q);
Eigen::Quaterniond quaternion_mult(Eigen::Quaterniond q1, Eigen::Quaterniond q0);
Eigen::Quaterniond quaternion_diff(Eigen::Quaterniond q1, Eigen::Quaterniond q0);
double quaternion_angle(Eigen::Quaterniond q);
double quaternion_diff_angle(Eigen::Quaterniond q1, Eigen::Quaterniond q0);

geometry_msgs::TransformStamped get_transform_msg(tf2_ros::Buffer &tf_buffer,
                                                  std::string source_frame, std::string dest_frame,
                                                  ros::Time time, double max_wait_time,
                                                  bool is_active);
geometry_msgs::PoseStamped get_pose_msg(tf2_ros::Buffer &tf_buffer, std::string reference_frame,
                                        std::string frame, ros::Time time, double max_wait_time);

Eigen::Matrix4d get_transform_array(tf2_ros::Buffer &tf_buffer, std::string source_frame,
                                    std::string dest_frame, ros::Time time, double max_wait_time,
                                    bool is_active);
Eigen::Matrix4d get_pose_array(tf2_ros::Buffer &tf_buffer, std::string reference_frame,
                               std::string frame, ros::Time time, double max_wait_time);

}  // namespace tf_util
#endif  // FEATURE_EXTRACTION
