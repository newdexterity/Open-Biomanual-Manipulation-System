#ifndef TF_UTILS
#define TF_UTILS


#include <iostream>
#include <string>

#include <string.h>
#include <vector>
#include <math.h>
#include <cmath>
#include <stdexcept>

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry> 
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


//------------------------------------------------------------------------------


Eigen::Quaterniond posemsg_to_quaternionvector(geometry_msgs::PoseStamped pose_msg);
Eigen::Vector3d posemsg_to_transvector(geometry_msgs::PoseStamped pose_msg);
Eigen::Matrix4d posemsg_to_array(geometry_msgs::PoseStamped pose_msg);
geometry_msgs::PoseStamped tmsg_to_posemsg(geometry_msgs::TransformStamped transform_msg);
geometry_msgs::PoseStamped tarray_to_posemsg(Eigen::Matrix4d transform_array, std::string frame_id);
Eigen::Quaterniond tmsg_to_quaternionvector(geometry_msgs::TransformStamped transform_msg);
Eigen::Vector3d tmsg_to_translation(geometry_msgs::TransformStamped transform_msg);
Eigen::Matrix4d tmsg_to_rotation_matrix(geometry_msgs::TransformStamped transform_msg);
Eigen::Matrix4d tmsg_to_array(geometry_msgs::TransformStamped transform_msg);
geometry_msgs::TransformStamped tarray_to_tmsg(Eigen::Matrix4d transform_array, std::string frame_id, std::string child_frame_id);

Eigen::Matrix4d align_tf_rotation_sym(Eigen::Matrix4d input_tf, Eigen::Matrix4d target_tf, int symmetery_axis, bool allow_flipping);
Eigen::Quaterniond quaternion_conj(Eigen::Quaterniond q);
Eigen::Quaterniond quaternion_inv(Eigen::Quaterniond q);
Eigen::Quaterniond quaternion_mult(Eigen::Quaterniond q1, Eigen::Quaterniond q0);
Eigen::Quaterniond quaternion_diff(Eigen::Quaterniond q1, Eigen::Quaterniond q0);
double quaternion_angle(Eigen::Quaterniond q);
double quaternion_diff_angle(Eigen::Quaterniond q1, Eigen::Quaterniond q0);

std::tuple<bool, Eigen::Matrix4d, Eigen::Vector3d, Eigen::Quaterniond, geometry_msgs::TransformStamped> get_transform(tf2_ros::Buffer Buffer, std::string source_frame, std::string dest_frame, ros::Time time, float max_wait_time, bool is_active, bool supress_error_msgs);
std::tuple<bool, Eigen::Matrix4d, Eigen::Vector3d, Eigen::Quaterniond, geometry_msgs::TransformStamped> get_pose(tf2_ros::Buffer tf_buffer, std::string reference_frame, std::string frame, ros::Time time, float max_wait_time, bool supress_error_msgs);





#endif // FEATURE_EXTRACTION










