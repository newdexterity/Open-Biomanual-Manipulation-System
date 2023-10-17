#include <ndx_util/tf_util.h>

Eigen::Quaterniond tf_util::posemsg_to_quaternionvector(geometry_msgs::PoseStamped pose_msg) {
    Eigen::Quaterniond q(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                         pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);

    return q;
}

Eigen::Vector3d tf_util::posemsg_to_transvector(geometry_msgs::PoseStamped pose_msg) {
    Eigen::Vector3d q(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

    return q;
}

Eigen::Matrix4d tf_util::posemsg_to_array(geometry_msgs::PoseStamped pose_msg) {
    Eigen::Matrix3d mat3 = tf_util::posemsg_to_quaternionvector(pose_msg).toRotationMatrix();
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0, 0, 3, 3) = mat3;
    mat4.block(0, 3, 3, 1) = tf_util::posemsg_to_transvector(pose_msg);

    return mat4;
}

geometry_msgs::PoseStamped tf_util::tarray_to_posemsg(Eigen::Matrix4d transform_array,
                                                      std::string frame_id) {
    geometry_msgs::PoseStamped p;

    p.header.frame_id = frame_id;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = transform_array(0, 3);
    p.pose.position.y = transform_array(1, 3);
    p.pose.position.z = transform_array(2, 3);

    Eigen::Matrix3d mat3 = transform_array.block(0, 0, 3, 3);
    Eigen::Quaterniond q(mat3);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    return p;
}

geometry_msgs::PoseStamped tf_util::tmsg_to_posemsg(geometry_msgs::TransformStamped transform_msg) {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = transform_msg.child_frame_id;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = transform_msg.transform.translation.x;
    p.pose.position.y = transform_msg.transform.translation.y;
    p.pose.position.z = transform_msg.transform.translation.z;
    p.pose.orientation = transform_msg.transform.rotation;
    return p;
}

Eigen::Quaterniond tf_util::tmsg_to_quaternionvector(
    geometry_msgs::TransformStamped transform_msg) {
    Eigen::Quaterniond q(transform_msg.transform.rotation.w, transform_msg.transform.rotation.x,
                         transform_msg.transform.rotation.y, transform_msg.transform.rotation.z);

    return q;
}

Eigen::Vector3d tf_util::tmsg_to_transvector(geometry_msgs::TransformStamped transform_msg) {
    Eigen::Vector3d q(transform_msg.transform.translation.x, transform_msg.transform.translation.y,
                      transform_msg.transform.translation.z);

    return q;
}

Eigen::Matrix4d tf_util::tmsg_to_rotation_matrix(geometry_msgs::TransformStamped transform_msg) {
    Eigen::Matrix3d mat3 = tf_util::tmsg_to_quaternionvector(transform_msg).toRotationMatrix();
    Eigen::Matrix4d mat4 = Eigen::Matrix4d::Identity();
    mat4.block(0, 0, 3, 3) = mat3;

    return mat4;
}

Eigen::Matrix4d tf_util::tmsg_to_array(geometry_msgs::TransformStamped transform_msg) {
    Eigen::Matrix4d mat4 = tf_util::tmsg_to_rotation_matrix(transform_msg);
    mat4.block(0, 3, 3, 1) = tf_util::tmsg_to_transvector(transform_msg);

    return mat4;
}

geometry_msgs::TransformStamped tf_util::tarray_to_tmsg(Eigen::Matrix4d transform_array,
                                                        std::string frame_id,
                                                        std::string child_frame_id) {
    geometry_msgs::TransformStamped t;
    t.header.frame_id = frame_id;
    t.header.stamp = ros::Time::now();
    t.child_frame_id = child_frame_id;
    t.transform.translation.x = transform_array(0, 3);
    t.transform.translation.y = transform_array(1, 3);
    t.transform.translation.z = transform_array(2, 3);

    Eigen::Matrix3d mat3 = transform_array.block(0, 0, 3, 3);
    Eigen::Quaterniond q(mat3);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    return t;
}

///////////////////// Missing mean_quarternion, mean_transform, align_tf_sym, align_tf_rotation_sym
/// unfinished/////////////////////////////

// Eigen::Matrix4d align_tf_rotation_sym(Eigen::Matrix4d input_tf, Eigen::Matrix4d target_tf,
//                                       int symmetery_axis, bool allow_flipping = true) {
//     // Handle flipping
//     Eigen::Vector3d symvec_input, symvec_target;
//     bool flip = false;
//     if (allow_flipping) {
//         symvec_input = input_tf.block(0, symmetery_axis, 3, 1);
//         symvec_target = target_tf.block(0, symmetery_axis, 3, 1);
//         if (symvec_input.dot(symvec_target) < 0.0) {
//             flip = true;
//         }
//     }

//     // Define axes
// }

Eigen::Quaterniond tf_util::quaternion_conj(Eigen::Quaterniond q) { return q.conjugate(); }

Eigen::Quaterniond tf_util::quaternion_inv(Eigen::Quaterniond q) { return q.inverse(); }

Eigen::Quaterniond tf_util::quaternion_mult(Eigen::Quaterniond q1, Eigen::Quaterniond q0) {
    // double x0 = q0(0);
    // double y0 = q0(1);
    // double z0 = q0(2);
    // double w0 = q0(3);

    // double x1 = q1(0);
    // double y1 = q1(1);
    // double z1 = q1(2);
    // double w1 = q1(3);

    // q_mult(0) = x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0;
    // q_mult(1) = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0;
    // q_mult(2) = x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0;
    // q_mult(3) = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0;

    return q1 * q0;
}

Eigen::Quaterniond tf_util::quaternion_diff(Eigen::Quaterniond q1, Eigen::Quaterniond q0) {
    return q1 * q0.inverse();
}

double tf_util::quaternion_angle(Eigen::Quaterniond q) {
    Eigen::Vector3d q_xyz(q.x(), q.y(), q.z());
    float theta = 2.0 * std::atan(q_xyz.norm() / q.w());
    double result =
        std::fmod((theta + M_PI), (2 * M_PI)) - M_PI;  // WARMING FMOD MAY CAUSE STRANGE BEHAVIOUR
    if (result == -M_PI) {
        result = M_PI;
    }
    return result;
}

double tf_util::quaternion_diff_angle(Eigen::Quaterniond q1, Eigen::Quaterniond q0) {
    double result = quaternion_angle(quaternion_diff(q1, q0));
    return result;
}

geometry_msgs::TransformStamped tf_util::get_transform_msg(tf2_ros::Buffer &tf_buffer,
                                                           std::string source_frame,
                                                           std::string dest_frame, ros::Time time,
                                                           double max_wait_time, bool is_active) {
    std::string t_frame, s_frame;
    geometry_msgs::TransformStamped tf_msg;

    t_frame = dest_frame;
    s_frame = source_frame;
    if (is_active) {
        t_frame = source_frame;
        s_frame = dest_frame;
    }

    return tf_buffer.lookupTransform(t_frame, s_frame, time, ros::Duration(max_wait_time));
}

Eigen::Matrix4d tf_util::get_transform_array(tf2_ros::Buffer &tf_buffer, std::string source_frame,
                                             std::string dest_frame, ros::Time time,
                                             double max_wait_time, bool is_active) {
    geometry_msgs::TransformStamped tf_msg = tf_util::get_transform_msg(
        tf_buffer, source_frame, dest_frame, time, max_wait_time, is_active);
    return tf_util::tmsg_to_array(tf_msg);
}

geometry_msgs::PoseStamped tf_util::get_pose_msg(tf2_ros::Buffer &tf_buffer,
                                                 std::string reference_frame, std::string frame,
                                                 ros::Time time, double max_wait_time) {
    geometry_msgs::TransformStamped tf_msg =
        tf_util::get_transform_msg(tf_buffer, frame, reference_frame, time, max_wait_time, false);

    geometry_msgs::PoseStamped ret;

    ret.header.frame_id = tf_msg.header.frame_id;
    ret.header.seq = tf_msg.header.seq;
    ret.header.stamp = tf_msg.header.stamp;

    ret.pose.orientation.w = tf_msg.transform.rotation.w;
    ret.pose.orientation.x = tf_msg.transform.rotation.x;
    ret.pose.orientation.y = tf_msg.transform.rotation.y;
    ret.pose.orientation.z = tf_msg.transform.rotation.z;

    ret.pose.position.x = tf_msg.transform.translation.x;
    ret.pose.position.y = tf_msg.transform.translation.y;
    ret.pose.position.z = tf_msg.transform.translation.z;

    return ret;
}

Eigen::Matrix4d tf_util::get_pose_array(tf2_ros::Buffer &tf_buffer, std::string reference_frame,
                                        std::string frame, ros::Time time, double max_wait_time) {
    return get_transform_array(tf_buffer, frame, reference_frame, time, max_wait_time, false);
}

geometry_msgs::TransformStamped tf_util::posemsg_to_tmsg(geometry_msgs::PoseStamped pose_msg,
                                                         std::string dest_frame) {
    geometry_msgs::TransformStamped ret;

    ret.child_frame_id = dest_frame;
    ret.header = pose_msg.header;
    ret.transform.rotation = pose_msg.pose.orientation;
    ret.transform.translation.x = pose_msg.pose.position.x;
    ret.transform.translation.y = pose_msg.pose.position.y;
    ret.transform.translation.z = pose_msg.pose.position.z;

    return ret;
}