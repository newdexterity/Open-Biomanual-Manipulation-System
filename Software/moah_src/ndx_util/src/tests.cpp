#include <ndx_util/tf_util.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>

bool stop_thread = false;

void transform_publisher() {
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;

    ts.header.frame_id = "world";
    ts.child_frame_id = "test_frame";

    ts.transform.rotation.w = 1;

    ts.transform.translation.x = 0;
    ts.transform.translation.y = 0;
    ts.transform.translation.z = 0;
    ros::Rate rate(1);
    while (!stop_thread) {
        ts.header.stamp = ros::Time::now();
        // ts.header.seq += 1;
        // ROS_WARN_STREAM("\t" << ts.transform.translation.z);
        ROS_WARN_STREAM(ts);
        br.sendTransform(ts);
        ts.transform.translation.z += 10.0;
        ROS_WARN_STREAM("Publishing data at: " << ts.header.stamp);
        rate.sleep();
    }
}

// Testing TF interpolation...
int main(int argc, char** argv) {
    using namespace std::chrono_literals;
    ros::init(argc, argv, "tf_util_test");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Publishing the transform that's going to be used for testing
    std::thread t(transform_publisher);
    // Waiting buffer to fill...
    std::this_thread::sleep_for(2500ms);

    // We should have 2 tfs in the buffer...
    try {
        ros::Time now = ros::Time::now();

        // geometry_msgs::PoseStamped ret =
        //     tf_util::get_pose_msg(tfBuffer, "world", "test_frame", now, 10.0);
        geometry_msgs::TransformStamped ret =
            tfBuffer.lookupTransform("world", "test_frame", now, ros::Duration(10.0));

        ROS_INFO_STREAM("Requested message at time: " << now);
        ROS_INFO_STREAM("Got message at time: " << ret.header.stamp);
        // ROS_INFO_STREAM("Z: " << ret.transform.translation.z);
        ROS_INFO_STREAM("Msg: " << ret);
    } catch (tf2::TransformException& e) {
        ROS_ERROR_STREAM(e.what());
    }
    stop_thread = true;
    t.join();
    return 0;
}