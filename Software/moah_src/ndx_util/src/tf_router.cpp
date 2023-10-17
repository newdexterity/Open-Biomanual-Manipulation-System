#include <ndx_util/tf_util.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_router");

    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string tf_name;
    std::string output_topic;
    std::string tf_parent = p_nh.param<std::string>("tf_parent", "world");
    if (!p_nh.getParam("tf_name", tf_name)) {
        ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("tf_name") << " not given.");
        return -1;
    }
    if (!p_nh.getParam("output_topic", output_topic)) {
        ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("output_topic")
                                               << " not given.");
        return -1;
    }
    int pool_rate = 1000;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::Publisher ts_pub = nh.advertise<geometry_msgs::TransformStamped>(output_topic, 1);
    ros::Publisher ps_pub = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 1);

    // We use a high rate instead of none so we don't bog down the cpu
    ros::Rate rate(pool_rate);
    ros::Time previous_stamp = ros::Time::now();

    while (ros::ok()) {
        try {
            geometry_msgs::PoseStamped pmsg =
                tf_util::get_pose_msg(tfBuffer, tf_parent, tf_name, ros::Time(0), 0);
            if (pmsg.header.stamp != previous_stamp) {
                previous_stamp = pmsg.header.stamp;
            } else {
                rate.sleep();
                continue;
            }
            ps_pub.publish(pmsg);
            // ts_pub.publish(tf_util::posemsg_to_tmsg(pmsg, tf_name));
        } catch (tf2::TransformException& e) {
            // We ignore the exceptions
        }

        rate.sleep();
    }
    return 0;
}