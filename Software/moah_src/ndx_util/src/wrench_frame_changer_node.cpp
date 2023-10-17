// #include <ndx_util/tf_util.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

class WrenchFrameChanger {
   public:
    WrenchFrameChanger(ros::NodeHandle &nh, ros::NodeHandle &p_nh)
        : tf2_(buffer_),
          target_frame_("teste"),
          tf2_filter_(wrench_sub_, buffer_, target_frame_, 10, 0) {
        std::string input_topic;
        std::string output_topic;

        if (!p_nh.getParam("target_frame", target_frame_)) {
            ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("target_frame")
                                                   << " not given.");
            return;
        }
        if (!p_nh.getParam("input_topic", input_topic)) {
            ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("input_topic")
                                                   << " not given.");
            return;
        }
        if (!p_nh.getParam("output_topic", output_topic)) {
            ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("output_topic")
                                                   << " not given.");
            return;
        }
        draw_force_vector_ = p_nh.param<bool>("draw_force_vector", true);
        m_id_ = p_nh.param<int>("marker_id", 0);

        wrench_sub_.subscribe(nh, input_topic, 10);
        tf2_filter_.registerCallback(boost::bind(&WrenchFrameChanger::wrench_cb, this, _1));
        tf2_filter_.setTargetFrame(target_frame_);
        ws_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(output_topic, 1);
        m_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void wrench_cb(const geometry_msgs::WrenchStampedConstPtr &msg) {
        geometry_msgs::WrenchStamped transformed_ws;
        try {
            buffer_.transform(*msg, transformed_ws, target_frame_);
            ws_pub_.publish(transformed_ws);
            if (draw_force_vector_) {
                marker_.header.frame_id = target_frame_;
                marker_.header.stamp = ros::Time::now();
                marker_.ns = "";
                marker_.type = visualization_msgs::Marker::ARROW;
                marker_.id = m_id_;
                marker_.action = visualization_msgs::Marker::ADD;

                marker_.points.clear();
                geometry_msgs::Point start;
                start.x = 0;
                start.y = 0;
                start.z = 0;
                geometry_msgs::Point end;
                end.x = transformed_ws.wrench.force.x * 0.1;
                end.y = transformed_ws.wrench.force.y * 0.1;
                end.z = transformed_ws.wrench.force.z * 0.1;
                marker_.points.push_back(start);
                marker_.points.push_back(end);

                marker_.scale.x = 0.01;
                marker_.scale.y = 0.05;
                marker_.scale.z = 0;

                marker_.color.a = 1.0;
                marker_.color.r = 255.0 / 255.0;
                marker_.color.g = 235.0 / 255.0;
                marker_.color.b = 59.0 / 255.0;

                marker_.pose.position.x = 0;
                marker_.pose.position.y = 0;
                marker_.pose.position.z = 0;
                marker_.pose.orientation.x = 0.0;
                marker_.pose.orientation.y = 0.0;
                marker_.pose.orientation.z = 0.0;
                marker_.pose.orientation.w = 1.0;

                marker_.lifetime = ros::Duration(1);
                m_pub_.publish(marker_);
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
        }
    }

   private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub_;
    tf2_ros::MessageFilter<geometry_msgs::WrenchStamped> tf2_filter_;
    ros::Publisher ws_pub_;
    bool draw_force_vector_;
    visualization_msgs::Marker marker_;
    ros::Publisher m_pub_;
    int m_id_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wrench_frame_changer_node");

    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    WrenchFrameChanger wfc(nh, p_nh);
    ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench", 1000);
    ros::spin();
    return 0;
}