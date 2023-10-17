#include <ndx_util/tf_util.h>
#include <ros/ros.h>
#include <signal.h>
#include <visualization_msgs/Marker.h>

class MeshMarkerPub {
   public:
    MeshMarkerPub() {}

    void init(ros::NodeHandle &nh, ros::NodeHandle &p_nh) {
        std::string mesh_url;
        std::string ref_frame;
        std::string output_topic;
        std::string ns;

        double scale;
        double r, g, b, a;
        int id;

        if (!p_nh.getParam("mesh_url", mesh_url)) {
            ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("mesh_url")
                                                   << " not given.");
            return;
        }
        if (!p_nh.getParam("ref_frame", ref_frame)) {
            ROS_ERROR_STREAM("Required parameter " << p_nh.resolveName("ref_frame")
                                                   << " not given.");
            return;
        }
        output_topic = p_nh.param<std::string>("output_topic", "visualization_marker");
        ns = p_nh.param<std::string>("namespace", "");
        scale = p_nh.param<double>("scale", 1.0);
        r = p_nh.param<double>("r", 0.5);
        g = p_nh.param<double>("g", 0.5);
        b = p_nh.param<double>("b", 0.5);
        a = p_nh.param<double>("a", 0.5);
        id = p_nh.param<int>("id", 0);

        publish_mesh_ = true;

        m_pub_ = nh.advertise<visualization_msgs::Marker>(output_topic, 1);

        marker_.header.frame_id = "test";
        marker_.header.stamp = ros::Time::now();
        marker_.ns = ns;
        // marker.type = visualization_msgs::Marker::CUBE;
        marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_.id = id;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.mesh_resource = mesh_url;

        marker_.mesh_use_embedded_materials = false;

        marker_.scale.x = scale;
        marker_.scale.y = scale;
        marker_.scale.z = scale;

        marker_.color.r = r;
        marker_.color.g = g;
        marker_.color.b = b;
        marker_.color.a = a;

        marker_.pose.position.x = 0;
        marker_.pose.position.y = 0;
        marker_.pose.position.z = 0;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;

        marker_.lifetime = ros::Duration();
    }

    void run() {
        ros::Rate rate(1);
        ROS_INFO_STREAM("Publishing marker. Terminate node to delete the mesh.");
        while (publish_mesh_) {
            m_pub_.publish(marker_);
            rate.sleep();
        }
    }

    void shutdown() {
        publish_mesh_ = false;
        ROS_INFO_STREAM("Deleting mesh...");
        marker_.action = visualization_msgs::Marker::DELETE;
        m_pub_.publish(marker_);
        ROS_INFO_STREAM("Done! Exiting...");
    }

   private:
    ros::Publisher m_pub_;
    visualization_msgs::Marker marker_;
    bool publish_mesh_;
};

MeshMarkerPub mmp;

void mySigintHandler(int sig) {
    mmp.shutdown();
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker_mesh_node");

    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    mmp.init(nh, p_nh);
    signal(SIGINT, mySigintHandler);
    mmp.run();
    return 0;
}