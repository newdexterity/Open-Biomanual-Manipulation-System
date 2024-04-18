#include "odrive_node.hpp"
#include "odrive_axis.hpp"

bool engage_on_startup;
std::vector<std::string> axis_names_list;
std::vector<int> axis_can_ids_list;
std::vector<std::string> axis_directions_list;
std::vector<odrive::ODriveAxis *> odrive_axises;
ros::Subscriber can_bridge_received_messages_sub;

bool intsAreDistinct(std::vector<int> arr) {
    int n = arr.size();
    std::unordered_set<int> s;
    for (int i = 0; i < n; i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

bool stringsAreDistinct(std::vector<std::string> arr) {
    int n = arr.size();
    std::unordered_set<std::string> s;
    for (int i = 0; i < n; i++) {
        s.insert(arr[i]);
    }
    return (s.size() == arr.size());
}

void canReceivedMessagesCallback(const can_msgs::Frame::ConstPtr& msg) {
    ROS_INFO("I heard from: [%02x]", msg->id);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odrive_can_ros_node");
	ros::NodeHandle node("~");
    node.param<bool>("engage_on_startup", engage_on_startup, DEFAULT_ENGAGE_ON_STARTUP);
    odrive::ODriveAxis *can_node;
    
    if (engage_on_startup) {
        ROS_INFO("Will engage axises on startup");
    } else {
        ROS_INFO("Will not engage axises on startup");
    }
    if (!node.hasParam("axis_names")) {
        ROS_ERROR("Can't run without axis_names parameter");
        return -1;
    }
    if (!node.hasParam("axis_can_ids")) {
        ROS_ERROR("Can't run without axis_can_ids parameter");
        return -1;
    }
    if (!node.hasParam("axis_directions")) {
        ROS_ERROR("Can't run without axis_directions parameter");
        return -1;
    }
    node.getParam("axis_names", axis_names_list);
    node.getParam("axis_can_ids", axis_can_ids_list);
    node.getParam("axis_directions", axis_directions_list);

    if (!(axis_names_list.size() == axis_can_ids_list.size() && axis_can_ids_list.size() ==
        axis_directions_list.size())) {
        ROS_ERROR("axis_names, axis_can_ids and axis_can_directions must be of an equal size");
        return -1;
    }
    if (!stringsAreDistinct(axis_names_list)) {
        ROS_ERROR("axis names must be distinct");
        return -1;
    }
    if (!intsAreDistinct(axis_can_ids_list)) {
        ROS_ERROR("axis CAN ids must be distinct");
        return -1;
    }
    for (int i = 0; i < axis_names_list.size(); i++) {
        ROS_INFO("Adding axis %s with CAN id %d and direction %s", axis_names_list[i].c_str(), 
            axis_can_ids_list[i], axis_directions_list[i].c_str());
        if (axis_names_list[i].length() == 0) {
            ROS_ERROR("axis name can't be empty");
            return -1;
        }
        if (axis_can_ids_list[i] <= 0) {
            ROS_ERROR("axis CAN id must be >0");
            return -1;
        }
        can_node = new odrive::ODriveAxis(&node, axis_names_list[i], axis_can_ids_list[i], 
            axis_directions_list[i]);
        odrive_axises.push_back(can_node);
    
    

        while(!can_node->ready2calibrate){
			ros::spinOnce();
			ros::Duration(0.01).sleep();
        }
        
    }
    for (int i = 0; i < axis_names_list.size(); i++) {
        odrive_axises[i]->setAxisRequestedState(odrive::ODriveAxisState::FULL_CALIBRATION_SEQUENCE);    	
    }
    double secs;
    secs = 0;
    while (secs < 15){
    	ros::spinOnce();
    	ros::Duration(0.001).sleep();
    	secs += 0.001;
    }
//    ros::Duration(15.0).sleep();
    for (int i = 0; i < axis_names_list.size(); i++) {
        odrive_axises[i]->engage();
    }

	ros::spin();
	return 0;
}
