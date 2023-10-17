/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <geometry_msgs/TransformStamped.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <std_msgs/Int8.h>

#include <thread>

static const std::string LOGNAME = "cpp_interface_example";

// Class for monitoring status of moveit_servo
class StatusMonitor {
   public:
    StatusMonitor(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
    }

   private:
    void statusCB(const std_msgs::Int8ConstPtr& msg) {
        moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
        if (latest_status != status_) {
            status_ = latest_status;
            const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
            ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
        }
    }
    moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
    ros::Subscriber sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, LOGNAME);
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(8);
    spinner.start();

    // Load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor->getPlanningScene()) {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor();

    // Create the pose tracker
    moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

    // Make a publisher for sending pose commands
    // ros::Publisher target_pose_pub =
    //     nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

    // Subscribe to servo status (and log it when it changes)
    StatusMonitor status_monitor(nh, "status");

    Eigen::Vector3d lin_tol{0.0, 0.0, 0.0};
    double rot_tol = 0.0;

    // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when
    // moving between multiple waypoints
    tracker.resetTargetPose();

    // Publish target pose
    // target_pose.header.stamp = ros::Time::now();
    // target_pose_pub.publish(target_pose);

    // Run the pose tracking in a new thread
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        tracker.moveToPose(lin_tol, rot_tol, 0.005 /* target pose timeout */);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Make sure the tracker is stopped and clean up
    tracker.stopMotion();

    return EXIT_SUCCESS;
}
