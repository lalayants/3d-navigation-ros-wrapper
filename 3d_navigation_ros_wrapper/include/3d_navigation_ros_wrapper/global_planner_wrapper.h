#pragma once

#include <octomap/octomap.h>
#include <iostream>
#include <ros.h>

#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"

#include "global_planner.h"

class GlobalPlanner3dNodeWrapper {
public:
	GlobalPlanner3dNodeWrapper(
        ros::NodeHandle & node_handler;
    );
	~GlobalPlanner3dNodeWrapper();
    bool initialize();

private:
    std::string map_frame;
    std::string robot_frame;
    std::string map_topic;

    std::string global_path_pub_topic_name;
    std::string goal_pose_sub_topic_name;
    /// The ROS node handle
    ros::NodeHandle node_handler;

    /// The received tf frame for start pose
    geometry_msgs::Transform start_in_map_transform;

    GlobalPlanner3dPtr global_planner;
	bool set_goal_callback(Eigen::VectorXd & goal);
    bool plan();
};