#pragma once

#include <octomap/octomap.h>
#include <iostream>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_srvs/Empty.h"

#include <geometry_msgs/Point.h>

#include "global_planner.h"

class GlobalPlanner3dNodeWrapper {
public:
    GlobalPlanner3dNodeWrapper(ros::NodeHandle & _node_handler);
	~GlobalPlanner3dNodeWrapper();
    bool initialize();

private:
    std::string map_frame;
    std::string robot_frame;
    std::string map_topic;

    std::string global_path_pub_topic_name;
    std::string goal_pose_sub_topic_name;

    std::string model_cube_size;
    std::string bound_min;
    std::string bound_max;

    std::string map_path;
    int path_msg_counter = 0;

    /// The ROS node handle
    ros::NodeHandle node_handler;
    ros::Publisher pub_path;

    /// The received tf frame for start pose
    geometry_msgs::Transform start_in_map_transform;

    GlobalPlanner3dPtr global_planner_ptr;
    void read_param(std::string param, std::string & to);
    bool set_start();
	void set_goal_callback(const geometry_msgs::Point::ConstPtr & msg);
    bool plan(nav_msgs::Path & path_msg);
};