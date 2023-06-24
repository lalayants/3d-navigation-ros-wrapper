#include "global_planner_wrapper.h"
#include "global_planner.h"


void GlobalPlanner3dNodeWrapper::read_param(std::string param, std::string & to){
    if (!node_handler.getParam(param, to)){
        ROS_ERROR("ROS param %s is missing!", param.c_str());
        exit(1);
    }
    ROS_INFO("Got param map_frame: %s", to.c_str());
}

GlobalPlanner3dNodeWrapper::GlobalPlanner3dNodeWrapper(ros::NodeHandle & _node_handler)
{
    node_handler = _node_handler;
    // Read all params
    read_param("map_frame", map_frame);
    read_param("robot_frame", robot_frame);
    read_param("map_topic", map_topic);
    read_param("goal_pose_sub_topic_name", goal_pose_sub_topic_name);
    read_param("global_path_pub_topic_name", global_path_pub_topic_name);
    read_param("model_cube_size", model_cube_size);
    read_param("bound_min", bound_min);
    read_param("bound_max", bound_max);

    // Creat planner
    double bmin = ::atof(bound_min.c_str());
    double bmax = ::atof(bound_max.c_str());
    Eigen::Vector3d bound_mi;
    bound_mi << bmin, bmin, bmin;
    Eigen::Vector3d bound_ma;
    bound_ma << bmax, bmax, bmax;

    double size = ::atof(model_cube_size.c_str());
    CollisionGeometryPtr drone_shape = CollisionGeometryPtr(new fcl::Box<double>(size, size, size));
    global_planner = GlobalPlanner3d(drone_shape, bound_mi, bound_ma);
    // GlobalPlanner3d planner(drone_shape, bound_mi, bound_ma);
    // global_planner = planner;
    

    // TODO: read from octomap server
    // octomap::OcTree octree(map_path);
    // global_planner.update_map(octree);


    //Goal subscriber
    ros::Subscriber sub_goal = node_handler.subscribe(goal_pose_sub_topic_name, 10, set_goal_callback);
    if (!sub_goal){
        ROS_ERROR("Unable to subscribe on %s", goal_pose_sub_topic_name.c_str());
        exit(1);
    }

    //Path publisher
    ros::Publisher pub_path = node_handler.advertise<nav_msgs::Path>(global_path_pub_topic_name, 10);
    
}

bool GlobalPlanner3dNodeWrapper::set_start()
{
    tf::TransformListener tf_map_robot_listener;
    tf::StampedTransform tf_map_robot;

    try {
        tf_map_robot_listener.waitForTransform(map_frame, robot_frame, ros::Time(0), ros::Duration(2.0));
        tf_map_robot_listener.lookupTransform(map_frame, robot_frame, ros::Time(0), tf_map_robot);
    } catch(tf::TransformException &ex) {
        ROS_ERROR("No start point TF.");
        return false;
    }
    Eigen::Vector3d robot_start;
    robot_start << tf_map_robot.getOrigin().x(), tf_map_robot.getOrigin().y(), tf_map_robot.getOrigin().z();
    global_planner.set_start(robot_start);
    return true;
}

bool GlobalPlanner3dNodeWrapper::plan(nav_msgs::Path & path_msg)
{
    if (!set_start()) 
        return false;
    bool ok = planner.plan();
    if(!ok){
        return false;
    }
    std::vector<Eigen::VectorXd> out_path = planner.get_smooth_path();

    // convert to ROS Path
	std::vector<geometry_msgs::PoseStamped> poses(out_path.size());
	for (int i = 0; i < out_path.size(); i++)
	{
		poses.at(i).pose.position.x = out_path.at(i)[0];
		poses.at(i).pose.position.y = out_path.at(i)[1];
		poses.at(i).pose.position.z = out_path.at(i)[2];
	}
	path_msg.poses = poses;
    path_msg.header.stamp = ros::Time(0);
    path_msg.header.frame_id = map_frame;
    path_msg.header.seq = path_msg_counter++;
	return true;
}

bool GlobalPlanner3dNodeWrapper::set_goal_callback(geometry_msgs::Point::ConstPtr & msg){
    Eigen::Vector3d robot_goal;
    robot_goal << msg->x, msg->y, msg->z;
    global_planner.set_goal(robot_goal);
    nav_msgs::Path path_msg;
    if(!plan(path_msg)){
        return false;
    }
    pub_path.publish(path_msg);
    return true;
}
