#include "global_planner_wrapper.h"

int main(int _argc, char **_argv){
    srand((unsigned int) time(0));
    ros::init(_argc, _argv, "global_planner");
    ros::NodeHandle node_handler;
    ros::Rate rate(10);
    ROS_INFO("Creating\n");
    GlobalPlanner3dNodeWrapper wrapper(node_handler);
    while(ros::ok){
        // std::cout << "1\n" << std::endl;
        // ROS_INFO("Spinning\n");
        ros::spinOnce();
        rate.sleep();
    }


}