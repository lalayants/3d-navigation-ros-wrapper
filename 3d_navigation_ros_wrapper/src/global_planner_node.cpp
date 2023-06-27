#include "global_planner_wrapper.h"

int main(int _argc, char **_argv){
    srand((unsigned int) time(0));
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle node_handler;
    ros::Rate rate(10);

    GlobalPlanner3dNodeWrapper 3dplanner_wrapper(ros::NodeHandle & _node_handler);
    while(ros::ok){
        std::cout << '1\n' << std::endl;
        ros::spinOnce();
        rate.sleep();
    }


}