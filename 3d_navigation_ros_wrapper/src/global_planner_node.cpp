#include "global_planner_wrapper.h"

int main(int _argc, char **_argv){
    srand((unsigned int) time(0));
    // Eigen::VectorXd start = Eigen::VectorXd::Random(3)*10;
    // Eigen::VectorXd end = Eigen::VectorXd::Random(3)*10;

    // CollisionGeometryPtr drone_shape = CollisionGeometryPtr(new fcl::Box<double>(0.25, 0.25, 0.25));

    // Eigen::VectorXd bound_min(3);
    // bound_min<<-10.0, -10.0, -10.0;
    // Eigen::VectorXd bound_max(3);
    // bound_max<<10.0, 10.0, 10.0;
    // GlobalPlanner3d planner(drone_shape, bound_min, bound_max);

    // planner.set_start(start);
    // planner.set_goal(end);
    // bool ok = planner.plan();
    // if(ok){
    //     auto out_path = planner.get_smooth_path();
    //     std::cout<<"Given path length: "<< out_path.size()<<std::endl;
    // }
    // octomap::OcTree octree("maps/sprint_501.bt");
    // planner.update_map(octree);
    // ok = planner.plan();
    // if(ok){
    //     auto out_path = planner.get_smooth_path();
    //     std::cout<<"Given path length: "<< out_path.size()<<std::endl;
    // }
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle node_handler;
    ros::Rate rate(10);

    GlobalPlanner3dNodeWrapper::GlobalPlanner3dNodeWrapper 3dplanner_wrapper(ros::NodeHandle & _node_handler);
    while(ros::ok){
        std::cout << '1\n' << std::endl;
        ros::spinOnce();
        rate.sleep();
    }


}