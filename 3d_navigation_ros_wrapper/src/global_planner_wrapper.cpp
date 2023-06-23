#include "global_planner_wrapper.h"

GlobalPlanner3dNodeWrapper::GlobalPlanner3dNodeWrapper(ros::NodeHandle & _node_handler)
: node_handler(_node_handler){

    node_handler.param("", , 50.0);
}