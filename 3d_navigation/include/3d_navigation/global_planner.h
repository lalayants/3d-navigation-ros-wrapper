/*
*   3d_navigation
*
*   Copyright (C) 2018  ./lab_449
*
*   THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
*   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
*   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
*   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
*   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*   DEALINGS IN THE SOFTWARE.
*
*   Author Antipov Vladislav <texnoman@gmail.com>
*/

#pragma once

#include <octomap/octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <array>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

typedef std::shared_ptr<fcl::CollisionGeometry<double>> CollisionGeometryPtr;
typedef std::shared_ptr<fcl::CollisionObject<double>>  CollisionObjectPtr;

namespace ob = ompl::base;
namespace og = ompl::geometric;

class GlobalPlanner3d {
public:
	GlobalPlanner3d(
        CollisionGeometryPtr robot_shape_ptr,
        Eigen::VectorXd & _bound_min, Eigen::VectorXd & _bound_max
    );
	~GlobalPlanner3d();

	bool set_start(Eigen::VectorXd & start);
	bool set_goal(Eigen::VectorXd & goal);

	void update_map(octomap::OcTree & tree_oct);
	bool plan(void);
	bool replan(void);
	std::vector<Eigen::VectorXd> get_smooth_path();

private:
    uint8_t dimension_ambient_space;
	Eigen::VectorXd bound_min, bound_max;
	ob::StateSpacePtr space_ptr;
	ob::SpaceInformationPtr si_ptr;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef_ptr;

	// Planner instance and properties
	ob::PlannerPtr plan_ptr;
	bool replan_flag = true;


	// Path instances
	og::PathSimplifierPtr path_simplifier;
	og::PathGeometricPtr path_smooth;
	
	CollisionObjectPtr obstacle_map;
	CollisionObjectPtr robot_object;

	bool is_state_valid(const ob::State *state);
};

typedef std::shared_ptr<GlobalPlanner3d> GlobalPlanner3dPtr;