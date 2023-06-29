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

#include "logging.h"
#include "global_planner.h"

#define MAX_TIME_PLAN 10

// Constructor
GlobalPlanner3d::GlobalPlanner3d(
	CollisionGeometryPtr robot_shape_ptr,
	Eigen::VectorXd & _bound_min, Eigen::VectorXd & _bound_max
):  dimension_ambient_space(_bound_min.rows()), bound_min(_bound_min), bound_max(_bound_max){

	robot_object = std::make_shared<fcl::CollisionObject<double>>(robot_shape_ptr);
	fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
	obstacle_map = std::make_shared<fcl::CollisionObject<double>>((CollisionGeometryPtr(tree)));
	
	INFO("Given ambient space dimension: " << dimension_ambient_space);
	space_ptr = ob::StateSpacePtr(new ob::RealVectorStateSpace(dimension_ambient_space));

	// set the bounds for the R^3
	ob::RealVectorBounds bounds(dimension_ambient_space);
	for(uint8_t i=0; i < dimension_ambient_space; i++){
		bounds.setLow(i, bound_min[i]);
		bounds.setHigh(i, bound_max[i]);
	}

	space_ptr->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// construct an instance of  space_ptr information from this state space_ptr
	si_ptr = ob::SpaceInformationPtr(new ob::SpaceInformation(space_ptr));
	si_ptr->setStateValidityChecker(std::bind(&GlobalPlanner3d::is_state_valid, this, std::placeholders::_1 ));
	path_simplifier = std::make_shared<og::PathSimplifier>(si_ptr);

	// create a problem instance
	pdef_ptr = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_ptr));
	
	plan_ptr = ob::PlannerPtr(new og::RRTConnect(si_ptr));
	plan_ptr->setProblemDefinition(pdef_ptr);
	plan_ptr->setup();

	INFO("GlobalPlanner3d Initialized");

}

// Destructor
GlobalPlanner3d::~GlobalPlanner3d(){
}

void GlobalPlanner3d::set_bound_max(Eigen::VectorXd & max){
	bound_max = max;
}
void GlobalPlanner3d::set_bound_min(Eigen::VectorXd & min){
	bound_min = min;
}

bool GlobalPlanner3d::set_start(Eigen::VectorXd & start_in){
	ob::ScopedState<ob::RealVectorStateSpace> start_state(space_ptr);
	for(uint8_t i = 0; i < dimension_ambient_space; i++){
		start_state->values[i] = start_in[i];
	}
	ob::State *state =  space_ptr->allocState();
	state->as<ob::RealVectorStateSpace::StateType>()->values = start_state->values;
	if(is_state_valid(state)) // Check if the start state is valid
	{	
		pdef_ptr->clearStartStates();
		pdef_ptr->addStartState(start_state);
		DEBUG("Start point set to: " << start_in.transpose());
		return true;
	}
	else
	{
		ERROR("Start state: " << start_in.transpose() << " invalid");
		return false;
	}
}

bool GlobalPlanner3d::set_goal(Eigen::VectorXd & goal_in){
	ob::ScopedState<ob::RealVectorStateSpace> goal(space_ptr);
	for(uint8_t i = 0; i < dimension_ambient_space; i++){
		goal->values[i] = goal_in[i];
	}
	pdef_ptr->clearGoal();
	pdef_ptr->setGoalState(goal);
	ob::State *state =  space_ptr->allocState();
	state->as<ob::RealVectorStateSpace::StateType>()->values = goal->values;
	if(is_state_valid(state)){ // Check if the goal state is valid	
		DEBUG("Goal point set to: " << goal_in.transpose());
		return true;
	}
	else{
		ERROR("Goal state: " << goal_in.transpose() << " invalid");
		return false;
	}
}

void GlobalPlanner3d::update_map(octomap::OcTree & tree_oct){
	// convert octree to collision object
	fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::make_shared<const octomap::OcTree>(tree_oct));
	std::shared_ptr<fcl::CollisionGeometry<double>> _obstacle_map = CollisionGeometryPtr(tree);
	obstacle_map = std::make_shared<fcl::CollisionObject<double>>((_obstacle_map));
}

bool GlobalPlanner3d::replan(void){	
	if(path_smooth != NULL){
		og::PathGeometric* path = pdef_ptr->getSolutionPath()->as<og::PathGeometric>();
		DEBUG("Total Points:" << path->getStateCount());
		double distance;
		if(pdef_ptr->hasApproximateSolution()){
			DEBUG("Goal state not satisfied and distance to goal is: " << pdef_ptr->getSolutionDifference());
			replan_flag = true;
		}
		else{
			for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
			{
				if(!replan_flag){
					replan_flag = !is_state_valid(path->getState(idx));
				}
				else
					break;
			}
		}
	}
	if(replan_flag){
		pdef_ptr->clearSolutionPaths();
		DEBUG("Replanning");
		return plan();
	}
	else{
		DEBUG("Replanning not required");
		return false;
	}
}

bool GlobalPlanner3d::plan(){
	pdef_ptr->clearSolutionPaths();
	plan_ptr->clear();
    // attempt to solve the problem within four seconds of planning time
	ob::PlannerStatus solved = plan_ptr->solve(MAX_TIME_PLAN);

	if (solved)
	{
		DEBUG("Found solution");
		ob::PathPtr path = pdef_ptr->getSolutionPath();
		og::PathGeometric* pth = pdef_ptr->getSolutionPath()->as<og::PathGeometric>();
        //Path smoothing using bspline
		path_smooth = std::make_shared<og::PathGeometric>(dynamic_cast<const og::PathGeometric&>(*pdef_ptr->getSolutionPath()));
		bool simplify_ok = path_simplifier->simplify(*path_smooth, MAX_TIME_PLAN);
		path_simplifier->smoothBSpline(*path_smooth);
		replan_flag = !simplify_ok;
		return simplify_ok;
	}
	else
		DEBUG("No solution found");
		return false;
}

bool GlobalPlanner3d::is_state_valid(const ob::State *state){
    // cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);
	// INFO("State: " << translation);
	robot_object->setTranslation(translation);
	fcl::CollisionRequest<double> requestType(1,false,1,false);
	fcl::CollisionResult<double> collisionResult;
	fcl::collide(robot_object.get(), obstacle_map.get(), requestType, collisionResult);

	return(!collisionResult.isCollision());
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space_ptr of computed
// paths.

// ob::OptimizationObjectivePtr GlobalPlanner3d::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si_ptr)
// {
// 	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si_ptr));
// 	// obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
// 	return obj;
// }


std::vector<Eigen::VectorXd> GlobalPlanner3d::get_smooth_path(){
	std::vector<Eigen::VectorXd> path;
	for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
	{
        // cast the abstract state type to the type we expect
		const ob::RealVectorStateSpace::StateType *pos = path_smooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();
		Eigen::VectorXd state = Eigen::VectorXd::Zero(dimension_ambient_space);
		for(uint8_t i=0; i < dimension_ambient_space; i++){
			state[i] = pos->values[i];
		}
		path.push_back(state);
	}
	return path;
}