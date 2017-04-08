#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <predicate.hpp>
#include <map>
#include <set>

using namespace std;

typedef set<proposition> state;

enum planner_type {
	forward_bfs_planner,
	forward_astar_planner,
	goal_stack_planner
};

extern map<char, planner_type> planners;

struct problem {
		planner_type type;
		state initial_state;
		state goal_state;
};
#endif /*PROBLEM_HPP*/
