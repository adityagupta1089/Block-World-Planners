#ifndef PROBLEM_HPP
#define PROBLEM_HPP

#include <map>
#include <set>

#include "predicate.hpp"

using namespace std;

typedef set<proposition> state;

enum planner_type {
	forward_bfs_planner,
	forward_astar_planner,
	goal_stack_planner
};

map<char, planner_type> planners =
	{
		{ 'f', forward_bfs_planner },
		{ 'a', forward_astar_planner },
		{ 'g', goal_stack_planner }

	};

struct problem {
		int blocks;
		planner_type type;
		state initial_state;
		state goal_state;
};
#endif /*PROBLEM_HPP*/
