#ifndef PROBLEM_CPP
#define PROBLEM_CPP

#include "problem.hpp"

#include <cstdlib>
#include <iostream>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "action.hpp"
#include "planner.hpp"
#include "predicate.hpp"

using namespace std;

void parse_propositions(int total_blocks, string line, state& _state) {
	regex _regex("\\((\\w+)((?:\\s\\d+)*)\\)");
	smatch match;
	while (regex_search(line, match, _regex)) {
		proposition _proposition = 0;
		_proposition += propositions[match[1]];
		stringstream ss(match[2]);
		string arg;
		int k = N;
		while (ss >> arg) {
			if (arg.length() > 0) {
				_proposition += atoi(arg.c_str()) * k;
				k *= N;
			}
		}
		line = match.suffix().str();
		_state.insert(_proposition);
	}
}

void find_actions(problem& _problem, vector<action>& _actions) {
	if (_problem.type == forward_bfs_planner) find_actions_forward_bfs(_problem.blocks, _problem.initial_state,
			_problem.goal_state, _actions);
	else if (_problem.type == forward_astar_planner) find_actions_forward_astar(_problem.blocks, _problem.initial_state,
			_problem.goal_state, _actions);
	else if (_problem.type == goal_stack_planner) find_actions_goal_stack(_problem.blocks, _problem.initial_state,
			_problem.goal_state, _actions);
}
bool is_goal_state(state& _state, state& goal) {
	for (proposition _proposition : goal) {
		if (!contains(_state, _proposition)) return false;
	}
	return true;
}
#endif /*PROBLEM_CPP*/
