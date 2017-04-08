#ifndef GOAL_STACK_CPP
#define GOAL_STACK_CPP

#include <action.hpp>
#include <planner.hpp>
#include <predicate.hpp>
#include <problem.hpp>
#include <map>
#include <set>
#include <stack>
#include <utility>
#include <vector>

#define CONJUNCT_GOAL -1
#define ACTION -2

typedef vector<proposition> conjunct_goal;

void find_actions_goal_stack(int total_blocks, state curr_state, state& goal_state, vector<action>& actions) {
	stack<proposition> goal_stack;
	stack<conjunct_goal> conjunct_goal_stack;
	stack<pair<action, variable_action>> action_stack;

	conjunct_goal _goal(goal_state.begin(), goal_state.end());
	conjunct_goal_stack.push(_goal);
	goal_stack.push(CONJUNCT_GOAL);
	for (proposition _proposition : goal_state) {
		goal_stack.push(_proposition);
	}
	while (!goal_stack.empty()) {
		proposition top = goal_stack.top();
		if (top != CONJUNCT_GOAL) goal_stack.pop(); // we check if it is satisfied
		if (top == CONJUNCT_GOAL) { // conjunct goal
			bool satisfied = true;
			conjunct_goal conjunct_top = conjunct_goal_stack.top();
			for (proposition _proposition : conjunct_top) {
				if (curr_state.find(_proposition) == curr_state.end()) {
					satisfied = false;
					break;
				}
			}
			if (!satisfied) {
				//skip first
				for (int i = 1; i < conjunct_top.size(); i++) {
					goal_stack.push(conjunct_top[i]);
				}
				goal_stack.push(conjunct_top[0]);
			} else {
				conjunct_goal_stack.pop();
				goal_stack.pop();
			}
		} else if (top == ACTION) { // action
			pair<action, variable_action> action_top = action_stack.top();
			action_stack.pop();
			actions.push_back(action_top.first);
			curr_state = apply_action(action_top.second, action_top.first.args[0], action_top.first.args[1], total_blocks,
					curr_state);
			if (is_goal_state(curr_state, goal_state)) {
				return;
			}
		} else { // predicate/proposition
			if (curr_state.find(top) != curr_state.end()) continue;
			variable_action _variable_action;
			action _action;
			get_relevant_action(_variable_action, _action, total_blocks, curr_state, top);
			vector<proposition> preconditions;
			action_stack.push(make_pair(_action, _variable_action));
			goal_stack.push(ACTION);
			goal_stack.push(CONJUNCT_GOAL);
			for (condition precondition : _variable_action.preconditions) {
				proposition _proposition = precondition.value(_action.args[0], _action.args[1], total_blocks);
				preconditions.push_back(_proposition);
				goal_stack.push(_proposition);
			}
			conjunct_goal_stack.push(preconditions);
		}
	}
}
#endif /*GOAL_STACK_CPP*/
