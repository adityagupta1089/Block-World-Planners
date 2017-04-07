#ifndef RELEVANT_ACTION_CPP
#define RELEVANT_ACTION_CPP

#include <cstdint>
#include <cstdlib>
#include <map>
#include <utility>
#include <vector>

#include "action.hpp"
#include "planner.hpp"
#include "predicate.hpp"
#include "problem.hpp"

using namespace std;

struct action_record {
		map<action, int> count;
};

map<pair<state, proposition>, action_record> action_records;

void get_relevant_action(variable_action& relevant_variable_action, action& _relevant_action, int total_blocks, state& curr_state,
		proposition goal) {
	int min_count = INT32_MAX;
	int max_relevant_propositions = 0;
	variable_action _va;
	action _a;
	auto key = make_pair(curr_state, goal);
	for (variable_action& _variable_action : variable_actions) {
		for (int i = 1; i <= (_variable_action.args.size() >= 1 ? total_blocks : 1); i++) {
			for (int j = 1; j <= (_variable_action.args.size() >= 2 ? total_blocks : 1); j++) {
				if (_variable_action.args.size() >= 2 && i == j) continue;
				if (relevant_action(_variable_action, i, j, total_blocks, goal)) {
					action _action;
					instantiate_action(_variable_action, _action, i, j);
					if (!contains(action_records[key].count, _action)) {
						action_records[key].count[_action] = 0;
					}
					int relevant_propositions = 0;
					for (condition precondition : _variable_action.preconditions) {
						if (contains(curr_state, precondition.value(i, j, total_blocks))) {
							relevant_propositions++;
						}
					}
					int count = action_records[key].count[_action];
					if (count < min_count
							|| (count == min_count && (relevant_propositions > max_relevant_propositions || rand() % 100 == 0))) {
						min_count = action_records[key].count[_action];
						if (count == min_count && relevant_propositions > max_relevant_propositions) max_relevant_propositions =
								relevant_propositions;
						_va = _variable_action;
						_a = _action;
					}
				}
			}
		}
	}

	action_records[key].count[_a]++;
	relevant_variable_action = _va;
	_relevant_action = _a;

}

bool relevant_action(variable_action& _variable_action, int var1, int var2, int total_blocks, proposition& _proposition) {
	bool relevant = false;
	for (condition effect : _variable_action.effects) {
		proposition __proposition = effect.value(var1, var2, total_blocks);
		if (effect._predicate > 0 && _proposition == __proposition) relevant = true;
		if (effect._predicate < 0 && _proposition == __proposition) return false;
	}
	return relevant;
}
#endif /*RELEVANT_ACTION_CPP*/
