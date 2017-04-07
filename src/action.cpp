#ifndef ACTION_CPP
#define ACTION_CPP

#include "action.hpp"

#include <map>
#include <set>
#include <utility>

#include "planner.hpp"
#include "problem.hpp"

bool action_applicable(variable_action& _variable_action, int var1, int var2, int total_blocks, state& _state, action& _action) {
	for (condition _condition : _variable_action.preconditions) {
		if (!contains(_state, _condition.value(var1, var2, total_blocks))) return false;
	}
	instantiate_action(_variable_action, _action, var1, var2);
	return true;
}

void instantiate_action(variable_action& _variable_action, action& _action, int var1, int var2) {
	if (_variable_action.args.size() == 0) _action= {_variable_action.name, {}};
	if (_variable_action.args.size() == 1) _action= {_variable_action.name, {var1}};
	if (_variable_action.args.size() == 2) _action= {_variable_action.name, {var1, var2}};
}

state apply_action(variable_action& _variable_action, int var1, int var2, int total_blocks, state& _state) {
// asserting that _variable_action is applicable on _state, i.e. preconditions are met
	state new_state;
	set<proposition> delete_set;
	for (condition effect : _variable_action.effects) {
		proposition _proposition = effect.value(var1, var2, total_blocks);
		if (effect._predicate > 0) new_state.insert(_proposition);
		else if (effect._predicate < 0) delete_set.insert(_proposition);
	}
	for (proposition _proposition : _state) {
		if (!contains(delete_set, _proposition)) new_state.insert(_proposition);
	}
	return new_state;
}
#endif /*ACTION_CPP*/
