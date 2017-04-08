#ifndef ACTION_HPP
#define ACTION_HPP

#include <predicate.hpp>
#include <string>
#include <vector>

using namespace std;

enum action_type {
	action_pick,
	action_unstack,
	action_release,
	action_stack
};

struct variable_action {
		action_type type;
		string name;
		vector<char> args;
		vector<condition> preconditions;
		vector<condition> effects;
};

struct action {
		string name;
		vector<int> args;
		bool operator<(const action& that) const {
			if (name.compare(that.name) >= 0) return false;
			if (args.size() >= that.args.size()) return false;
			for (int i = 0; i < args.size(); i++)
				if (args[i] >= that.args[i]) return false;
			return true;
		}
};

extern variable_action variable_action_pick;
extern variable_action variable_action_unstack;
extern variable_action variable_action_release;
extern variable_action variable_action_stack;
extern vector<variable_action> variable_actions;

#endif /*ACTION_HPP*/
