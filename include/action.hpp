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

variable_action variable_action_pick =
	{ action_pick, "pick",
		{ 'a' },
		{
			{ predicate_on_table,
				{ 'a' } },
			{ predicate_clear,
				{ 'a' } },
			{ predicate_empty, {} }

		},
		{
			{ predicate_hold,
				{ 'a' } },
			{ -predicate_clear,
				{ 'a' } },
			{ -predicate_empty, {} },
			{ -predicate_on_table,
				{ 'a' } }

		}

	};

variable_action variable_action_unstack =
	{ action_unstack, "unstack",
		{ 'a', 'b' },
		{
			{ predicate_on,
				{ 'a', 'b' } },
			{ predicate_clear,
				{ 'a' } },
			{ predicate_empty, {} } },

		{
			{ predicate_hold,
				{ 'a' } },
			{ predicate_clear,
				{ 'b' } },
			{ -predicate_on,
				{ 'a', 'b' } },
			{ -predicate_empty, {} },
			{ -predicate_clear,
				{ 'a' } }

		}

	};

variable_action variable_action_release =
	{ action_release, "release",
		{ 'a' },
		{
			{ predicate_hold,
				{ 'a' } }

		},
		{
			{ predicate_on_table,
				{ 'a' } },
			{ predicate_clear,
				{ 'a' } },
			{ predicate_empty, {} },
			{ -predicate_hold,
				{ 'a' } }

		},

	};

variable_action variable_action_stack =
	{ action_stack, "stack",
		{ 'a', 'b' },
		{
			{ predicate_clear,
				{ 'b' } },
			{ predicate_hold,
				{ 'a' } }

		},
		{
			{ predicate_on,
				{ 'a', 'b' } },
			{ predicate_clear,
				{ 'a' } },
			{ predicate_empty, {} },
			{ -predicate_hold,
				{ 'a' } },
			{ -predicate_clear,
				{ 'b' } }

		}

	};

vector<variable_action> variable_actions =
	{ variable_action_pick, variable_action_unstack, variable_action_release, variable_action_stack };

#endif /*ACTION_HPP*/
