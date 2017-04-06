#include <stdlib.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <queue>
#include <regex>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

#define contains(_map, _val) (_map.find(_val) != _map.end())

//=============================================
// ENUMS
//=============================================
enum planner_type {
	forward_bfs_planner,
	forward_astar_planner,
	goal_stack_planner
};

typedef int predicate;

#define	predicate_on 1
#define	predicate_on_table 2
#define	predicate_clear 3
#define	predicate_hold 4
#define	predicate_empty 5
#define total_predicates 5

enum action_type {
	action_pick,
	action_unstack,
	action_release,
	action_stack
};

//=============================================
// STRUCTS
//=============================================

typedef int proposition;

#define N (max(total_blocks, total_predicates) + 1)

#define type(x) (x % N)

#define var1(x) ((x / N) % N)
#define var2(x) (x / (N * N))

struct condition {
		predicate _predicate;
		vector<char> args;

		inline proposition value(int var1, int var2, int total_blocks) {
			int k = N;
			int v = abs(_predicate);
			for (char c : args) {
				v += k * ((c == 'a') ? var1 : var2);
				k *= N;
			}
			return v;
		}
};

typedef set<proposition> state;

struct problem {
		int blocks;
		planner_type type;
		state initial_state;
		state goal_state;
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

struct forward_search_node {
		state curr_state;
		action _action;
		forward_search_node* prev_node;
		int level;
};

struct action_record {
		map<action, int> count;
};

typedef vector<proposition> conjunct_goal;

//=============================================
// MAPS
//=============================================
map<string, predicate> propositions =
	{
		{ "ontable", predicate_on_table },
		{ "on", predicate_on },
		{ "clear", predicate_clear },
		{ "hold", predicate_hold },
		{ "empty", predicate_empty }

	};

map<char, planner_type> planners =
	{
		{ 'f', forward_bfs_planner },
		{ 'a', forward_astar_planner },
		{ 'g', goal_stack_planner }

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

//=============================================
// PROTOTOTYPES
//=============================================

// top level functions
void read_input(char*, problem&);
void parse_propositions(int, string, state&);
void find_actions(problem&, vector<action>&);
void write_actions(char*, vector<action>&);

// action finding functions
void find_actions_forward_bfs(int, state, state&, vector<action>&);
void find_actions_forward_astar(int, state, state&, vector<action>&);
void find_actions_goal_stack(int, state, state&, vector<action>&);

// helper functions
bool action_applicable(variable_action&, int, int, int, state&, action&);
void instantiate_action(variable_action&, action&, int, int);
void get_relevant_action(variable_action&, action&, int, state&, proposition);
bool relevant_action(variable_action&, int, int, int, proposition&);
bool is_goal_state(state&, state&);
state apply_action(variable_action&, int, int, int, state&);
void track_actions(forward_search_node*, vector<action>&);

// heuristic functions
int* map_heights(const state&, int);
int heuristic_value(const state&, int*, int);

//print functions
void print_state(const state&, int);
void print_action(const action&);
void print_proposition(const proposition&, int);

//=============================================
// MAIN
//=============================================
int main(int argc, char **argv) {
	if (argc < 3) {
		printf("Input Format: %s <input file> <output file>", argv[0]);
		exit(1);
	}
	problem _problem;
	vector<action> _actions;

	read_input(argv[1], _problem);
	find_actions(_problem, _actions);
	write_actions(argv[2], _actions);

	return 0;
}

//=============================================
// FUNCTIONS
//=============================================

//=============================================
// I. TOP LEVEL FUNCTIONS
//=============================================
void read_input(char* file, problem& _problem) {
	ifstream in(file);
	string line;
	int line_num = 0;
	while (getline(in, line)) {
		switch (line_num) {
			case 0:
				_problem.blocks = atoi(line.c_str());
				break;
			case 1:
				_problem.type = planners[line[0]];
				break;
			case 3:
				parse_propositions(_problem.blocks, line, _problem.initial_state);
				break;
			case 5:
				parse_propositions(_problem.blocks, line, _problem.goal_state);
				break;
			default:
				break;
		}
		line_num++;
	}
}

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

void track_actions(forward_search_node* _node, vector<action>& actions) {
	stack<action> actions_stack;
	while (_node->_action.name.length() > 0) {
		actions_stack.push(_node->_action);
		_node = _node->prev_node;
	}
	while (!actions_stack.empty()) {
		actions.push_back(actions_stack.top());
		actions_stack.pop();
	}
}

void write_actions(char* file, vector<action>& _actions) {
	ofstream out(file);
	out << _actions.size() << "\n";
	for (action& _action : _actions) {
		out << _action.name << " ";
		for (int arg : _action.args)
			out << arg << " ";
		out << "\n";
	}
	out.close();
}

//=============================================
// II. SEARCHING FUNCTIONS
//=============================================

template<typename T, typename U> void find_actions_forward(int total_blocks, state init_state, state& goal_state,
		vector<action>& actions, T& nodes_queue, U top_node) {
	set<state> visited;
	forward_search_node* initial_node = new forward_search_node();
	*initial_node=
	{	init_state,
		{	"", {}},
		NULL,
		0

	};
	nodes_queue.push(initial_node);
	forward_search_node goal_node =
		{ goal_state,
			{ "", {} }, NULL };
	while (!nodes_queue.empty()) {
		forward_search_node* top = new forward_search_node();
		top = top_node(nodes_queue);
		nodes_queue.pop();
		if (is_goal_state(top->curr_state, goal_state)) {
			track_actions(top, actions);
			return;
		}
		if (!contains(visited, top->curr_state)) {
			visited.insert(top->curr_state);
			for (variable_action& _variable_action : variable_actions) {
				action _action;
				for (int i = 1; i <= (_variable_action.args.size() >= 1 ? total_blocks : 1); i++) {
					for (int j = 1; j <= (_variable_action.args.size() >= 2 ? total_blocks : 1); j++) {
						if (_variable_action.args.size() >= 2 && i == j) continue;
						if (action_applicable(_variable_action, i, j, total_blocks, top->curr_state, _action)) {
							state next_state = apply_action(_variable_action, i, j, total_blocks, top->curr_state);
							if (contains(visited, next_state)) continue;
							forward_search_node* next_node = new forward_search_node();
							*next_node =
							{
								next_state,
								_action,
								top,
								top->level + 1

							};
							nodes_queue.push(next_node);
						}
					}
				}
			}
		}
	}
}

void find_actions_forward_bfs(int total_blocks, state init_state, state& goal_state, vector<action>& actions) {
	queue<forward_search_node*> nodes;
	find_actions_forward(total_blocks, init_state, goal_state, actions, nodes,
			[&](queue<forward_search_node*> x) {return x.front();});
}

void find_actions_forward_astar(int total_blocks, state init_state, state& goal_state, vector<action>& actions) {
	int* height_goal = map_heights(goal_state, total_blocks);
	const auto comparator =
			[=](const forward_search_node* n1, const forward_search_node* n2) {
				return n1->level + heuristic_value(n1->curr_state, height_goal, total_blocks) > n2->level + heuristic_value(n2->curr_state, height_goal, total_blocks);
			};
	priority_queue<forward_search_node*, vector<forward_search_node*>, decltype(comparator)> nodes(comparator);
	find_actions_forward(total_blocks, init_state, goal_state, actions, nodes,
			[&](priority_queue<forward_search_node*, vector<forward_search_node*>, decltype(comparator)> x) -> forward_search_node* {
				return x.top();
			});
	delete[] height_goal;
}

#define CONJUNCT_GOAL -1
#define ACTION -2

void print_goal_stack() {

}

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

//=============================================
// IIa. HEURISTIC FUNCTIONS
//=============================================

int* map_heights(const state& _state, int total_blocks) {
	int next[total_blocks + 1];
	int* h = new int[total_blocks + 1];
	for (int i = 1; i <= total_blocks; i++) {
		next[i] = -1;
		h[i] = -1;
	}
	queue<int> _queue;
	for (proposition _proposition : _state) {
		if (type(_proposition) == predicate_on_table) {
			h[var1(_proposition)] = 0;
			_queue.push(var1(_proposition));
		} else if (type(_proposition) == predicate_on) {
			next[var2(_proposition)] = var1(_proposition);
		}
	}
	while (!_queue.empty()) {
		int top = _queue.front();
		_queue.pop();
		if (next[top] != -1) {
			h[next[top]] = h[top] + 1;
			_queue.push(next[top]);
		}
	}
	return h;
}

int heuristic_value(const state& curr_state, int* height_goal, int total_blocks) {
	int* height_curr = map_heights(curr_state, total_blocks);
	int _heuristic_value = 0;
	for (int i = 1; i <= total_blocks; i++) {
		if (height_curr[i] != -1 && height_goal[i] != -1 && height_curr[i] != height_goal[i]) {
			_heuristic_value += 2;
		}
	}
	for (proposition _proposition : curr_state)
		if (type(_proposition) == predicate_hold) _heuristic_value -= 1;
	delete[] height_curr;
	return _heuristic_value;
}

//=============================================
// IIb. HELPER FUNCTIONS
//=============================================

bool action_applicable(variable_action& _variable_action, int var1, int var2, int total_blocks, state& _state, action& _action) {
	for (condition _condition : _variable_action.preconditions) {
		if (!contains(_state,_condition.value(var1, var2, total_blocks))) return false;
	}
	instantiate_action(_variable_action, _action, var1, var2);
	return true;
}

void instantiate_action(variable_action& _variable_action, action& _action, int var1, int var2) {
	if (_variable_action.args.size() == 0) _action= {_variable_action.name, {}};
	if (_variable_action.args.size() == 1) _action= {_variable_action.name, {var1}};
	if (_variable_action.args.size() == 2) _action= {_variable_action.name, {var1, var2}};
}

bool is_goal_state(state& _state, state& goal) {
	for (proposition _proposition : goal) {
		if (!contains(_state, _proposition)) return false;
	}
	return true;
}

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
		if (!contains(delete_set,_proposition)) new_state.insert(_proposition);
	}
	return new_state;
}

//=============================================
// III. PRINTING FUNCTIONS
//=============================================

map<int, string> proposition_names =
	{
		{ 1, "on" },
		{ 2, "ontable" },
		{ 3, "clear" },
		{ 4, "hold" },
		{ 5, "empty" }

	};

void print_state(const state& top, int total_blocks) {
	for (const proposition& _proposition : top) {
		print_proposition(_proposition, total_blocks);
	}
	cout << "\n";
}

void print_proposition(const proposition& _proposition, int total_blocks) {
	cout << "(" << proposition_names[type(_proposition)];
	if (_proposition > N) cout << " " << var1(_proposition);
	if (_proposition > N * N) cout << " " << var2(_proposition);
	cout << ") [";
	cout << _proposition;
	cout << "] ";
}

void print_action(const action& _action) {
	cout << _action.name;
	for (int x : _action.args)
		cout << " " << x;
}
