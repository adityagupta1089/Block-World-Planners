#include <stdlib.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <regex>
#include <set>
#include <stack>
#include <string>
//#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

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

		proposition value(int var1, int var2, int total_blocks) {
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
};

struct node {
		state curr_state;
		action _action;
		node* prev_node;
		int level;
};

typedef set<proposition> compound_goal;

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
	{
		variable_action_pick,
		variable_action_unstack,
		variable_action_release,
		variable_action_stack };

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
bool is_goal_state(state&, state&);
state apply_action(variable_action&, int, int, int, state&);
void track_actions(node*, vector<action>&);

// heuristic functions
int* map_heights(const state&, int);
int heuristic_value(const state&, int*, int);

//print functions
void print_state(const state&, int);
void print_action(const action&);

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
	if (_problem.type == forward_bfs_planner) find_actions_forward_bfs(_problem.blocks,
			_problem.initial_state, _problem.goal_state, _actions);
	else if (_problem.type == forward_astar_planner) find_actions_forward_astar(
			_problem.blocks, _problem.initial_state, _problem.goal_state, _actions);
	else if (_problem.type == goal_stack_planner) find_actions_goal_stack(_problem.blocks,
			_problem.initial_state, _problem.goal_state, _actions);
}

void track_actions(node* _node, vector<action>& actions) {
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
	for (auto& _action : _actions) {
		out << _action.name << " ";
		for (auto arg : _action.args)
			out << arg << " ";
		out << "\n";
	}
	out.close();
}

//=============================================
// II. SEARCHING FUNCTIONS
//=============================================

template<typename T, typename U> void find_actions_forward(int total_blocks,
		state init_state, state& goal_state, vector<action>& actions, T& nodes_queue,
		U top_node) {
	set<state> visited;
	node* initial_node = new node();
	*initial_node=
	{	init_state,
		{	"", {}},
		NULL,
		0

	};
	nodes_queue.push(initial_node);
	node goal_node =
		{ goal_state,
			{ "", {} }, NULL };
	int current_level = 0;
	while (!nodes_queue.empty()) {
		node* top = new node();
		top = top_node(nodes_queue);
		nodes_queue.pop();
		if (is_goal_state(top->curr_state, goal_state)) {
			track_actions(top, actions);
			return;
		}
		if (visited.find(top->curr_state) == visited.end()) {
			visited.insert(top->curr_state);
			for (auto& _variable_action : variable_actions) {
				action _action;
				for (int i = 1;
						i <= (_variable_action.args.size() >= 1 ? total_blocks : 1);
						i++) {
					for (int j = 1;
							j <= (_variable_action.args.size() >= 2 ? total_blocks : 1);
							j++) {
						if (_variable_action.args.size() >= 2 && i == j) continue;
						if (action_applicable(_variable_action, i, j, total_blocks,
								top->curr_state, _action)) {
							state next_state = apply_action(_variable_action, i, j,
									total_blocks, top->curr_state);
							if (visited.find(next_state) != visited.end()) continue;
							node* next_node = new node();
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

void find_actions_forward_bfs(int total_blocks, state init_state, state& goal_state,
		vector<action>& actions) {
	queue<node*> nodes;
	find_actions_forward(total_blocks, init_state, goal_state, actions, nodes,
			[&](queue<node*> x) {return x.front();});
}

void find_actions_forward_astar(int total_blocks, state init_state, state& goal_state,
		vector<action>& actions) {
	int* height_goal = map_heights(goal_state, total_blocks);
	const auto comparator = [=](const node* n1, const node* n2) {
		return n1->level + heuristic_value(n1->curr_state, height_goal, total_blocks)
		> n2->level + heuristic_value(n2->curr_state, height_goal, total_blocks);
	};
	priority_queue<node*, vector<node*>, decltype(comparator)> nodes(comparator);
	find_actions_forward(total_blocks, init_state, goal_state, actions, nodes,
			[&](priority_queue<node*, vector<node*>, decltype(comparator)> x) -> node* {
				return x.top();
			});
	delete[] height_goal;
}

void find_actions_goal_stack(int total_blocks, state curr_state, state& goal_state,
		vector<action>& actions) {
	stack<compound_goal> goal_stack;
	goal_stack.push(goal_state);
	for (auto _proposition : goal_state) {
		goal_stack.push(
			{ _proposition });
	}
	while (!goal_stack.empty()) {
		auto top = goal_stack.top();
		goal_stack.pop();
		if (top.size() > 1) {

		} else if (top.size() == 1) {
			for (auto& _variable_action : variable_actions) {
				action _action;
				for (int i = 1;
						i <= (_variable_action.args.size() >= 1 ? total_blocks : 1);
						i++) {
					for (int j = 1;
							j <= (_variable_action.args.size() >= 2 ? total_blocks : 1);
							j++) {
						if (_variable_action.args.size() >= 2 && i == j) continue;
						if (action_applicable(_variable_action, i, j, total_blocks,
								curr_state, _action)) {
							//TODO
						}
					}
				}
			}
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
	for (auto x : _state) {
		if (type(x) == predicate_on_table) {
			h[var1(x)] = 0;
			_queue.push(var1(x));
		} else if (type(x) == predicate_on) {
			next[var2(x)] = var1(x);
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

map<state, int> cache;

int heuristic_value(const state& curr_state, int* height_goal, int total_blocks) {
	if (cache.find(curr_state) != cache.end()) return cache[curr_state];
	int* height_curr = map_heights(curr_state, total_blocks);
	int _heuristic_value = 0;
	for (int i = 1; i <= total_blocks; i++) {
		if (height_curr[i] != -1 && height_goal[i] != -1
				&& height_curr[i] != height_goal[i]) {
			_heuristic_value += 2;
		}
	}
	for (auto _proposition : curr_state)
		if (type(_proposition) == predicate_hold) _heuristic_value -= 1;
	delete[] height_curr;
	cache[curr_state] = _heuristic_value;
	return _heuristic_value;
}

//=============================================
// IIb. HELPER FUNCTIONS
//=============================================

bool action_applicable(variable_action& _variable_action, int var1, int var2,
		int total_blocks, state& _state, action& _action) {
	for (auto _condition : _variable_action.preconditions) {
		if (_state.find(_condition.value(var1, var2, total_blocks)) == _state.end()) return false;
	}
	if (_variable_action.args.size() == 0) _action= {_variable_action.name, {}};
	if (_variable_action.args.size() == 1) _action= {_variable_action.name, {var1}};
	if (_variable_action.args.size() == 2) _action= {_variable_action.name, {var1, var2}};
	return true;
}

bool is_goal_state(state& _state, state& goal) {
	for (auto _proposition : goal) {
		if (_state.find(_proposition) == _state.end()) return false;
	}
	return true;
}

state apply_action(variable_action& _variable_action, int var1, int var2,
		int total_blocks, state& _state) {
// asserting that action is applicable
	state new_state;
	set<proposition> delete_set;
	for (auto effect : _variable_action.effects) {
		proposition prop = effect.value(var1, var2, total_blocks);
		if (effect._predicate > 0) new_state.insert(prop);
		else if (effect._predicate < 0) delete_set.insert(prop);
	}
	for (auto _proposition : _state) {
		if (delete_set.find(_proposition) == delete_set.end()) new_state.insert(
				_proposition);
	}
	return new_state;
}

//=============================================
// III. PRINTING FUNCTIONS
//=============================================

map<int, string> mp =
	{
		{ 1, "on" },
		{ 2, "ontable" },
		{ 3, "clear" },
		{ 4, "hold" },
		{ 5, "empty" }

	};

void print_state(const state& top, int total_blocks) {
	for (auto x : top) {
		cout << "(" << mp[type(x)];
		if (x > N) cout << " " << var1(x);
		if (x > N * N) cout << " " << var2(x);
		cout << ") ";
	}
	cout << "\n";
}

void print_action(const action& _action) {
	cout << _action.name;
	for (auto x : _action.args)
		cout << " " << x;
}
