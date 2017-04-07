#ifndef FORWARD_SEARCH_CPP
#define FORWARD_SEARCH_CPP

#include <forward_search.hpp>
#include <planner.hpp>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "heuristics.cpp"

using namespace std;

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
#endif /*FORWARD_SEARCH_CPP*/
