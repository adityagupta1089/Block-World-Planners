#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <action.hpp>
#include <predicate.hpp>
#include <problem.hpp>
#include <string>
#include <vector>

struct forward_search_node;

using namespace std;

template<typename K, typename V> inline bool contains(map<K, V> _map, V _val) {
	return _map.find(_val) != _map.end();
}

template<typename V> inline bool contains(set<V> _set, V _val) {
	return _set.find(_val) != _set.end();
}

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
#endif /*PLANNER_HPP*/
