#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <action.hpp>
#include <predicate.hpp>
#include <problem.hpp>
#include <map>
#include <set>
#include <string>
#include <vector>

struct forward_search_node;

extern int TOTAL_BLOCKS;

using namespace std;

template<typename K, typename V> inline bool contains(map<K, V> _map, K _key) {
	return _map.find(_key) != _map.end();
}

template<typename V> inline bool contains(set<V> _set, V _val) {
	return _set.find(_val) != _set.end();
}

// top level functions
void read_input(char* filename, problem& prob);
void parse_propositions(string line, state& stat);
void find_actions(problem& prob, vector<action>& acts);
void write_actions(char* filename, vector<action>& acts);

// action finding functions
void find_actions_forward_bfs(state init, state& goal, vector<action>& acts);
void find_actions_forward_astar(state init, state& goal, vector<action>& acts);
void find_actions_goal_stack(state init, state& goal, vector<action>& acts);

// helper functions
bool action_applicable(variable_action& varact, int var1, int var2, state& stat, action& act);
void instantiate_action(variable_action& varact, action& act, int var1, int var2);
void get_relevant_action(variable_action& varact, action& act, state& stat, proposition prop);
bool relevant_action(variable_action& varact, int var1, int var2, proposition&);
bool is_goal_state(state& stat, state& goal);
state apply_action(variable_action& varact, int var1, int var2, state& stat);
void track_actions(forward_search_node* node, vector<action>& acts);

// heuristic functions
int* map_heights(const state& stat);
int heuristic_value(const state& stat, int* goalh);

//print functions
void print_state(const state& stat);
void print_action(const action& act);
void print_proposition(const proposition& prop);
#endif /*PLANNER_HPP*/
