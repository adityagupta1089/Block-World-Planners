#ifndef FORWARD_SEARCH_HPP
#define FORWARD_SEARCH_HPP

#include <action.hpp>
#include <problem.hpp>

struct forward_search_node {
		state curr_state;
		action _action;
		forward_search_node* prev_node;
		int level;
};

#endif /*FORWARD_SEARCH_HPP*/
