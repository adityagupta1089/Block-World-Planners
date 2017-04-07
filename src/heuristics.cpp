#ifndef HEURISTICS_CPP
#define HEURISTICS_CPP

#include <predicate.hpp>
#include <problem.hpp>
#include <queue>

using namespace std;

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
#endif /*HEURISTICS_CPP*/
