#ifndef PRINT_CPP
#define PRINT_CPP

#include <action.hpp>
#include <predicate.hpp>
#include <problem.hpp>
#include <iostream>
#include <map>
#include <string>

using namespace std;

map<int, string> proposition_names =
	{
		{ 1, "on" },
		{ 2, "ontable" },
		{ 3, "clear" },
		{ 4, "hold" },
		{ 5, "empty" }

	};
void print_proposition(const proposition& _proposition, int total_blocks) {
	cout << "(" << proposition_names[type(_proposition)];
	if (_proposition > N) cout << " " << var1(_proposition);
	if (_proposition > N * N) cout << " " << var2(_proposition);
	cout << ") [";
	cout << _proposition;
	cout << "] ";
}

void print_state(const state& top, int total_blocks) {
	for (const proposition& _proposition : top) {
		print_proposition(_proposition, total_blocks);
	}
	cout << "\n";
}

void print_action(const action& _action) {
	cout << _action.name;
	for (int x : _action.args)
		cout << " " << x;
}
#endif /*PRINT_CPP*/
