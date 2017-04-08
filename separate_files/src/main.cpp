#include <action.hpp>
#include <planner.hpp>
#include <problem.hpp>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace std;

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
