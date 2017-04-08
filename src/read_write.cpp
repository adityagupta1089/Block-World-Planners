#ifndef READ_WRITE_CPP
#define READ_WRITE_CPP

#include <action.hpp>
#include <planner.hpp>
#include <problem.hpp>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int TOTAL_BLOCKS;

void read_input(char* file, problem& _problem) {
	ifstream in(file);
	string line;
	int line_num = 0;
	while (getline(in, line)) {
		switch (line_num) {
			case 0:
				TOTAL_BLOCKS = atoi(line.c_str());
				break;
			case 1:
				_problem.type = planners[line[0]];
				break;
			case 3:
				parse_propositions(line, _problem.initial_state);
				break;
			case 5:
				parse_propositions(line, _problem.goal_state);
				break;
			default:
				break;
		}
		line_num++;
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
#endif /*READ_WRITE_CPP*/
