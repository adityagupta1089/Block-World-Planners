#ifndef PREDICATE_HPP
#define PREDICATE_HPP

#include <stdlib.h>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

using namespace std;

#define	predicate_on 1
#define	predicate_on_table 2
#define	predicate_clear 3
#define	predicate_hold 4
#define	predicate_empty 5
#define total_predicates 5

typedef int proposition;
typedef int predicate;

#define N (max(total_blocks, total_predicates) + 1)

#define type(x) (x % N)

#define var1(x) ((x / N) % N)
#define var2(x) (x / (N * N))

map<string, predicate> propositions =
	{
		{ "ontable", predicate_on_table },
		{ "on", predicate_on },
		{ "clear", predicate_clear },
		{ "hold", predicate_hold },
		{ "empty", predicate_empty }

	};

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
#endif /*PREDICATE_HPP*/
