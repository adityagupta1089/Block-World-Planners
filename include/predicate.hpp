#ifndef PREDICATE_HPP
#define PREDICATE_HPP

#include <stdlib.h>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

extern int TOTAL_BLOCKS;

using namespace std;

#define	predicate_on 1
#define	predicate_on_table 2
#define	predicate_clear 3
#define	predicate_hold 4
#define	predicate_empty 5
static const int TOTAL_PREDICATES = 5;

typedef int proposition;
typedef int predicate;

using namespace std;

#define M (max(TOTAL_BLOCKS, TOTAL_PREDICATES) + 1)

inline int type(proposition x) {
	return (x % M);
}

inline int var1(proposition x) {
	return ((x / M) % M);
}
inline int var2(proposition x) {
	return (x / (M * M));
}

extern map<string, predicate> propositions;

struct condition {
		predicate _predicate;
		vector<char> args;

		inline proposition value(int var1, int var2) {
			int k = M;
			int v = abs(_predicate);
			for (char c : args) {
				v += k * ((c == 'a') ? var1 : var2);
				k *= M;
			}
			return v;
		}
};
#endif /*PREDICATE_HPP*/
