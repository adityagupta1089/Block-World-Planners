#ifndef PREDICATE_CPP
#define PREDICATE_CPP

#include <predicate.hpp>
#include <map>
#include <string>

using namespace std;

map<string, predicate> propositions =
	{
		{ "ontable", predicate_on_table },
		{ "on", predicate_on },
		{ "clear", predicate_clear },
		{ "hold", predicate_hold },
		{ "empty", predicate_empty }

	};

#endif /*PREDICATE_CPP*/
