IDIR =include
CC=clang++
CFLAGS=-I$(IDIR) -Ofast -std=c++11

ODIR=bin
SDIR=src

_DEPS = action.hpp forward_search.hpp planner.hpp predicate.hpp problem.hpp
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = action.o forward_search.o goal_stack.o heuristics.o print.o problem.o read_write.o relevant_action.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
	