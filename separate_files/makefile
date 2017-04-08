IDIR = include
CC = g++
CFLAGS = -I $(IDIR) -Ofast -std=c++11

ODIR = bin
SDIR = src

_DEPS = action.hpp forward_search.hpp planner.hpp predicate.hpp problem.hpp
DEPS = $(patsubst %, $(IDIR)/%, $(_DEPS))

_OBJ = action.o forward_search.o goal_stack.o heuristics.o predicate.o print.o problem.o read_write.o relevant_action.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))


$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(OBJ)
	$(CC) -o $(ODIR)/main.o $(SDIR)/main.cpp $(OBJ) $(CFLAGS) 
	
.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 
	