# README

# Compiling
```bash
make all
```

# Running
```bash
./main.o <input-file> <output-file> <optinal heuristic/relevant action selector id>
```

## Optional Argument
If planning using Forward A\* Search: Heuristic ID:

* 0: Optimal Height Based Heuristic. 
* 1: Non-Optimal Total Goal Propositions Satisfied Count Based. **(default)**

If planning using Goal Stack Planner: Relevant Action Selector ID:

* 0: First Relevant Action Based.
* 1: Most Relevant Action Based.
* 2: Least Used Action with tie breaking with Most Relevant Action Based.
* 3: Manually coded Proposition-Action Tables Based. **(default)**
