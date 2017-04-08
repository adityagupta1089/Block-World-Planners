all:
	clang++ -std=c++11 -O2 main.cpp -o main.o
clean:
	rm -rf *.o
