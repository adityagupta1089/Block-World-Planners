all:
	g++ -std=c++11 -O0 main.cpp -o main0.o
	g++ -std=c++11 -O1 main.cpp -o main1.o
	g++ -std=c++11 -O2 main.cpp -o main2.o
	g++ -std=c++11 -O3 main.cpp -o main3.o
	g++ -std=c++11 -Ofast main.cpp -o mainfast.o
clean:
	rm -rf *.o