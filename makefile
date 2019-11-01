make: hub.cpp
	g++ -g -Wall -o start hub.cpp queue.c -lm

clean:
	rm hub
