make: hub.cpp
	g++ -g -Wall -o start hub.cpp queue.c -lm #-lwiringPi

clean:
	rm hub

run:
	sudo ./start
