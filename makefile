make: hub.c
	gcc -g -Wall -o start hub.c queue.c -lm

clean:
	rm hub
