make: hub.c
	gcc -g -Wall -o start hub.c -lm

clean:
	rm hub
