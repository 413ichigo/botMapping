#ifndef _A2_H
#define _A2_H

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <ctype.h>
#include <limits.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pthread.h> //if older use -lpthread flag in compilitaion
#include <unistd.h>
#include <error.h>
#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <time.h>

//pi UART libraries
#include <wiringPi.h>
#include <wiringSerial.h>

//linix UART libraries
#include <fcntl.h>
#include <termios.h>

#define MAXPATHLEN 1024

//couple of colors
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

struct Queue* createQueue(unsigned capacity);
int isFull(struct Queue* queue);
int isEmpty(struct Queue* queue);
void enqueue(struct Queue* queue, int item);
int dequeue(struct Queue* queue);
int front(struct Queue* queue);
int rear(struct Queue* queue);

struct Queue
{
	int front, rear, size;
	unsigned capacity;
	int* array;
};

struct square
{ //see map README for documentation
     int x;
     int y;
     int weight;
     int feature;
};

struct squareNode
{
     struct squareNode* north;
     struct squareNode* south;
     struct squareNode* east;
     struct squareNode* west;
     struct square* element;
};

#endif
