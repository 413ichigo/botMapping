
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <time.h>
//#include <iostream>
//#include "RoboteqDevice.h"
//#include "ErrorCodes.h"
//#include "Constants.h"

using namespace std;

void forward(int fd){
	serialPuts(fd, "!G 1 500_!G 2 500_");
	usleep(10000);
}

void reverse(int fd){
	serialPuts(fd, "!G 1 -500_!G 2 -500_");
	usleep(10000);
}

void left(int fd){
	serialPuts(fd, "!G 1 -500_!G 2 500_");
	usleep(10000);
}

void right(int fd){
	serialPuts(fd, "!G 1 500_!G 2 -500_");
	usleep(10000);
}

void stop(int fd){
	serialPuts(fd, "!G 1 0_!G 2 0_");
	usleep(10000);
}

int main(int argc, char **argv)
{
	//int status = device.Connect("/dev/ttyS0");
	int fd;
	int choice;
	//sleepms(10);
	
	if((fd = serialOpen("/dev/ttyS0",115200)) < 0){
		fprintf(stderr, "unable to open serial device: %s\n", strerror(errno));
		printf("unable to open serial device\n");
		return 1;
	}
	if(wiringPiSetup() == -1){
		fprintf(stdout, "unable to start wiringPi: %s\n", strerror(errno));
		printf("unable to start wiringPi\n");
		return 2;
	}
	
	serialPuts(fd, "^rwd 0_");
	serialPuts(fd, "^rwd 0_");
	serialPuts(fd, "^rwd 0_");
	usleep(10000);
	
	stop(fd);
	
	while(1){
		
		printf("0:stop  1:forward  2:reverse  3:right  4:left\n");
		scanf("%d", &choice);
		switch(choice){
			case 0:
				stop(fd);
			break;
			case 1:
				forward(fd);
			break;
			case 2:
				reverse(fd);
			break;
			case 3:
				right(fd);
			break;
			case 4:
				left(fd);
			break;
			default:
				stop(fd);
		}
		
	}
	
	return 0;
}

