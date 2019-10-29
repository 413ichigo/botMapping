#include "my.h"

int sysInit();
int movement(int act, int arg); //act 0: rotate, act 1: forward, arg:dist/deg
int scan(int rotations); //tells lidar to scan and reads the data into "sweep"
int mapGen();   //generates map (fuck you reid), JK generates "field" from "sweep"
int localize(); //compares X and Y of current location as well as angle from wall to one in memory



double [2][419][10] sweep; //up to ten rotations
int field[30][30];
int x;
int y;
double angle;


int main(int argc, char const *argv[]) {
  /* code */
  return 0;
}
