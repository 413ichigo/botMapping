#include "my.h"


int main(int argc, char const *argv[]) {
  int X = 0;
  int Y = 0;

  int nextX = 5;
  int nextY = 5;

  double dist = sqrt(((nextX-X)^2) + ((nextY-Y)^2));
  double angle = atan((double)(nextX-X)/(double)(nextY-Y));
  angle = angle/0.0174533;

  printf("%f\n", dist);
  printf("%f\n", angle);

  return 0;
}
