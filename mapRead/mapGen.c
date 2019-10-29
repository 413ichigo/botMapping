#include "my.h"



int main(int argc, char const *argv[]) {
  int height = 45;
  int width = 110;
  FILE* fp = fopen("hallwayTest.bMap","w+");

  for(int i = 0; i < height; i++){
    for(int j = 0; j < width; j++){
        fprintf(fp, "%d,%d,0,10\n", j, i);
    }
  }

}
