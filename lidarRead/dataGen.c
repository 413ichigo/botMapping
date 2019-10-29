#include "my.h"

int initMap();
int dataGen();
int readFile();
int mapGen();
int mapPrint();
int heuristic();    //the location of the robot is where the heuristic is least.
                    //the first number coresponds to the first filter/map overlay center.
double mse(int x,int y);

int data [2][360];
double dataRead [2][491];
int readData;
int element = 0;
int dividend = 25;

int size = 145;
int bigSize = 290;
int map [145][145];
int bigMap[290][290];
int heuristicMap [290-145][290-145];
char path [MAXPATHLEN] = "bathroom.csv";

int main(int argc, char const *argv[]) {
     //dataGen(); //only data input or readData();
     readFile();
     mapGen();
     mapPrint();
     //initMap();
     //heuristic();
     return 0;
}

int initMap(){

     for(int i = 0; i < size; i++){
          for(int j = 0; j < size; j++){
               bigMap[i][j]             = map[i][j];
               bigMap[i+size][j]        = map[i][j];
               bigMap[i][j+size]        = map[i][j];
               bigMap[i+size][j+size]   = map[i][j];
          }
     }
     for(int i = 0; i < bigSize; i++){
          for(int j = 0; j < bigSize; j++){
               printf("%d", bigMap[i][j]);
          }
          printf("\n");
     }

     printf("Map Initialized...\n");
     return 0;
}

int dataGen(){
     srand(time(0));
     int angle = 0;
     int dist = 0;
     int upper = floor(18*3.28);
     int lower = floor(14*3.28);
     for (int col = 0; col < 360; col ++){
          for (size_t row = 0; row < 2; row++) {

               if(row == 0){
                    data[row][col] =  angle;    //angle degree
                    //printf("%d \t:\t ", angle);
                    angle++;
               }else{
                    dist = (int) floor(((rand() % (upper-lower)) + 1)+lower);
                    data[row][col] = dist;     //distance in feet ()
                    //printf("%d\n", dist);
               }
          }
     }
     printf("Date Generated...\n");
     readData = 0;
     return 0;
}

int readFile(){

  printf("opening file...\n");
  FILE* fp = fopen(path, "r");
  char* line = NULL;
  size_t len = 0;
  int length = 0;

  if(fp == NULL){
       printf(ANSI_COLOR_RED"I'm afraid I can't let you do that, Dave"ANSI_COLOR_RESET"\n");
       printf("That file name must be incorrect or something. Try a .bMap file with the correct format.\n");
       return -1;
  }
  printf("file opened! reading...\n");
  int count = 0;
  while ((length=getline(&line, &len, fp)) != -1) {
       char* token = "initializer";
       int j = 0;
       printf("%d\n", count);
       while (token) {
         //printf("in the loop\n");
            if(j == 0){
                token = strtok(line,",\n");
                dataRead[0][element] = atof(token);
                printf("%f ", atof(token));
                j++;
            }else{
                dataRead[1][element] = atof(token);
                printf("%f \n", atof(token));
                break;
            }
            element++;
            token = strtok(NULL, ",\n");
       }

       count++;
       readData = 1;

     }
     printf("Data read!\n");
     return 0;
}



int mapGen(){
     int x=0;
     int y=0;
     printf("read data?... %d\n", readData);
     if(readData == 1){
       for(int i = 0; i < 491; i++){
          x = (int) 73 + (((dataRead[1][i])/dividend) * cos(0.0174533*dataRead[0][i]));
          y = (int) 73 + (((dataRead[1][i])/dividend) * sin(0.0174533*dataRead[0][i]));
          printf("i : %d\tr : %f\td : %f\tx : %d\ty : %d\tcos : %f\tsin : %f\n",i,dataRead[1][i],dataRead[0][i],x,y,cos(0.0174533*dataRead[0][i]),sin(0.0174533*dataRead[0][i]));
          if(!((x==73)&&(y==73))){
            map[x][y] = 1;
          }else{

          }
        }
     }else{
       for(int i = 0; i < 360; i++){
          x = (int) 73 + (data[1][i] * cos(0.0174533*data[0][i]));
          y = (int) 73 + (data[1][i] * sin(0.0174533*data[0][i]));
          printf("i : %d\tr : %d\td : %d\tx : %d\ty : %d\tcos : %f\tsin : %f\n",i,data[1][i],data[0][i],x,y,cos(0.0174533*data[0][i]),sin(0.0174533*data[0][i]));
          map[x][y] = 1;
        }
     }
     printf("Map generation complete..\n");
     return 0;
}

int mapPrint(){
     for(int i = 0; i < size; i++){
          for(int j = 0; j < size; j++){
               if(map[i][j] == 0){
                    printf("%d ", map[i][j]);
               }else{
                    printf(ANSI_COLOR_RED"%d "ANSI_COLOR_RESET, map[i][j]);
               }

               if(j==size-1){
                    printf("\n");
               }
          }
     }
     return 0;
}

int heuristic(){
     for(int i = 0; i < bigSize-size+1; i++){
          for(int j = 0; j < bigSize-size+1; j++){
               heuristicMap[i][j] = 100* mse(i,j)/((bigSize-size)*(bigSize-size));
               //printf("%d: %d:\t ", count, heuristicMap[i][j]);
          }
          //printf("\n");
     }
     for(int i = 0; i < bigSize-size+1; i++){
          for(int j = 0; j < bigSize-size+1; j++){
               if(heuristicMap[i][j] < 20){
                    printf(ANSI_COLOR_RED"%d "ANSI_COLOR_RESET, heuristicMap[i][j]);
               }else{
                    printf("%d ", heuristicMap[i][j]);
               }
          }
          printf("\n");
     }
     return 0;
}

double mse(int x,int y){
     //returns mean squared error of filter from full
     double error = 0.0;
     int cumSum = 0;
     for(int i = 0; i < size; i++){
          for(int j = 0; j < size; j++){
               cumSum = cumSum + abs(bigMap[i+x][j+y]-map[i][j]);
          }
     }
     error = cumSum;
     return error;
}
