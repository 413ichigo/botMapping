#include "my.h"

//method headers
int readMap(char*);
int printMap(struct square[155][300]);
int writeBack(struct square[155][300]);
int editFeature(int x, int y, int featureNum);
int swath(int x, int y, int dir, int distance, int weight);
int sweep(int x, int y, int quad, int delX, int delY, int weight);

//global variables
int squareCount = 0;
char mapin [MAXPATHLEN] = "hallwayTest.bMap";
char mapout [MAXPATHLEN] = "../mapEdit1.bMap";
struct square squareList [46500];
struct square squareGraph[155][300];
int height = 300;
int width = 155;
int features [3][10]; //features are stored with feature num, X, and Y
int featureNum = 0;
int homeX = 0;
int homeY = 0;
struct Queue* Xqueue;
struct Queue* Yqueue;

int main(int argc, char const *argv[]) {

     Xqueue = createQueue(1000);
     Yqueue = createQueue(1000);

     //first, open the map and read the contents into a list.
     if(readMap(mapin) == -1){
          exit(1); //check if valid? maybe ask for user intervention at this point
     }
     //at this point, we have all the map, the square list and the feature list

     //map editing
     //swath: x, y, dir(0,1,2,3), distance(within bounds), weight
     //sweep: x, y, quad(1,2,3,4), delX, delY, weight);

     //swath(0, 0, 0, 300, 1);
     //swath(0, 0, 1, 155, 1);
     //sweep(0,43, 1, 155,2, 1);
     //sweep(25,20,1, 80, 5, 1);
     //sweep(30,20, 1, 75, 23, 1);
     //sweep(105,0, 1, 5, 300, 1);
     //editFeature(94, 10, 100);
     swath(10, 0, 0, 300, 1);
     swath(10, 0, 1, 145, 1);
     sweep(10,43, 1, 145,2, 1);
     sweep(35,20,1, 80, 5, 1);
     sweep(30,20, 1, 75, 23, 1);
     sweep(115,0, 1, 5, 300, 1);
     editFeature(104, 10, 100);
     //editFeature(71, 17, 202);
     //editFeature(58, 17, 203);
     //editFeature(49, 17, 204);
     //editFeature(43, 17, 205);


     //edit check
     printMap(squareGraph);


     /*
     for(int i = 0; i < featureNum; i++){
          printf("x = %d, y = %d, feature = %d\n", features[1][i], features[2][i], features[3][i]);
          printf("Xqueue: %d\n",dequeue(Xqueue));
          printf("Yqueue: %d\n",dequeue(Yqueue));
     }
     */
     writeBack(squareGraph);

/*
     if(findStart() == -1){
          exit(1);
     }
*/

     return 0;
}


/////////////////////////////////////////////////////////////////////////////////
// METHOD readMap  //////////////////////////////////////////////////////////////
// input: string path location of the map file to be readMap
// output: error codes if they exist. Will result in a populated squareList variables
//
/////////////////////////////////////////////////////////////////////////////////

int readMap(char* path){

     FILE* fp = fopen(path,"r");    //originally fp = fopen(in->paths[k],"r"); opens the path at "path" for read
     char* line = NULL;
     size_t len = 0;
     int length = 0;

     if(fp == NULL){
          printf(ANSI_COLOR_RED"I'm afraid I can't let you do that, Dave"ANSI_COLOR_RESET"\n");
          printf("That file name must be incorrect or something. Try a .bMap file with the correct format.\n");
          return -1;
     }

     while ((length=getline(&line, &len, fp)) != -1) {

          char* token = "initializer";
          struct square curSquare;
          int j = 0;
          while (token) {
               if(j == 0){
                    token = strtok(line,",\n");
                    //strcpy(curInstr.name, token);
                    curSquare.x = atoi(token);
                    //printf("%d ", atoi(token));
                    j++;
               }else if(j == 1){
                    //strcpy(curInstr.arg1, token);
                    curSquare.y = atoi(token);
                    //printf("%d ", atoi(token));
                    j++;
               }else if(j == 2){
                    //strcpy(curInstr.arg1, token);
                    curSquare.weight = atoi(token);
                    //printf("%d ", atoi(token));
                    j++;
               }else{
                    curSquare.feature = atoi(token);
                    //printf("%d \n", atoi(token));
                    break;
               }
               token = strtok(NULL, ",\n");
          }
          squareList[squareCount] = curSquare;
          squareGraph[curSquare.x][curSquare.y] = curSquare;

          //if location feature, add to location list
          if(curSquare.feature > 100){

               features[1][featureNum] = curSquare.x;
               enqueue(Xqueue, curSquare.x);
               features[2][featureNum] = curSquare.y;
               enqueue(Yqueue, curSquare.y);
               features[3][featureNum] = curSquare.feature;
               featureNum = featureNum + 1;

          }

          //if 0, add home

          squareCount++;
     }

     return 0;
}

// END METHOD readMap
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// METHOD swath  //////////////////////////////////////////////////////////////
// input: map generated from the readMap function
// output: error codes if they exist. Will print the weights in a well formatted grid
//
/////////////////////////////////////////////////////////////////////////////////

int swath(int x, int y, int dir, int distance, int weight){

    if(dir == 0){//paint weights up
      for(int i = 0; i < distance; i++){
          squareGraph[x][y+i].weight = weight;
      }
    }else if(dir == 1){//paint weights right
      for(int i = 0; i < distance; i++){
          squareGraph[x+i][y].weight = weight;
      }
    }else if(dir == 2){//paint weights down
      for(int i = 0; i < distance; i++){
          squareGraph[x][y-i].weight = weight;
      }
    }else if(dir == 3){//paint weights left
      for(int i = 0; i < distance; i++){
          squareGraph[x-i][y].weight = weight;
      }
    }else{
      printf("There was some invalid argument to swath. It is a dumb function, please check the documentation!");
      return 1;
    }
    return 0;
}

// END METHOD swath
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// METHOD sweep  //////////////////////////////////////////////////////////////
// input: map generated from the readMap function
// output: error codes if they exist. Will print the weights in a well formatted grid
//
/////////////////////////////////////////////////////////////////////////////////

int sweep(int x, int y, int quad, int delX, int delY, int weight){

  if(quad == 1){
    for(int i = 0; i < delY; i ++){
      swath(x, y+i, 1, delX, weight);
    }
  }else if(quad == 2){
    for(int i = 0; i < delY; i ++){
      swath(x, y+i, 3, delX, weight);
    }
  }else if(quad == 3){
    for(int i = 0; i < delY; i ++){
      swath(x, y-i, 3, delX, weight);
    }
  }else if(quad == 4){
    for(int i = 0; i < delY; i ++){
      swath(x, y-i, 1, delX, weight);
    }
  }else{
    printf("work with me here, pal. Those arent valid arguments for sweep!");
    return 1;
  }

  return 0;
}

// END METHOD sweep
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// METHOD editFeature  //////////////////////////////////////////////////////////////
// input: map generated from the readMap function
// output: error codes if they exist. Will print the weights in a well formatted grid
//
/////////////////////////////////////////////////////////////////////////////////

int editFeature(int x, int y, int featureNum){
  for(int i = x-2; i < x+1; i ++){
    for(int j = y-2; j < y+1; j++){
        squareGraph[i][j].feature = featureNum;
    }
  }
  return 0;
}

// END METHOD swath
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// METHOD printMap  //////////////////////////////////////////////////////////////
// input: map generated from the readMap function
// output: error codes if they exist. Will print the weights in a well formatted grid
//
/////////////////////////////////////////////////////////////////////////////////

int printMap(struct square map [155][300]){

     for(int i = height-1; i >= 0; i--){
          for(int j = 0; j < width; j++){
            if(map[j][i].weight != 0){
              printf(ANSI_COLOR_RED"%d "ANSI_COLOR_RESET, map[j][i].weight);
            }else if(map[j][i].feature == 100){
              printf(ANSI_COLOR_GREEN"%d "ANSI_COLOR_RESET, map[j][i].weight);
            }else if(map[j][i].feature > 100){
              printf(ANSI_COLOR_CYAN"%d "ANSI_COLOR_RESET, map[j][i].weight);
            }else{
              printf("%d ", map[j][i].weight);
            }
          }
          printf("\n");
     }

     return 0;
}

// END METHOD printMap
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// METHOD writeBack  ////////////////////////////////////////////////////////////
// input: map generated from the readMap function
// output: error codes if they exist. Will write the map to a text file, readable
//         by the readMap function
//
/////////////////////////////////////////////////////////////////////////////////

int writeBack(struct square map[155][300]){

     FILE* mapOutput =fopen(mapout, "w+"); //opens a file for writing and reading
     //will rewrite the map entirely each time
     //printf("%d,%d,%d,%d\n", map[0][0].x, map[0][0].y, map[0][0].weight, map[0][0].feature);
     for(int q = 0; q < 155; q++){
          for(int p = 0; p < 300; p++){
               fprintf(mapOutput, "%d,%d,%d,%d\n", squareGraph[q][p].x, squareGraph[q][p].y, squareGraph[q][p].weight, squareGraph[q][p].feature);
          }
     }
     return 0;
}

// END METHOD printMap
/////////////////////////////////////////////////////////////////////////////////
