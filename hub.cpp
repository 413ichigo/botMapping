#include "my.h"
#include "sdk/sdk/include/rplidar.h" //RPLIDAR standard sdk, all-in-one header


//overhead functions
int sysInit();  //initialzes all hardware items and loads map from .bMap file
int move(int act, double arg); //act 0: rotate, act 1: forward, arg:dist/deg
int scan(int rotations); //tells lidar to scan and reads the data into "sweep"
int mapGen();   //generates map (fuck you reid), JK, generates "field" from "sweep"
int localize(); //compares X and Y of current location as well as angle from wall to one in memory
int findNext(); //finds X and Y positions of the next nearest unvisited destination.
int pathfind(); //A* or djikstra, generates instruction sets in "action" and "argument"
int speak(int room);  //the feature of the current square is passed and the appropriate speach is found


//map functions
int printMap(struct square[155][400]);


double sweep[2][419][10]; //up to ten rotations
int field[30][30];    //map generated from mapGen
int X = 104;  //current X position
int Y = 10;  //current Y position
int dest; //feature number destination
double angle; //current angle away from wall.
int nextX;  //destination X
int nextY;  //destination Y
double nextAngle;
double nextDist;
int action[5];  //list of actions to be sent to move
double arg[5];  //list of arguments to be sent to move


//map variables
int squareCount = 0;
char mapin [MAXPATHLEN] = "mapEdit1.bMap";
//char mapout [MAXPATHLEN] = "mapEdit1.bMap";
struct square squareList [62000];
struct square squareGraph[155][400];
int height = 400;
int width = 155;
int features [3][10]; //features are stored with feature num, X, and Y
int featureNum = 0;
struct Queue* Xqueue;
struct Queue* Yqueue;



//lidar read variables
int element = 0;
int dividend = 152.4;
int size = 145;
int bigSize = 290;
int map [145][145];
int bigMap[290][290];
int heuristicMap [290-145][290-145];
char lidarin [MAXPATHLEN] = "bathroom.csv";


int main(int argc, char const *argv[]) {

  //initialize hardware components
  printf("initializing hardware..\n");

  if(sysInit() == 1){
    printf("I didnt even fuckin start\n");
    return 1;
  }

  printf("Hardware initialized successfully!!\n");

  printf("Testing motors..\n");
  if((move(1,1) == 1) && (move(1,-1) == 1)){ //move forward and back
    printf("I'm a dumb robot and i cant move\n");
    return 1;
  }
  printf("Moved successfully!!\n");

  printf("Scanning and localizing..\n");

  if(scan(10) == 1){
    printf("I cant fuckin see!\n");
    return 1;
  }
  if(mapGen() == 1){
    printf("Location not found!\n");
    return 1;
  }

  printf("Scanning and localizing complete!! Currently at %d, %d, and I am %f degrees off from parallel\n", x, y, angle);


  //loop
  while(1){
    printf("Finding next waypoint..\n");
    if(findNext() == 1){
      printf("There is not a waypoint in driving distance that I have not visited!!\n");
      break;
    }
    printf("Found one! It is at %d, %d and is room %d\n", nextX, nextY, dest);
    while((x!=nextX) && (y != nextY)){
      pathfind(); //finds nextAngle and nextDist, a fraction of the total distance traveled
      move(1, nextAngle);
      move(0, nextDist);

      //position checking
      scan(5);
      mapGen();
      localize(); //updates current position if necessary
    }

    speak(dest);
    //end loop
    scan(5);
    mapGen();
    localize();
    return 1;
  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////
// METHOD sysInit  //////////////////////////////////////////////////////////////
// input:
// output: error codes if they exist. Will initialize all hardware items
//
/////////////////////////////////////////////////////////////////////////////////
int sysInit(){

  //nathan put your lidar initializing shit in here


  //map stuff

  FILE* fp = fopen(mapin,"r");    //originally fp = fopen(in->paths[k],"r"); opens the path at "path" for read
  char* line = NULL;
  size_t len = 0;
  int length = 0;

  if(fp == NULL){
       printf(ANSI_COLOR_RED "I'm afraid I can't let you do that, Dave" ANSI_COLOR_RESET "\n");
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

  printMap(squareGraph);
  return 0;
}


//////////////////////end sysInit()



/////////////////////////////////////////////////////////////////////////////////
// METHOD move  /////////////////////////////////////////////////////////////
// input: integer action- 0: move forward or backward, 1: rotate right or left
//        double argument- cast to int if action is 0, distance or angle measure
// output: error codes if they exist. May return motor feedback
//
/////////////////////////////////////////////////////////////////////////////////
int move(int act, double arg){
  if(act == 0){ //forward movement
    int d = (int)arg;
    //send command to motor controller here
  }else if(act == 1){ //rotational movement
    //convert angle to rotational distance needed to achieve degrees rotated
    //send command to motor controller here
  }else{
    printf("this is an improper function input, act must be a 0 or a 1\n", );
    return 1;
  }


  return 0;
}


/////////////////////end move()



/////////////////////////////////////////////////////////////////////////////////
// METHOD scan  /////////////////////////////////////////////////////////////////
// input: integer number of rotations to collect
// output: error codes if they exist. Reads lidar data into "sweep" variable
//
/////////////////////////////////////////////////////////////////////////////////
int scan(int rotations){

    return 0;
}


/////////////////////////end scan()



/////////////////////////////////////////////////////////////////////////////////
// METHOD mapGen  ///////////////////////////////////////////////////////////////
// input:
// output: uses "sweep" to generate a field that we can compare to the map in a reasonable fashion
//
/////////////////////////////////////////////////////////////////////////////////
int mapGen(){
  int x=0;
  int y=0;

  for(int i = 0; i < 491; i++){
      x = (int) 73 + (((dataRead[1][i])/dividend) * cos(0.0174533*dataRead[0][i]));
      y = (int) 73 + (((dataRead[1][i])/dividend) * sin(0.0174533*dataRead[0][i]));
      printf("i : %d\tr : %f\td : %f\tx : %d\ty : %d\tcos : %f\tsin : %f\n",i,dataRead[1][i],dataRead[0][i],x,y,cos(0.0174533*dataRead[0][i]),sin(0.0174533*dataRead[0][i]));
      if(!((x==73)&&(y==73))){
          map[x][y] = 1;
   }

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
  return 0;
}


////////////////////////////end mapGen()



/////////////////////////////////////////////////////////////////////////////////
// METHOD localize //////////////////////////////////////////////////////////////
// input:
// output: uses the preconstructed map and "field" to find where the robot currently is
//
/////////////////////////////////////////////////////////////////////////////////
int localize(){

  return 0;
}

////////////////////////////end localize()



/////////////////////////////////////////////////////////////////////////////////
// METHOD findNext //////////////////////////////////////////////////////////////
// input:
// output: focussed search to find the next nearest unvisited waypoint.
//
/////////////////////////////////////////////////////////////////////////////////
int findNext(){  //finds X and Y positions of the next nearest unvisited destination.
  if(firstWaypoint == 0){
    firstWaypoint = 1;
    return 0;
  }else{
    //find next
    nextY = Y+1;
    while (squareGraph[X][nextY] < 100){
      nextY = Y+1;
    }
  }
  return 0;
}

/////////////////////////////end findNext()



/////////////////////////////////////////////////////////////////////////////////
// METHOD pathfind  //////////////////////////////////////////////////////////////
// input:
// output: sets the nextDist and nextAngle values to the appropriate values for a point
//          between the current position and the next one
//
/////////////////////////////////////////////////////////////////////////////////
int pathfind(){
  nextDist = sqrt(((nextX-X)^2) + ((nextY-Y)^2));
  nextAngle = atan((double)(nextX-X)/(double)(nextY-Y))/0.0174533;
  printf("vectors set! angle of %f, for a distance of %f", nextAngle, nextDist);
  return 0;
}


//////////////////////////////end pathfind()



/////////////////////////////////////////////////////////////////////////////////
// METHOD speak  //////////////////////////////////////////////////////////////
// input: room number/feature found
// output: plays the appropriate audio clip for the given room number
//
/////////////////////////////////////////////////////////////////////////////////
int speak(int room){
  printf("speaking!!!\n");
  sleep(10);
  return 0;
}



int printMap(struct square map [155][400]){

     for(int i = height-1; i >= 0; i--){
          for(int j = 0; j < width; j++){
            if(map[j][i].weight != 0){
              printf(ANSI_COLOR_RED "%d " ANSI_COLOR_RESET, map[j][i].weight);
            }else if(map[j][i].feature == 100){
              printf(ANSI_COLOR_GREEN "%d " ANSI_COLOR_RESET, map[j][i].weight);
            }else if(map[j][i].feature > 100){
              printf(ANSI_COLOR_CYAN "%d " ANSI_COLOR_RESET, map[j][i].weight);
            }else{
              printf("%d ", map[j][i].weight);
            }
          }
          printf("\n");
     }

     return 0;
}
