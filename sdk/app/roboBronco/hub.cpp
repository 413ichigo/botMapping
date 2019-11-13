#include "my.h"
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <iostream>//cant be in my.h because its only for cpp files
#include <fstream>//cant be in my.h because its only for cpp files

//overhead functions
int sysInit();  //initialzes all hardware items and loads map from .bMap file
int move(int act, double arg); //act 0: rotate, act 1: forward, arg:dist/deg
int scan(); //tells lidar to scan and reads the data into "sweep"
int mapGen();   //generates map (fuck you reid), JK, generates "field" from "sweep"
int localize(); //compares X and Y of current location as well as angle from wall to one in memory
int findNext(); //finds X and Y positions of the next nearest unvisited destination.
int pathfind(); //A* or djikstra, generates instruction sets in "action" and "argument"
int speak(int room);  //the feature of the current square is passed and the appropriate speach is found


// mid level motor control functions
int maintainForward();
int nudgeRight(); //push right wheel forward a little faster for a second.
int nudgeLeft();  //push  left wheel forward a little faster for a second.
int quickRotateR(); //stops and rotates right
int quickRotateL(); //stops and rotates left
//low level motor control functions
void forward(int fd);
void reverse(int fd);
void right(int fd);
void left(int fd);
void stop(int fd);

//map functions
int printMap(struct square[155][400]);


double sweep[2][419]; //up to ten rotations
int fd;
int choice;
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
int firstWaypoint = 0;
double wallDist = 5000;
double wallAng = 0;

//char mapout [MAXPATHLEN] = "mapEdit1.bMap";
struct square squareList [62000];
struct square squareGraph[155][400];
int height = 400;
int width = 155;
int features [3][10]; //features are stored with feature num, X, and Y
int featureNum = 0;
struct Queue* Xqueue;
struct Queue* Yqueue;

//lidar variables and stuff
using namespace rp::standalone::rplidar;
static inline void delay(_word_size_t ms);
bool checkRPLIDARHealth(RPlidarDriver * drv);
void ctrlc(int);
bool ctrl_c_pressed;
const char * opt_com_path = "/dev/ttyUSB0";//lidar port
_u32         baudrateArray[2] = {115200, 256000};
_u32         opt_com_baudrate = 0;
u_result     op_result;
bool useArgcBaudrate = false;
RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
rplidar_response_measurement_node_hq_t nodes[8192];//added
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
size_t   count = _countof(nodes);//added

//motor controller variables

//lidar read variables
double dataRead[2][491];
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
  signal(SIGINT, ctrlc);
  if(sysInit() == 1){
    printf("I didnt even fuckin start\n");
    goto on_finished;
  }

  printf("Hardware initialized successfully!!\n");

  printf("Testing motors..\n");
  if((move(1,1) == 1) && (move(1,-1) == 1)){ //move forward and back
    printf("I'm a dumb robot and i cant move\n");
    return 1;
  }
  printf("Moved successfully!!\n");

  printf("Scanning and localizing..\n");

  if(scan() == 1){
    printf("I cant fuckin see!\n");
    return 1;
  }
  if(mapGen() == 1){
    printf("Location not found!\n");
    return 1;
  }

  printf("Scanning and localizing complete!! Currently at %d, %d, and I am %f degrees off from parallel\n", X, Y, angle);


  //main loop
  while(1){
    printf("Finding next waypoint..\n");
    if(findNext() == 1){
      printf("There is not a waypoint in driving distance that I have not visited!!\n");
      break;
    }
    printf("Found one! It is at %d, %d and is room %d\n", nextX, nextY, dest);
    while((X!=nextX) && (Y != nextY)){
      pathfind(); //finds nextAngle and nextDist, a fraction of the total distance traveled
      move(1, nextAngle);
      move(0, nextDist);

      //position checking

      scan();
      drv->stop();
      drv->stopMotor();
      //mapGen();
      //localize(); //updates current position if necessary
      return 1;
    }

    //speak(dest);
    //end loop
    scan();
    //mapGen();
    //localize();

    //for when kill signal is found exit nicely and shut stuff down
    if (ctrl_c_pressed){
        break;
    }
    //return 1;//why is this here??
  }


  //for when program ends
  drv->stop();
  drv->stopMotor();
  // done!
on_finished:
  RPlidarDriver::DisposeDriver(drv);
  drv = NULL;
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
  if (!drv) {
      fprintf(stderr, "insufficent memory, exit\n");
      exit(-2);
  }
  rplidar_response_device_info_t devinfo;
  bool connectSuccess = false;
  // make connection...
  if(useArgcBaudrate)
  {
      if(!drv)
          drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
      if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
      {
          op_result = drv->getDeviceInfo(devinfo);

          if (IS_OK(op_result))
          {
              connectSuccess = true;
          }
          else
          {
              delete drv;
              drv = NULL;
          }
      }
  }
  else
  {
      size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
      for(size_t i = 0; i < baudRateArraySize; ++i)
      {
          if(!drv)
              drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
          if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
          {
              op_result = drv->getDeviceInfo(devinfo);

              if (IS_OK(op_result))
              {
                  connectSuccess = true;
                  break;
              }
              else
              {
                  delete drv;
                  drv = NULL;
              }
          }
      }
  }

  if (!connectSuccess) {

      fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
          , opt_com_path);
      return 1;
  }

  printf("RPLIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
      printf("%02X", devinfo.serialnum[pos]);
  }

  printf("\n"
          "Firmware Ver: %d.%02d\n"
          "Hardware Rev: %d\n"
          , devinfo.firmware_version>>8
          , devinfo.firmware_version & 0xFF
          , (int)devinfo.hardware_version);



  // check health...
  if (!checkRPLIDARHealth(drv)) {
      return 1;
  }


  drv->startMotor();
  drv->startScan(0,1);
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

       const char * token = "initializer"; //if things break here remove const
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

            features[0][featureNum] = curSquare.x;
            //enqueue(Xqueue, curSquare.x);
            features[1][featureNum] = curSquare.y;
            //enqueue(Yqueue, curSquare.y);
            features[2][featureNum] = curSquare.feature;
            featureNum = featureNum + 1;

       }

       //if 0, add home

       squareCount++;
  }

  //printMap(squareGraph);

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
    printf("this is an improper function input, act must be a 0 or a 1\n" );
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
int scan(){
  rplidar_response_measurement_node_hq_t nodes[8192];
  size_t   count = _countof(nodes);

  op_result = drv->grabScanDataHq(nodes, count);
	//usleep(4000000);
  if (IS_OK(op_result)) {
      drv->ascendScanData(nodes, count);
      for (int pos = 0; pos < (int)count ; pos++) {
        //myfile << nodes[pos].angle_z_q14 * 90.f / (1 << 14) << "," << nodes[pos].dist_mm_q2/4.0f << "\n";
        
        sweep[0][pos] = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
        if(nodes[pos].dist_mm_q2/4.0f == 0){
			sweep[1][pos] = 100000;
		}
		else{
			sweep[1][pos] = nodes[pos].dist_mm_q2/4.0f;
		}
        //printf("angle: %03.2f Dist: %8.0f\n", sweep[0][pos], sweep[1][pos]);
          /*printf("%s theta: %03.2f Dist: %8.0f Q: %d pos: %d\n",
              (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
              (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
              nodes[pos].dist_mm_q2/4.0f,
              nodes[pos].quality, pos);*/
      }
    }
    //finds minimum distance of object from lidar. Should be the wall
    for(int f = 0; f < 491; ++f){
      if(sweep[1][f] < wallDist){
        wallDist = sweep[1][f];
        wallAng = sweep[0][f];
      }
    }
    //results in wallDist holding the distance from the wall and wallAng holding the angle of that measurement

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
  }

   for(int i = 0; i < size; i++){
        for(int j = 0; j < size; j++){
             if(map[i][j] == 0){
                  printf("%d ", map[i][j]);
             }else{
                  printf( ANSI_COLOR_RED " %d " ANSI_COLOR_RESET, map[i][j]);
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
    while (squareGraph[X][nextY].feature < 100){
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

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

void ctrlc(int)
{
    ctrl_c_pressed = true;
}

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
