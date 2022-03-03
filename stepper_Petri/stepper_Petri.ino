
/*
  Stepper Motor Control - XY Petri Dish

  TODO: Add Homing Functionality
        Switch Circle to Ellipse
        (Grid Sizes: [2.172, 1.577], [1.86, 1.351])
        Add button to change between the grid sizes

*/

#include <Stepper.h>


//int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// initialize the stepper library on pins 8 through 11:
Stepper xStepper(200, 8, 9, 10, 11);
Stepper yStepper(200, 4, 5, 6, 7);   //TODO change ystepper pins

unsigned short int arrSizeY = 28;
unsigned short int arrSizeX = 40;
bool grids[28][40];


void setup() {
  
  Serial.begin(9600);
}

int roundUp(double x) {
  short mod = 1000;
  unsigned int intX = mod *(x);
  int ret = round(x);
  if (intX % mod != 0) {
    ret = (intX / mod) + 1;
    return ret;
  }
  return intX;
}

short loopCount = 0;

void displayGrids() {
  for (int i = 0; i < arrSizeY; i++) {
    for (int j = 0; j < arrSizeX; j++) {
      Serial.print(String(grids[i][j]) + ", ");
    }
    Serial.println();
  }
}

void fillWithTrue() {
  bool checkBeg = 0;
  bool checkMid = 0;
  bool checkSta = 0;
  for (int i = 0; i < arrSizeY; i++) {
    for (int j = 0; j < arrSizeX; j++) {
      if (grids[i][j] && !grids[i][j + 1]) {
        if (checkBeg) {
          checkBeg = 0;
        } else {
          checkBeg = 1;
        }
      } else if (checkBeg && i != 1 && i != arrSizeY-1) {
        grids[i][j] = 1;
      }
    }
    checkBeg = 0;
  }

}

//void goHome(int spr){
//  bool isHome = false;
//  bool isHomeX = false;
//  bool isHomeY = false;
//  while(!isHome){
//    if(!isHomeX){
//      moveX(1, 75, spr)
//    }
//  }
//}

int pause = 1000;


//Inefficiency in how snake handles lower half of petri dish
void snake(int snakeSpeed, int spr, double stepX, double stepY) {
  //TODO Add Homing Feature
  int x = 0;
  int y = 0; //Positive y means that it has gone down on the y axis
  bool check = false;
  for (int i = 0; i < arrSizeY; i++) {
    for(int j = 0; j < arrSizeX; j++){
      if(grids[i][j] == 1){
        check = true;
        j = arrSizeX;
      }
    }
    if(check){
      if (i % 2 == 1) {
        while (!grids[i][x]) {
          moveX(-stepX, snakeSpeed, spr);
          x--;
        }
  
        while (x >= 0 && grids[i][x]) {
          grids[i][x] = 0;
          delay(pause);
          moveX(-stepX, snakeSpeed, spr);
          x--;
        }
        if (x - 1 >= 0) {
          if (x - 2 >= 0) {
            moveX(-stepX, snakeSpeed, spr);
            x--;
          }
          moveX(-stepX, snakeSpeed, spr);
          x--;
        }
  
      } else {
        while (!grids[i][x]) {
          moveX(stepX, snakeSpeed, spr);
          x++;
        }
  
        while (x < arrSizeX && grids[i][x]) {
          grids[i][x] = 0;
          delay(pause);
          moveX(stepX, snakeSpeed, spr);
          x++;
        }
        if (x + 1 < arrSizeX) {
          if (x + 2 < arrSizeX) {
            moveX(stepX, snakeSpeed, spr);
            x++;
          }
          moveX(stepX, snakeSpeed, spr);
          x++;
        }
      }
    }
    moveY(-stepY, snakeSpeed, spr);
    y++;

  }
}

//Write code
//Machine can only move to an accuracy of so much, round grid square size up to match tolerance
double roundToMin(double x){
  return x;
}

double revolutionsPerMM = 5; //Depends on thread size (pitch), how many times the motor must rotate to move one mm

void loop() {
  const short int moveOnceSpeed = 100; //speed of stepper motor, % from 0-100
  const short int stepsPerRevolution = 200;
  const double gridSizeX = 1.86; //2.172;           //size of the length (X) of a grid square in mm
  const double gridSizeY = 1.351; //1.577;           //size of the length (Y) of a grid square in mm
  const int sizeOfPetri = 50;               //diameter of petri dish in mm
  
  int centerX = (arrSizeY / 2);             //Center of grids
  int centerY = (arrSizeX / 2);
  
  double xSize = roundToMin(gridSizeX);
  double ySize = roundToMin(gridSizeY);
  Serial.println("CenterX: " + String(centerX));
  Serial.println("CenterY: " + String(centerY));
  //0,0 cant be center of circle
  if (loopCount == 0) {
    double temp = sizeOfPetri / xSize;
    short int tempX = roundUp(temp) / 2;
    temp = sizeOfPetri / ySize;
    short int tempY = roundUp(temp) / 2;

    drawEllipse(tempX, tempY, centerX, centerY);
    
    fillWithTrue();
    displayGrids();

    snake(moveOnceSpeed, stepsPerRevolution, gridSizeX, gridSizeY);

    displayGrids();

    loopCount++;
  }

}


//Moves right + dist (mm)
//Moves Left  - dist (mm)
void moveX(double dist, int s, int spr) {
  xStepper.setSpeed(s);
  double steps = (spr * revolutionsPerMM) * dist;
  xStepper.step(steps);
}

//Moves Up   + dist (mm)
//Moves Down - dist (mm)
void moveY(double dist, int s, int spr) {
  yStepper.setSpeed(s);
  double steps = (spr * revolutionsPerMM) * dist;
  yStepper.step(steps);
}

//Draw Ellipse for Petri Dish
//Major Radius, Minor Radius, Center Coordinates
void drawEllipse(int rx, int ry, int x_center, int y_center) {

  float dx, dy, d1, d2;
  int x = 0;
  int y = ry;

  // Initial decision parameter of region 1
  d1 = (ry * ry) - (rx * rx * ry) +
       (0.25 * rx * rx);
  dx = 2 * ry * ry * x;
  dy = 2 * rx * rx * y;

  // For region 1
  while (dx < dy){

    // Print points based on 4-way symmetry
    grids[x + x_center][y + y_center] = 1;
    grids[-x + x_center][y + y_center] = 1;
    grids[x + x_center][-y + y_center] = 1;
    grids[-x + x_center][-y + y_center] = 1;

    // Checking and updating value of
    // decision parameter based on algorithm
    if (d1 < 0){
      x++;
      dx = dx + (2 * ry * ry);
      d1 = d1 + dx + (ry * ry);
    }
    else{
      x++;
      y--;
      dx = dx + (2 * ry * ry);
      dy = dy - (2 * rx * rx);
      d1 = d1 + dx - dy + (ry * ry);
    }
  }

  // Decision parameter of region 2
  d2 = ((ry * ry) * ((x + 0.5) * (x + 0.5))) +
       ((rx * rx) * ((y - 1) * (y - 1))) -
       (rx * rx * ry * ry);

  // Plotting points of region 2
  while (y >= 0){
    
    // Print points based on 4-way symmetry
    grids[x + x_center][y + y_center] = 1;
    grids[-x + x_center][y + y_center] = 1;
    grids[x + x_center][-y + y_center] = 1;
    grids[-x + x_center][-y + y_center] = 1;

    // Checking and updating parameter
    // value based on algorithm
    if (d2 > 0){
      y--;
      dy = dy - (2 * rx * rx);
      d2 = d2 + (rx * rx) - dy;
    }else{
      y--;
      x++;
      dx = dx + (2 * ry * ry);
      dy = dy - (2 * rx * rx);
      d2 = d2 + dx - dy + (rx * rx);
    }
  }
}
