
/*
  Stepper Motor Control - speed control

  This program drives a unipolar or bipolar stepper motor.
  The motor is attached to digital pins 8 - 11 of the Arduino.
  A potentiometer is connected to analog input 0.

  The motor will rotate in a clockwise direction. The higher the potentiometer value,
  the faster the motor speed. Because setSpeed() sets the delay between steps,
  you may notice the motor is less responsive to changes in the sensor value at
  low speeds.

  Created 30 Nov. 2009
  Modified 28 Oct 2010
  by Tom Igoe

*/

#include <Stepper.h>


//int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// initialize the stepper library on pins 8 through 11:
Stepper xStepper(200, 8, 9, 10, 11);
Stepper yStepper(200, 7, 6, 5, 4);   //TODO change ystepper pins



bool grids[34][34];

void setup() {
  const double gridSize = 1.5;  //size of a single grid square in mm
  const int sizeOfPetri = 50; //diameter of petri dish in mm
  Serial.begin(115200);
}

int roundUp(double x) {
  int intX = 1000 * x;
  if (intX % 1000 != 0) {
    return (intX / 1000) + 1;
  }
  return intX;
}

int loopCount = 0;

void midPointCircleDraw(int x_center, int y_center, int r) {
  int x = r;
  int y = 0;

  // Printing the initial point on the axes after translation
  //grids[x_center][y_center] = 1;

  // When radius is zero only a single
  // point will be printed
  if (r > 0) {
    grids[x + x_center][-y + y_center] = 1;
    grids[y + x_center][x + y_center] = 1;
    grids[-y + x_center][x + y_center] = 1;
  }

  // Initialising the value of P
  int P = 1 - r;
  while (x > y)
  {
    Serial.print("");
    y++;

    // Mid-point is inside or on the perimeter
    if (P <= 0)
      P = P + 2 * y + 1;
    // Mid-point is outside the perimeter
    else
    {
      x--;
      P = P + 2 * y - 2 * x + 1;
    }

    // All the perimeter points have already been printed
    if (x < y)
      break;

    // Printing the generated point and its reflection
    // in the other octants after translation
    grids[x + x_center][y + y_center] = 1;
    grids[-x + x_center][y + y_center] = 1;
    grids[x + x_center][-y + y_center] = 1;
    grids[-x + x_center][-y + y_center] = 1;

    // If the generated point is on the line x = y then
    // the perimeter points have already been printed
    if (x != y) {
      grids[y + x_center][x + y_center] = 1;
      grids[-y + x_center][x + y_center] = 1;
      grids[y + x_center][-x + y_center] = 1;
      grids[-y + x_center][-x + y_center] = 1;
    }
  }
}

void displayGrids() {
  for (int i = 0; i < 34; i++) {
    for (int j = 0; j < 34; j++) {
      Serial.print(String(grids[i][j]) + ", ");
    }
    Serial.println();
  }
}

void fillWithTrue(){
  bool checkBeg = 0;
  bool checkMid = 0;
  bool checkSta = 0;
  for(int i = 0; i < 34; i++){
    for(int j = 0; j < 34; j++){
      if(grids[i][j] && !grids[i][j+1]){
        if(checkBeg){
          checkBeg = 0;
        }else{
          checkBeg = 1;
        }
      }else if(checkBeg && i != 1 && i != 33){
        grids[i][j] = 1;
      }
    }
    checkBeg = 0;
  }
  
}

int pause = 1000;


//Inefficiency in how snake handles lower half of petri dish
void snake(int snakeSpeed, int spr, double stepSize){
  //TODO Add Homing Feature
  int x = 0;
  int y = 0; //Positive y means that it has gone down on the y axis
  for(int i = 0; i < 34; i++){
    if(i % 2 == 1){
      while(!grids[i][x]){
        moveX(-stepSize, snakeSpeed, spr);
        x--;
      }

      while(x >= 0 && grids[i][x]){
        grids[i][x] = 0;
        delay(pause);
        moveX(-stepSize, snakeSpeed, spr);
        x--;
      }
      if(x - 1 >= 0){
        if(x - 2 >= 0){
          moveX(-stepSize, snakeSpeed, spr);
          x--;
        }
        moveX(-stepSize, snakeSpeed, spr);
        x--;
      }
      
    }else{
      while(!grids[i][x]){
        moveX(stepSize, snakeSpeed, spr);
        x++;
      }
      
      while(x < 34 && grids[i][x]){
        grids[i][x] = 0;
        delay(pause);
        moveX(stepSize, snakeSpeed, spr);
        x++;
      }
      if(x + 1 < 34){
        if(x + 2 < 34){
          moveX(stepSize, snakeSpeed, spr);
          x++;
        }
        moveX(stepSize, snakeSpeed, spr);
        x++;
      }
    }
    moveY(-stepSize, snakeSpeed, spr);
    y++;
    
  }
}

void loop() {
  const int moveOnceSpeed = 25; //speed of stepper motor, % from 0-100
  const int stepsPerRevolution = 200;
  int half0 = round(34 / 2);
  int half1 = int(25 / 2);
  //0,0 cant be center of circle
  if (loopCount == 0) {
    midPointCircleDraw(half0, half0, half0 - 1);
    grids[17][1] = 1;
    grids[1][17] = 1;
    fillWithTrue();
    displayGrids();

    snake(moveOnceSpeed, stepsPerRevolution, 1.5);

    displayGrids();
    
    loopCount++;
  }
  
}

double revolutionsPerMM = 2; //Depends on thread size (pitch), how many times the motor must rotate to move one mm

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
