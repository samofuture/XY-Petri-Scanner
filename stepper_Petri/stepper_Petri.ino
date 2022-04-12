
/*
  Stepper Motor Control - XY Petri Dish

  Grid Sizes: [2.172, 1.577], [1.86, 1.351]
        
*/

#include <AccelStepper.h>

//Pins for stepper drivers
#define dirPinX 2
#define stepPinX 3
#define dirPinY 4     //TODO change ystepper pins
#define stepPinY 5

//Limit switch pins for homing
#define switchPinX 7
#define switchPinY 8

int stateX = 0;
int stateY = 0;

// Switch
#define pausePin 9

int stateP = 0;

//Size Switch
//On = Bigger Grid Size [2.172, 1.577], Off = Smaller Grid Size[1.86, 1.351]
#define sizePin 10

double ySize = (1.351);
double xSize = (1.86);

#define stepsPerRevolution 200

#define pitch 5

#define SPEED 200 //Steps per second

#define sleepPinX 12
#define sleepPinY 13

//Position in steps, pos[0] = x, pos[1] = y
long pos[] = {0, 0};

// initialize the stepper library on pins 8 through 11:
//AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
//Motor interface type must be set to 1 when using a driver:
AccelStepper xStepper(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper yStepper(AccelStepper::DRIVER, stepPinY, dirPinY);


#define arrSizeY 28
#define arrSizeX 40

bool grids[arrSizeY][arrSizeX];

//TODO Find where the microscope is
unsigned short int camLocation;

short loopCount = 0;

#define pause 1250

void setup() {

  Serial.begin(9600);

  //Limit Switch Pin Setup
  pinMode(switchPinX, INPUT);
  pinMode(switchPinY, INPUT);

  //Pause Switch Setup
  pinMode(pausePin, INPUT);

  //Grid Size Switch Setup
  pinMode(sizePin, INPUT);

  //Sleep Pin Setup
  pinMode(sleepPinX, OUTPUT);
  pinMode(sleepPinY, OUTPUT);
  
  digitalWrite(sleepPinX, HIGH);
  digitalWrite(sleepPinY, HIGH);

  //Set Max Speeds for the steppers (max steps per second)
  xStepper.setMaxSpeed(500);
  yStepper.setMaxSpeed(500);

  xStepper.setAcceleration(20);
  yStepper.setAcceleration(20);

  camLocation = distanceToSteps(75);
  
  //Home the petri dish to 0, 0
  bool isHome = false;
//  while (!isHome) {
//    isHome = homePetri();
//    Serial.println("Not Home");
//  }
//  Serial.println("Is Home");

}

void loop() {
  readStates();
//  delay(500);
//  const double gridSizeY = 1.86; //2.172;           //size of the length (X) of a grid square in mm
//  const double gridSizeX = 1.351; //1.577;           //size of the length (Y) of a grid square in mm
  const int sizeOfPetri = 50;               //diameter of petri dish in mm

  int centerX = (arrSizeY / 2);             //Center of grids
  int centerY = (arrSizeX / 2);



  //0,0 cant be center of circle
  if (loopCount == 0) {
    if(digitalRead(sizePin) == HIGH){
      xSize = 1.577;
      ySize = 2.172;
    }
    
    double temp = sizeOfPetri / xSize;
    short int tempX = roundUp(temp) / 2;
    temp = sizeOfPetri / ySize;
    short int tempY = roundUp(temp) / 2;

    drawEllipse(tempX, tempY, centerX, centerY);

    fillWithTrue();
    displayGrids();
        
    snake();
    Serial.println(F("After Snake:"));
    displayGrids();
    digitalWrite(sleepPinX, LOW);
    loopCount++;
  }

}

/* readStates updates the positions of the switches
 * Parameters: None
 * Returns void
 */
void readStates() {
  stateX = digitalRead(switchPinX);
  stateY = digitalRead(switchPinY);
  stateP = digitalRead(pausePin);
}


//Movement Functions

/* homePetri moves stepper motors to the home position
 * Parameters: None
 * Returns boolean (true if it is at home)
 */

#define gap 5       //Distance in mm that the thing has to be before it stops pressing limit switch 
#define fineTune 2  //Number of steps for the fine tuning
 
bool homePetri() {
  readStates();
  while(stateP == LOW){
    readStates();
  }
  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);
//  Serial.println("Before Checking Switches");

  //Initial Move to limit switch
  while (stateX != LOW) {
    long temp = pos[0] - distanceToSteps(1);
    moveX(temp);
    digitalWrite(sleepPinX, LOW);
    pos[0] = temp;
    do{
      readStates();
    }while(stateP == LOW);
    digitalWrite(sleepPinX, HIGH);
    delay(5);
  }
  
  //Move to reset the limit switch
  long tempGap = pos[0] + distanceToSteps(gap);
  moveX(tempGap);
  pos[0] = tempGap;
  
  //Move Slower to fine tune hitting the limit switch
  while (stateX != LOW) {
    long temp = pos[0] - fineTune;
    moveX(temp);
    digitalWrite(sleepPinX, LOW);
    pos[0] = temp;
    do{
      readStates();
    }while(stateP == LOW);
    digitalWrite(sleepPinX, HIGH);
    delay(5);
  }
  
  //Y Homing
  
  //Initial Move to limit switch
  while (stateY != LOW) {
    long temp = pos[1] - distanceToSteps(1);
    moveY(temp);
    digitalWrite(sleepPinY, LOW);
    pos[1] = temp;
    do{
      readStates();
    }while(stateP == LOW);
    digitalWrite(sleepPinY, HIGH);
    delay(5);
  }
  
  //Move to reset the limit switch
  tempGap = pos[1] + distanceToSteps(gap);
  moveY(tempGap);
  pos[1] = tempGap;
  
  //Move Slower to fine tune hitting the limit switch
  while (stateY != LOW) {
    long temp = pos[1] - fineTune;
    moveY(temp);
    digitalWrite(sleepPinY, LOW);
    pos[1] = temp;
    do{
      readStates();
    }while(stateP == LOW);
    digitalWrite(sleepPinY, HIGH);
    delay(5);
  }

  pos[0] = distanceToSteps(0);
  pos[1] = distanceToSteps(0);
  xStepper.setCurrentPosition(pos[0]);
  yStepper.setCurrentPosition(pos[1]);
  return true;
}

/* MoveX / MoveY moves the corresponding stepper motor to a step position
 * Parameters: long steps, the desired position of the stepper motor
 * Returns void
 */
void moveX(long steps){
  Serial.println("(X) Moving to: " + String(steps) );
  xStepper.setCurrentPosition(pos[0]);
  while(xStepper.currentPosition() != steps){
    if(steps < pos[0]){
      xStepper.setSpeed(-SPEED);
    }else{
      xStepper.setSpeed(SPEED);
    }
    xStepper.runSpeed();
  }
}
void moveY(long steps){
  Serial.println("(Y) Moving to: " + String(steps) );
  yStepper.setCurrentPosition(pos[1]);
  while(yStepper.currentPosition() != steps){
    if(steps < pos[1]){
      yStepper.setSpeed(-SPEED);
    }else{
      yStepper.setSpeed(SPEED);
    }
    yStepper.runSpeed();
  }
}

/* wait
 * Parameters: char axis (which motor is being paused)
 * Returns void
 */
void wait(char axis){
  if(axis == 'X'){
    digitalWrite(sleepPinX, LOW);
  }else if(axis == 'Y'){
    digitalWrite(sleepPinY, LOW);
  }
  delay(pause);
  if(axis == 'X'){
    digitalWrite(sleepPinX, HIGH);
    delay(5);                        //delay for DRV8825 to wake back up
  }
}

//Assumed that camera is in top right of the petri dish array (grids[arrSizeX][arrSizeY])
/* snake moves the petri dish through the grid of squares in a snake like fashion (through one row, then move down a column, repeat)
 * Parameters: None
 * Returns void
 */
void snake() {
  readStates();
  while(stateP == LOW){
    readStates();
  }
  
  long xSteps = distanceToSteps(ySize);
  long ySteps = distanceToSteps(xSize);
  double halfX = xSteps * arrSizeX / 2;
  
  for(int y = 0; y < arrSizeY; y++){
    //Check if there is a searchable grid in the row, and checks for what side of the dish it is on
    bool validRow = false;
    int tempX = 0;
    if(pos[0] < halfX){
      while(tempX < arrSizeX && !grids[y][tempX]){
        tempX++;
      }
    }else{
      tempX = arrSizeX - 1;
      while(tempX >= 0 && !grids[y][tempX]){
        tempX--;
      }
    }
    if(grids[y][tempX]){
      validRow = true;
    }
    
    if (validRow) {
      //Move to where tempX is
      pos[0] = tempX * xSteps;
      moveX(pos[0]);
      wait('X');
      if (tempX < arrSizeX / 2){
        //Move right through the row
        while(tempX < arrSizeX && grids[y][tempX]){
//          Serial.println(F("Move Right"));
          long temp = pos[0] + xSteps;
          moveX(temp);
          pos[0] = temp;
          wait('X');
          grids[y][tempX] = false;
          tempX++;
        }
      } else{        
        //Move left through the row
        while(tempX >= 0 && grids[y][tempX]){
//          Serial.println(F("Move Left"));
          long temp = pos[0] - xSteps;
          moveX(temp);
          pos[0] = temp;
          wait('X');
          grids[y][tempX] = false;
          tempX--;
        }
      }
    }
    //Move Down one row
    digitalWrite(sleepPinY, HIGH);
    delay(5);
    long temp = pos[1] - ySteps;
    moveY(temp);
    pos[1] = temp;
    wait('Y');
  }
}

/*  fillWithTrue Fills in the ellipse with true
 *  Parameters: none
 *  Returns none
 */
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
      } else if (checkBeg && i != 1 && i != arrSizeY - 1) {
        grids[i][j] = 1;
      }
    }
    checkBeg = 0;
  }

}

//Custom Math Functions

/* distanceToSteps calculates the steps to take with a given distance
 * Parameters: double distance (the distance in mm)
 * Returns long (number of steps to move the distance)
 */
long distanceToSteps(double distance) {
  double temp = distance / pitch;
  temp = temp * stepsPerRevolution;
  long ret = round(temp);
  return ret;
}

/* roundUp rounds a double up to the next int
 * Parameters: double x (value to be rounded up)
 * Returns int (rounded value)
 */
int roundUp(double x) {
  short mod = 1000;
  unsigned int intX = mod * (x);
  int ret = round(x);
  if (intX % mod != 0) {
    ret = (intX / mod) + 1;
    return ret;
  }
  return intX;
}

/* drawEllipse draws an ellipse on a grid of boolean values
 * Parameters: int rx (major radius), int ry (minor radius), int x_center, int y_center (center of the ellipse coordinates)
 * Returns void
 */
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
  while (dx < dy) {

    // Print points based on 4-way symmetry
    grids[x + x_center][y + y_center] = 1;
    grids[-x + x_center][y + y_center] = 1;
    grids[x + x_center][-y + y_center] = 1;
    grids[-x + x_center][-y + y_center] = 1;

    // Checking and updating value of
    // decision parameter based on algorithm
    if (d1 < 0) {
      x++;
      dx = dx + (2 * ry * ry);
      d1 = d1 + dx + (ry * ry);
    }
    else {
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
  while (y >= 0) {

    // Print points based on 4-way symmetry
    grids[x + x_center][y + y_center] = 1;
    grids[-x + x_center][y + y_center] = 1;
    grids[x + x_center][-y + y_center] = 1;
    grids[-x + x_center][-y + y_center] = 1;

    // Checking and updating parameter
    // value based on algorithm
    if (d2 > 0) {
      y--;
      dy = dy - (2 * rx * rx);
      d2 = d2 + (rx * rx) - dy;
    } else {
      y--;
      x++;
      dx = dx + (2 * ry * ry);
      dy = dy - (2 * rx * rx);
      d2 = d2 + dx - dy + (rx * rx);
    }
  }
}

/* displayGrids displays the grid
 * Parameters: none
 * Returns void
 */
void displayGrids() {
  for (int i = 0; i < arrSizeY; i++) {
    for (int j = 0; j < arrSizeX; j++) {
      String temp = String(grids[i][j]);
      Serial.print(temp + ", ");
    }
    Serial.println();
  }
}
