
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
#define pausePin 10

int stateP = 0;

//Size Switch
//On = Bigger Grid Size [2.172, 1.577], Off = Smaller Grid Size[1.86, 1.351]
#define sizePin 11

//Defaults to smaller size
double stepX = 1.351;
double stepY = 1.86;

const int stepsPerRevolution = 200;

#define SPEED 200 //Steps per second

#define sleepPinX 13
#define sleepPinY 14

//Position in steps, pos[0] = x, pos[1] = y
long pos[] = {0, 0};

// initialize the stepper library on pins 8 through 11:
//AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
//Motor interface type must be set to 1 when using a driver:
AccelStepper xStepper(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper yStepper(AccelStepper::DRIVER, stepPinY, dirPinY);


const unsigned short int arrSizeY = 28;
const unsigned short int arrSizeX = 40;
bool grids[28][40];

//TODO Find where the microscope is
unsigned short int camLocation;

short loopCount = 0;

short unsigned int pause = 500;

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
  while (!isHome) {
    isHome = homePetri();
  }

}

void loop() {
  readStates();
  const double gridSizeY = 1.86; //2.172;           //size of the length (X) of a grid square in mm
  const double gridSizeX = 1.351; //1.577;           //size of the length (Y) of a grid square in mm
  const int sizeOfPetri = 50;               //diameter of petri dish in mm

  int centerX = (arrSizeY / 2);             //Center of grids
  int centerY = (arrSizeX / 2);

  double xSize = (gridSizeX);
  double ySize = (gridSizeY);

  //0,0 cant be center of circle
  if (loopCount == 0) {
    if(digitalRead(sizePin) == HIGH){
      stepX = 1.577;
      stepY = 2.172;
      xSize = stepX;
      ySize = stepY;
    }
    
    double temp = sizeOfPetri / xSize;
    short int tempX = roundUp(temp) / 2;
    temp = sizeOfPetri / ySize;
    short int tempY = roundUp(temp) / 2;

    drawEllipse(tempX, tempY, centerX, centerY);

    fillWithTrue();
    displayGrids();

    snake();
    Serial.println("After Snake:");
    displayGrids();

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
bool homePetri() {
  readStates();
  while(stateP == LOW){
    readStates();
  }
  xStepper.setCurrentPosition(0);
  yStepper.setCurrentPosition(0);
  
  //While the x or y switches are not pressed
  while (stateX == LOW || stateY == LOW) {
    //decrement position until the limit switch is hit
    long tempX, tempY;
    if (stateX != HIGH)
      tempX = pos[0] - 1;
    if (stateY != HIGH)
      tempY = pos[1] - 1;

    //Move both x and y steppers simultaneously
    while(xStepper.currentPosition() != tempX && yStepper.currentPosition() != tempY){
      if(tempX < pos[0]){
        xStepper.setSpeed(-SPEED);
      }else{
        xStepper.setSpeed(SPEED);
      }
      if(tempY < pos[0]){
        yStepper.setSpeed(-SPEED);
      }else{
        yStepper.setSpeed(SPEED);
      }
    }

    //If x or y gets home before the other one
    if(stateY != HIGH)
      moveY(tempY);
    if(stateX != HIGH)
      moveX(tempX);
      
    readStates();
    while(stateP == LOW){
      readStates();
    }
  }
  pos[0] = distanceToSteps(camLocation);
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
//  Serial.println("(X) Moving to: " + String(steps) );
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
//  Serial.println("(Y) Moving to: " + String(steps) );
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
    delay(3);                        //delay for DRV8825 to wake back up
  }
}

//Assumed that camera is in top right of the petri dish array (grids[arrSizeX][arrSizeY])
/* snake moves the petri dish through the grid of squares in a snake like fashion (through one row, then move down a column, repeat)
 * Parameters: None
 * Returns void
 */
void snake() {
  readStates();

  //grids[y][x]
  for (int y = 0; y < arrSizeY; y++) {
    //0 is the top of the array
    bool checkRow = false;
    int tempX = 0;
    //If the camera is on the right half of the petri dish
    if (pos[0] > (distanceToSteps(arrSizeX) / 2) ) {
      for (int x = arrSizeX - 1; x >= 0; x--) {
        
        if (!checkRow) {
          int temp = x;
          //Checks the row to make sure that it isn't empty
          while (temp >= 0 && !checkRow) {
            if (grids[y][temp]) {
              checkRow = true;
            }
            temp--;
          }
          if(checkRow)
            x = temp;
        }
        if(grids[y][x]){
          grids[y][x] = 0;
          //Moves to the next available position
          long temp = distanceToSteps(x * stepX);
          moveX(temp);
          pos[0] = temp;
          wait('X');
          readStates();
          while(stateP == LOW){
            readStates();
          }
          tempX = x - 1;
        }
      }
    }else{  //If camera is on left side of the petri dish

      for(int x = 0; x < arrSizeX; x++){
        if(!checkRow){
          int temp = x;
          while(temp < arrSizeX && !checkRow){
            if(grids[y][temp]){
              checkRow = true;
            }
            temp++;
          }
          if(checkRow)
            x = temp;
        }
        if(grids[y][x]){
          grids[y][x] = 0;
          
          long temp = distanceToSteps(x * stepX);
          moveX(temp);
          pos[0] = temp;
          wait('X');          
          readStates();
          while(stateP == LOW){
            readStates();
          }
          tempX = x + 1;
        }
      }
    }
    grids[y][tempX] = 0;
    long temp = distanceToSteps((arrSizeY - y) * stepY);
    digitalWrite(sleepPinY, HIGH);
    delay(3);
    
    moveY(temp);
    pos[1] = temp;
    wait('Y');
    
    readStates();
    while(stateP == LOW){
      readStates();
    }
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
  int pitch = 5;        //pitch of lead screw (5 mm)
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
      Serial.print(String(grids[i][j]) + ", ");
    }
    Serial.println();
  }
}
