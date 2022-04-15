
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
#define switchPinY 7
#define switchPinX 8

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
#define microSteps 32

#define pitch 5

#define SPEED 12800 //Steps per second

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

#define pause 500

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

  //Set Max Speeds for the steppers (max steps per second)
  xStepper.setMaxSpeed(500 * microSteps);
  yStepper.setMaxSpeed(500 * microSteps);

  xStepper.setAcceleration(500 * microSteps);
  yStepper.setAcceleration(500 * microSteps);

  camLocation = distanceToSteps(75);
  
  //Home the petri dish to 0, 0
  bool isHome = false;
  while (!isHome) {
    isHome = homePetri();
//    Serial.println(F("Not Home"));
  }
//  Serial.println(F("Is Home"));

  digitalWrite(sleepPinX, LOW);
  digitalWrite(sleepPinY, LOW);

}

void loop() {
  readStates();
//  delay(500);
//  const double gridSizeY = 1.86; //2.172;           //size of the length (X) of a grid square in mm
//  const double gridSizeX = 1.351; //1.577;           //size of the length (Y) of a grid square in mm
  const int sizeOfPetri = 50;               //diameter of petri dish in mm

  int centerX = (arrSizeY / 2);             //Center of grids
  int centerY = (arrSizeX / 2);

  bool stateS = false;

  //0,0 cant be center of circle
  if (loopCount == 0) {
    //Debug switches reading between 0 and 1 when the switch is off
    for(int i = 0; i < 2000; i++){
      stateS = digitalRead(sizePin);
      if(stateS == LOW){
        i = 2000;
        xSize = 1.577;
        ySize = 2.172;
      }
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
  for(int i = 0; i < 2000; i++){
    stateP = digitalRead(pausePin);
    if(stateP == LOW){
      i = 2000;
    }
  }
}


//Movement Functions

/* homePetri moves stepper motors to the home position
 * Parameters: None
 * Returns boolean (true if it is at home)
 */

#define gap 3       //Distance in mm that the thing has to be before it stops pressing limit switch 
 
bool homePetri() {
  uint8_t counter = 0;
  readStates();
  while(stateP == LOW){
    readStates();
  }
  xStepper.setCurrentPosition(0);
  xStepper.setSpeed(SPEED * 2);
  yStepper.setCurrentPosition(0);
  yStepper.setSpeed(-SPEED * 2);
//  Serial.println(F("Before Checking Switches"));

  digitalWrite(sleepPinY, LOW);

  //Initial Move to limit switch
//  Serial.println(F("Initial Start"));
  while (stateX == LOW) {
    xStepper.runSpeed();
    while(stateP == LOW){
      readStates();
    }
//    Serial.println(F("Hitting X Switch"));
    if(counter > 150){
      readStates();
      counter = 0;
    }
    counter++;
  }
  counter = 0;
  
//  Serial.println(F("End Initial"));
  //Move to reset the limit switch
  xStepper.setCurrentPosition(0);
  long tempGap = -distanceToSteps(gap);
  xStepper.runToNewPosition(tempGap);
  readStates();
  
  //Move Slower to fine tune hitting the limit switch
  xStepper.setSpeed(SPEED / 1.5);
  while (stateX == LOW) {
    xStepper.runSpeed();
    while(stateP == LOW){
      readStates();
    }
//    Serial.println(F("Hitting X Switch"));
    if(counter > 200){
      readStates();
      counter = 0;
    }
    counter++;
  }
  
  xStepper.setSpeed(SPEED);

  digitalWrite(sleepPinX, LOW);
  
  //Y Homing
  counter = 0;
  digitalWrite(sleepPinY, HIGH);
  delay(5);
  
  //Initial Move to limit switch
  while (stateY == LOW) {
    yStepper.runSpeed();
    while(stateP == LOW){
      readStates();
    }
//    Serial.println(F("Hitting Y Switch"));
    if(counter > 200){
      readStates();
      counter = 0;
    }
    counter++;
  }
  counter = 0;
  
  //Move to reset the limit switch
  yStepper.setCurrentPosition(0);
  tempGap = distanceToSteps(gap);
  yStepper.runToNewPosition(tempGap);
  readStates();

  yStepper.setSpeed(-SPEED);
  
  //Move Slower to fine tune hitting the limit switch
  while (stateY == LOW) {
    yStepper.runSpeed();
    while(stateP == LOW){
      readStates();
    }
//    Serial.println(F("Hitting Y Switch"));
    if(counter > 200){
      readStates();
      counter = 0;
    }
    counter++;
  }

  yStepper.setSpeed(-SPEED * 1.5);

  pos[0] = distanceToSteps(0);
  pos[1] = distanceToSteps(0);
  xStepper.setCurrentPosition(pos[0]);
  yStepper.setCurrentPosition(pos[1]);
  
  digitalWrite(sleepPinX, HIGH);
  
  return true;
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
void snake(){
  readStates();
  long xSteps = distanceToSteps(xSize);
  long ySteps = distanceToSteps(ySize);
  uint8_t mod = 0;
  if(xSize == 2.172){
    mod = 7;
  }
  bool check = false;
  for(int i = 1; i < arrSizeY; i++){
    Serial.print(F("Row: "));
    Serial.println(String(i));
    digitalWrite(sleepPinX, HIGH);
    delay(5);
    for(int j = 1 + mod; j < (arrSizeX - mod); j++){
      while(stateP == LOW){
        digitalWrite(sleepPinX, LOW);
        readStates();
        if(stateP == HIGH){
          digitalWrite(sleepPinX, HIGH);
          delay(5);
        }
      }
      if(grids[i][j]){
        pos[0] = -j * xSteps;
        Serial.print(F("J: "));
        Serial.println(String(j));
        xStepper.runToNewPosition(pos[0]);
        wait('X');
        grids[i][j] = false;
        check = true;

      }else{
        if(check){
          j = arrSizeX;
          check = false;
        }
      }
      
      readStates();
    }
    
    xStepper.runToNewPosition(pos[0]);

    pos[1] += ySteps;
    digitalWrite(sleepPinY, HIGH);
    yStepper.runToNewPosition(pos[1]);
    i++;
    wait('Y');

    for(int j = arrSizeX - 1 - mod; j >= mod + 1; j--){
      readStates();
      while(stateP == LOW){
        digitalWrite(sleepPinX, LOW);
        readStates();
        if(stateP == HIGH){
          digitalWrite(sleepPinX, HIGH);
          delay(5);
        }
      }
      
      if(grids[i][j]){
        pos[0] = -j * xSteps;
        Serial.print(F("J: "));
        Serial.println(String(j));
        xStepper.runToNewPosition(pos[0]);
        wait('X');
        grids[i][j] = false;
        check = true;
      }else{
        if(check){
          j = arrSizeX;
          check = false;
        }
      }

    }
    pos[1] += ySteps;
    digitalWrite(sleepPinY, HIGH);
    yStepper.runToNewPosition(pos[1]);
    wait('Y');
    
    readStates();
    while(stateP == LOW){
      digitalWrite(sleepPinX, LOW);
      readStates();
      if(stateP == HIGH){
        digitalWrite(sleepPinX, HIGH);
        delay(5);
      }
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
  double temp = distance / pitch;
  temp = temp * stepsPerRevolution * microSteps;
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
