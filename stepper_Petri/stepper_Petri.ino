
/*
  Stepper Motor Control - XY Petri Dish

  Grid Sizes: [2.172, 1.577], [1.86, 1.351]

  Potential Grid Sizes: [2.25, 1.6875], [1.85, 1.3875], [2, 1.5]

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

bool stateX = 0;
bool stateY = 0;

// Switch
#define pausePin 10

bool stateP = 0;

//Size Switch
//On = 500 ms Pause, Off = 1250 ms Pause
#define speedPin 9

double ySize = (2.0);
double xSize = (1.5);

#define stepsPerRevolution 200
#define microSteps 32

#define pitch 5

int Speed = 19200; //Steps per second

#define sleepPinX 12
#define sleepPinY 13

//Position in steps, pos[0] = x, pos[1] = y
long pos[] = {0, 0};

// initialize the stepper library on pins 8 through 11:
//AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
//Motor interface type must be set to 1 when using a driver:
AccelStepper xStepper(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper yStepper(AccelStepper::DRIVER, stepPinY, dirPinY);


#define arrSizeY 25
#define arrSizeX 35

bool grids[arrSizeY][arrSizeX] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, },
  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, },
  { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, },
  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, },
  { 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }
};

int pause = 500;

void setup() {

  Serial.begin(9600);

  //Limit Switch Pin Setup
  pinMode(switchPinX, INPUT);
  pinMode(switchPinY, INPUT);

  //Pause Switch Setup
  pinMode(pausePin, INPUT);

  //Speed Switch Setup
  pinMode(speedPin, INPUT);
  bool velocity = false;
  for (int i = 0; i < 2000; i++) {
    velocity = digitalRead(speedPin);
    if (velocity == LOW) {
      pause = 1250;
      i = 2000;
    }
  }

  //Sleep Pin Setup
  pinMode(sleepPinX, OUTPUT);
  pinMode(sleepPinY, OUTPUT);

  digitalWrite(sleepPinX, HIGH);

  //Set Max Speeds for the steppers (max steps per second)
  xStepper.setMaxSpeed(1000 * microSteps);
  yStepper.setMaxSpeed(1000 * microSteps);

  xStepper.setAcceleration(500 * microSteps);
  yStepper.setAcceleration(500 * microSteps);


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
  pauser();
  digitalWrite(sleepPinX, HIGH);
  delay(5);

  long xSteps = distanceToSteps(xSize);

  xStepper.runToNewPosition(-13 * xSteps);
  
  snake();
  Serial.println(F("After Snake:"));
  displayGrids();
  digitalWrite(sleepPinX, LOW);
  while (true) {}

}

/* readStates updates the positions of the switches
   Parameters: None
   Returns void
*/
void readStates() {
//  stateX = digitalRead(switchPinX);
//  stateY = digitalRead(switchPinY);
  int samples = 10000;
  for (int i = 0; i < samples; i++) {
    stateP = digitalRead(pausePin);
    if (stateP == LOW) {
      i = samples;
    }
  }
//  Serial.print(F("State P: "));
//  Serial.println(String(stateP));
}


//Movement Functions

/* homePetri moves stepper motors to the home position
   Parameters: None
   Returns boolean (true if it is at home)
*/

#define gap 3       //Distance in mm that the thing has to be before it stops pressing limit switch 

bool homePetri() {
  
  uint8_t counter = 0;
  readStates();
  while (stateP == LOW) {
    readStates();
  }
  int xSpeed = Speed * 2;
  int ySpeed = Speed * -2;
  xStepper.setCurrentPosition(0);
  xStepper.setSpeed(Speed * -2);
  yStepper.setCurrentPosition(0);
  yStepper.setSpeed(Speed * 2);
  //  Serial.println(F("Before Checking Switches"));

  digitalWrite(sleepPinY, LOW);

  //Initial Move to limit switch
  //  Serial.println(F("Initial Start"));
  while (stateX == LOW) {
    xStepper.runSpeed();
    stateX = digitalRead(switchPinX);
    delayMicroseconds(50);
  }

  //  Serial.println(F("End Initial"));
  //Move to reset the limit switch
  xStepper.setCurrentPosition(0);
  long tempGap = -distanceToSteps(gap);
  xStepper.runToNewPosition(tempGap);
  stateX = digitalRead(switchPinX);

  //Move Slower to fine tune hitting the limit switch
  xStepper.setSpeed(Speed / 1.5);
  while (stateX == LOW) {
    xStepper.runSpeed();
    stateX = digitalRead(switchPinX);
    delayMicroseconds(50);
  }

  xStepper.setSpeed(Speed);

  digitalWrite(sleepPinX, LOW);

  //Y Homing
  counter = 0;
  digitalWrite(sleepPinY, HIGH);
  delay(5);

  //Initial Move to limit switch
  while (stateY == LOW) {
    yStepper.runSpeed();
    stateY = digitalRead(switchPinY);

  }

  //Move to reset the limit switch
  yStepper.setCurrentPosition(0);
  tempGap = distanceToSteps(gap);
  yStepper.runToNewPosition(tempGap);
  readStates();
  stateY = digitalRead(switchPinY);

  yStepper.setSpeed(-Speed);

  //Move Slower to fine tune hitting the limit switch
  while (stateY == LOW) {
    yStepper.runSpeed();
    stateY = digitalRead(switchPinY);

  }

  yStepper.setSpeed(-Speed * 1.5);

  pos[0] = distanceToSteps(0);
  pos[1] = distanceToSteps(0);
  xStepper.setCurrentPosition(pos[0]);
  yStepper.setCurrentPosition(pos[1]);

  digitalWrite(sleepPinX, HIGH);

  return true;
}

/* wait
   Parameters: char axis (which motor is being paused)
   Returns void
*/
void wait(char axis) {
  if (axis == 'X') {
    digitalWrite(sleepPinX, LOW);
  } else if (axis == 'Y') {
    digitalWrite(sleepPinY, LOW);
  }
  delay(pause);
  if (axis == 'X') {
    digitalWrite(sleepPinX, HIGH);
    delay(5);                        //delay for DRV8825 to wake back up
  }
}

void pauser(){
  readStates();
  while (stateP == LOW) {
    digitalWrite(sleepPinX, LOW);
    readStates();
    if (stateP == HIGH) {
      digitalWrite(sleepPinX, HIGH);
      delay(5);
    }
  }
  
}

void snake(){
  readStates();
  long xSteps = distanceToSteps(xSize);
  long ySteps = distanceToSteps(ySize);

  
  for(int i = 0; i < arrSizeY; i++){
    Serial.print(F("Row: "));
    Serial.println(String(i));
    int tempJ;
    for(int j = 0; j < arrSizeX; j++){
      if(grids[i][j]){
        pos[0] = -j * xSteps;
        xStepper.runToNewPosition(pos[0]);
        wait('X');
        Serial.println(String(j));
        pauser();
        grids[i][j] = 0;
        tempJ = j;
      }
    }
    
    digitalWrite(sleepPinY, HIGH);
    delay(5);
    i++;
    if(i < arrSizeY){
      pos[1] += ySteps;
    }
    yStepper.runToNewPosition(pos[1]);
    wait('Y');
    grids[i][tempJ] = 0;
    Serial.print(F("Row: "));
    Serial.println(String(i));
    pauser();

    for(int j = arrSizeX - 1; j >= 0 && i < arrSizeY; j--){
      if(grids[i][j]){
        pos[0] = -j * xSteps;
        xStepper.runToNewPosition(pos[0]);
        wait('X');
        Serial.println(String(j));
        pauser();
        grids[i][j] = 0;
        tempJ = j;
      }
    }

    digitalWrite(sleepPinY, HIGH);
    delay(5);
    if(i < arrSizeY){
      pos[1] += ySteps;
    }
    yStepper.runToNewPosition(pos[1]);
    wait('Y');
    pauser();
    grids[i][tempJ] = 0;
  }
}

//Assumed that camera is in top right of the petri dish array (grids[arrSizeX][arrSizeY])
/* snake moves the petri dish through the grid of squares in a snake like fashion (through one row, then move down a column, repeat)
   Parameters: None
   Returns void
*/




//Custom Math Functions

/* distanceToSteps calculates the steps to take with a given distance
   Parameters: double distance (the distance in mm)
   Returns long (number of steps to move the distance)
*/
long distanceToSteps(double distance) {
  double temp = distance / pitch;
  temp = temp * stepsPerRevolution * microSteps;
  long ret = round(temp);
  return ret;
}

/* roundUp rounds a double up to the next int
   Parameters: double x (value to be rounded up)
   Returns int (rounded value)
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



/* displayGrids displays the grid
   Parameters: none
   Returns void
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
