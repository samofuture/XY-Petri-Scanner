
/*
  Stepper Motor Control - XY Petri Dish

  TODO: Add Homing Functionality
        Fix edge of circle
        Check circle at smaller grid size
        (Grid Sizes: [2.172, 1.577], [1.86, 1.351])
        Add button to change between the grid sizes

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

void fillWithTrue() {
  bool checkBeg = 0;
  bool checkMid = 0;
  bool checkSta = 0;
  for (int i = 0; i < 34; i++) {
    for (int j = 0; j < 34; j++) {
      if (grids[i][j] && !grids[i][j + 1]) {
        if (checkBeg) {
          checkBeg = 0;
        } else {
          checkBeg = 1;
        }
      } else if (checkBeg && i != 1 && i != 33) {
        grids[i][j] = 1;
      }
    }
    checkBeg = 0;
  }

}

int pause = 1000;


//Inefficiency in how snake handles lower half of petri dish
void snake(int snakeSpeed, int spr, double stepX, double stepY) {
  //TODO Add Homing Feature
  int x = 0;
  int y = 0; //Positive y means that it has gone down on the y axis
  for (int i = 0; i < 34; i++) {
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

      while (x < 34 && grids[i][x]) {
        grids[i][x] = 0;
        delay(pause);
        moveX(stepX, snakeSpeed, spr);
        x++;
      }
      if (x + 1 < 34) {
        if (x + 2 < 34) {
          moveX(stepX, snakeSpeed, spr);
          x++;
        }
        moveX(stepX, snakeSpeed, spr);
        x++;
      }
    }
    moveY(-stepY, snakeSpeed, spr);
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

    snake(moveOnceSpeed, stepsPerRevolution, 1.5, 2);

    displayGrids();

    loopCount++;
  }

}

double revolutionsPerMM = 5; //D5epends on thread size (pitch), how many times the motor must rotate to move one mm

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
//Major Radius, Minor Radius, Center Coordinates
void drawEllipse(int rx, int ry, int xc, int yc) {

  float dx, dy, d1, d2, x, y;
  x = 0;
  y = ry;

  // Initial decision parameter of region 1
  d1 = (ry * ry) - (rx * rx * ry) +
       (0.25 * rx * rx);
  dx = 2 * ry * ry * x;
  dy = 2 * rx * rx * y;

  // For region 1
  while (dx < dy){

    // Print points based on 4-way symmetry
    cout << x + xc << " , " << y + yc << endl;
    cout << -x + xc << " , " << y + yc << endl;
    cout << x + xc << " , " << -y + yc << endl;
    cout << -x + xc << " , " << -y + yc << endl;

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
    cout << x + xc << " , " << y + yc << endl;
    cout << -x + xc << " , " << y + yc << endl;
    cout << x + xc << " , " << -y + yc << endl;
    cout << -x + xc << " , " << -y + yc << endl;

    // Checking and updating parameter
    // value based on algorithm
    if (d2 > 0){
      y--;
      dy = dy - (2 * rx * rx);
      d2 = d2 + (rx * rx) - dy;
    }
    else{
      y--;
      x++;
      dx = dx + (2 * ry * ry);
      dy = dy - (2 * rx * rx);
      d2 = d2 + dx - dy + (rx * rx);
    }
  }
}
