/*********************************************************************
 * M.I.A ROBOTICS TEAM
 * Robot Arm Test
 *********************************************************************/

#include<Servo.h>         // include servo library to control servo motors
Servo base, elbow, wrist;      // create servo objects for motors

int base_degree, elbow_degree, wrist_degree;     // create variables to store servo degree

// define degree limits
#define MAX_DEGREE 180
#define MIN_DEGREE 0

/***************** INITIAL SETUP ******************/
void setup() {
  // attach servos to pwm pins
  base.attach(9);
  elbow.attach(10);
  wrist.attach(11);
  
  initial();        // initialize servos positions
}

// Function to read servo positions
void read_degree() {
  base_degree = base.read();
  elbow_degree = elbow.read();
  wrist_degree = wrist.read();
}

// Function to initialize servo positions
void initial()
 {
  read_degree();

  // Base servo will be set to 90deg
  if(base_degree > 90) {
    for(base_degree; base_degree > 90; base_degree--)
    {
      base.write(base_degree);
      delay(15);
    }
  }
  else if(base_degree < 90) {
    for(base_degree; base_degree < 90; base_degree++)
    {
      base.write(base_degree);
      delay(15);
    }
  }

  // Elbow & Wrist servo will be set to 0deg
  for(elbow_degree; elbow_degree > MIN_DEGREE; elbow_degree--)
  {
    elbow.write(elbow_degree);
    delay(15);
  }
  
  for(wrist_degree; wrist_degree > MIN_DEGREE; wrist_degree--)
  {
    wrist.write(wrist_degree);  
    delay(15);
  }
 }

// Function to test the arm
void test() {
  read_degree();

  // rotate wrist 180deg CW
  for(wrist_degree; wrist_degree < MAX_DEGREE; wrist_degree++)
  {
    wrist.write(wrist_degree);  
    delay(15);
  }
  
  // rotate elbow 180deg CW
  for(elbow_degree; elbow_degree < MAX_DEGREE; elbow_degree++)
  {
    elbow.write(elbow_degree);
    delay(15);
  }

  //rotate base 90deg CWW
  for(base_degree; base_degree > MIN_DEGREE; base_degree--)
  {
    base.write(base_degree);
    delay(15);
  }
}

/***************** MAIN LOOP ******************/ 
void loop() {  
  test();       // test the arm
  initial();    // set it back to initial values
  delay(500);
}
