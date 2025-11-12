// Libraries

#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdio.h>
#include <math.h>

//Global Variables

//// Servo
Servo myservo;

//preset numbers
#define SERVO_UP 180
#define SERVO_MEASURE 60
#define SERVO_DOWN 0

#define BENDER 0
#define FEEDER 1
#define ROTATER 2

#define MEASURE_ACC 0

//pins
const int TOUCH_PIN = 47; 
const int SERVO_PIN = 48;
const int chipselect = 53;  // CS pin for SD Card Reader Module
// MOSI pin 51
// MISO pin 50
// SCK pin 52
// CS pin 53

String funct = "";
String dir = "";
double centi_amt;
double degree_amt;
double pi = 2 * acos(0.0);

String data[100];

int count = 0;

//// Stepper motors
#define enable_pin 8                                      // define the enable pin to make the drive high and low (All stepper motor share this pin)
AccelStepper feed_stepper(AccelStepper::DRIVER, 2, 5);    // X Driver ((Don't Change), stepPin, dirPin)
AccelStepper rotate_stepper(AccelStepper::DRIVER, 3, 6);  // Y Driver ((Don't Change), stepPin, dirPin)
AccelStepper bend_stepper(AccelStepper::DRIVER, 4, 7);    // Z Driver ((Don't Change), stepPin, dirPin)


// Convert degree into steps (bender only cause its micro steps)
double unit_to_step(int motor, double amt) {
  if (motor == BENDER) return ((amt / 360.0) * 200.0 * 8.0) / 17.0 * 31.0;
  if (motor == ROTATER) return (((amt / 360.0) * 200.0) / 20.0) * 65.0;
  if (motor == FEEDER) return (amt / (33.33 * pi)) * 200.0;
}

// Convert steps to degree (bender only cause its micro steps)
double step_to_unit(int motor, double amt) {
  if (motor == BENDER) return ((amt / 200.0 / 8.0) * 360.0) * 17.0 / 31.0;
  if (motor == ROTATER) return (((amt / 200.0) * 360.0) * 20.0) / 65.0;
  if (motor == FEEDER) return (amt / 200.0 * (33.33 * pi)); // may be wrong!!
}

// Functiion to initialise the sd card reader module (this should be placed at the start of set up)
void initialise() {
  SD.begin(chipselect);
  Serial.println("Initializing SD Card...");
  if (!SD.begin(chipselect)) {
    while (true) {
      Serial.println("Initialization failed"); // SD Card reader is not connected properly or sd card is not in the reader
      delay(100);
    }
  } else {
    Serial.println("Initialization done"); // Arduino can read the SD Card
  }
}

//Test to find center of pin
void setPin(float angle) {
  myservo.write(SERVO_DOWN); // Bring servo down
  bend_stepper.moveTo(unit_to_step(BENDER, angle));  // Set what degree for the motor to move to 
  bend_stepper.runToPosition();
  bend_stepper.setCurrentPosition(0); // Set current position as 0 (Default)
}

// PID to fix overbend and underbend of wire
long fixBend(float bend_angle, int max_tries = -1) {
  //EVERYTHING IS IN STEP NOT DEG
  int end_limit = unit_to_step(BENDER, 120);
  float kp = 0.2;
  float minimum_move = 3;
  int threshold = unit_to_step(BENDER, 1.5);

  int touch, dir;
  double underbend;
  long error = threshold + 1;  // plus 1 so that the while loop will run at least once
  int tries = 0;
  long correction = 0;


  dir = bend_angle / abs(bend_angle);  //the sign of bend angle

  while (true) {
    if (max_tries > 0 && max_tries <= tries) break;  // limit the number of attempts
    tries++;
    touch = 1;
    underbend = 0;

    //find the wire
    
    bend_stepper.setSpeed(unit_to_step(BENDER,200));
    bend_stepper.runToNewPosition(0);
    bend_stepper.runToNewPosition(unit_to_step(BENDER, bend_angle/2));
    myservo.write(SERVO_MEASURE);
    while (touch != 0) {
      touch = digitalRead(TOUCH_PIN);
      bend_stepper.move(1 * dir);
      bend_stepper.runSpeed();
      //if (bend_stepper.runSpeed()) underbend++;
      //Serial.println(step_to_unit(BENDER, underbend));
      //delay(1);
    }
    underbend = abs(bend_stepper.currentPosition());
    error = abs(unit_to_step(BENDER, bend_angle)) - underbend;  // minus offset to make the angle centered to the center of the pin
    int error_dir = error / abs(error);
    float move_amt = max(abs(error) * kp, unit_to_step(BENDER, minimum_move)) * dir * error_dir; //must move at least minimum move eg. 3 deg
    correction += error * kp;

    Serial.println("found the wire");
    Serial.print("angle measured: ");
    Serial.println(step_to_unit(BENDER, underbend));
    Serial.print("error: ");
    Serial.println(step_to_unit(BENDER, error));
    Serial.print("movement: ");
    Serial.println(step_to_unit(BENDER, move_amt));

    delay(1500);

    //quit if the error is small already
    if (abs(error) < threshold) break;

    if (error_dir == 1) { //underbend
      bend_stepper.runToNewPosition(0);
      myservo.write(SERVO_UP);
      delay(1000);
      bend_stepper.runToNewPosition(unit_to_step(BENDER,bend_angle) + move_amt);
    } else {
      bend_stepper.runToNewPosition(0);
      myservo.write(SERVO_DOWN);
      delay(1000);
      bend_stepper.runToNewPosition(end_limit*dir);
      myservo.write(SERVO_UP);
      delay(1000);
      bend_stepper.runToNewPosition(unit_to_step(BENDER,bend_angle) + move_amt);
      bend_stepper.runToNewPosition(end_limit*dir);
      myservo.write(SERVO_DOWN);
      delay(1000);
    }
  }
  return correction;

}

void test(int type){
  if (type == MEASURE_ACC){
    int touch = 1;
    for (int i = 0; i < 10; i++) {
      bend_stepper.runToNewPosition(200);
      myservo.write(SERVO_MEASURE);
      while (touch != 0) {
        touch = digitalRead(TOUCH_PIN);
        bend_stepper.move(-1);
        bend_stepper.runSpeed();
      }
      touch = 1;
      Serial.println(bend_stepper.currentPosition());
    }
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(TOUCH_PIN, INPUT_PULLUP);

  myservo.attach(SERVO_PIN);  // Attaches servo to pin
  myservo.write(0);           // Maker servo run to zero

  //Stepper Settings
  pinMode(enable_pin, OUTPUT);    // Set up enable pin
  digitalWrite(enable_pin, LOW);  // Enable Drivers


  //// Feeding Stepper Motor Settings
  feed_stepper.setMaxSpeed(unit_to_step(FEEDER, 20));   // Max speed of stepper
  feed_stepper.setAcceleration(10000);                  //Set acceleration for stepper
  feed_stepper.setSpeed(unit_to_step(FEEDER, 20));      //Set the constant speed that the stepper will move

  //// Rotating Stepper Motor Settings
  rotate_stepper.setMaxSpeed(300);     // Max speed of stepper
  rotate_stepper.setAcceleration(50);  //Set acceleration for stepper
  rotate_stepper.setSpeed(300);        //Set the constant speed that the stepper will move

  //// Bending Stepper Motor Settings
  bend_stepper.setMaxSpeed(unit_to_step(BENDER, 1000));     // Max speed of stepper
  bend_stepper.setAcceleration(1000);  //Set acceleration for stepper
  bend_stepper.setSpeed(unit_to_step(BENDER, 45));    //Set the constant speed that the stepper will move


  //make the all the motors zeroed
  rotate_stepper.setCurrentPosition(0);
  feed_stepper.setCurrentPosition(0);

  // Reader Initialization
  initialise();

  

  // Function to make pin center
  setPin(114);

  // Reading SD Card
  File file = SD.open("cube.txt"); // Open file inside open() 

  //checking file for errors
  if (!file) { // Checks if file inputted above exists
    while (true) {
      Serial.println("Error opening file!");   // File does not exist 
      delay(500);
    }
  } else if (file.size() == 0) {
    while (true) {
      Serial.println("File exists but is empty!");   // File is empty
      delay(500);
    }
  }
  while (file.available()) {
    String line = file.readStringUntil('\n'); // Read the instructions line by line
    Serial.println(line); // Print each line
    data[count] = line;
    count += 1;
    delay(20);
  }

  // Close the file
  file.close();
  Serial.println("File Closed");

  long springback_offset = 0;
  bool first = true;

  //main code
  for (int i = 0; i < count; i++) {
    String line = data[i];
    Serial.println(line);
    funct = line[0]; // Check the first character of the line for F, B, R (To decide which function to do; Feed, Rotate, Bend)
    dir = line[1]; // Check second character of line for + or - (To decide which direction the motor will turn, + is anti-clockwise, - is clockwise)

    if (funct == "F") {
      // Feeding Code

      //// Get Amount to feed
      float centi_amt = line.substring(1).toFloat(); // Convert second character onwards till the last character (How much to feed in milimetres) onwards from a string to a float 
      float feed_amt = unit_to_step(FEEDER, centi_amt); // Calculate the amount of steps to feed (Formula is based on the gear size used)

      //// Run the Motors
      feed_stepper.move(-1 * feed_amt);
      feed_stepper.runToPosition();

      feed_stepper.setCurrentPosition(0); // Set the current position where the feeding stops at 0 (starting posiiton)
      delay(2000);

    } else if (funct == "R") {
      // Rotating Code

      //// Getting amount to rotate
      degree_amt = line.substring(2).toFloat(); // Convert third character onwards till the last character (How much to rotate in degree) onwards from a string to a float 
      float rotate_amt = unit_to_step(ROTATER, degree_amt); // Calculate amount of steps to rotate (Formula is based on the gear ratio used)

      int dir_sign = 1;
      if (dir == "-") dir_sign = -1; // find the direction of rotating

      rotate_stepper.move(rotate_amt * dir_sign * -1); //rotate
      rotate_stepper.runToPosition();

      delay(2000);

    } else if (funct == "B") {
      // Bending Code

      //// Get amount to bend 
      degree_amt = line.substring(2).toFloat(); // Convert third character onwards to till the last character (How much to bend in degree) onwards from a string to a float 
      float bend_amt = unit_to_step(BENDER, degree_amt) + abs(springback_offset);
      Serial.println(step_to_unit(BENDER, bend_amt));

    
      int dir_sign = 1;
      if (dir == "-") dir_sign = -1; // find the direction of turning

      bend_stepper.move(-1 * dir_sign * unit_to_step(BENDER, 15.0)); // Offset pin by 15 degrees
      bend_stepper.runToPosition();
      delay(1000);

      myservo.write(SERVO_UP); // Bring pin up 
      delay(1000);

      bend_stepper.move(1 * dir_sign * bend_amt);  // Move pin by bend_amt
      bend_stepper.runToPosition();
      delay(1500);

      //fixBend(degree_amt*dir_sign, 3);

     
      if (first) {
        springback_offset += fixBend(degree_amt*dir_sign, 3);
        first = false;
      }
      else { 
        springback_offset += fixBend(degree_amt*dir_sign, 3);
      }
   
      bend_stepper.moveTo(0); // Move back to the center
      bend_stepper.runToPosition();
      delay(500);

      myservo.write(SERVO_DOWN); // Bring pin down
        
      delay(2000);
    }    
    
  }
  
}

void loop() {
}