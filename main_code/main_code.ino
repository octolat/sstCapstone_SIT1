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

//pins
const int TOUCH_PIN = 47; 
const int SERVO_PIN = 48;
const int chipselect = 53;  // CS pin for SD Card Reader Module

String funct = "";
String dir = "";
double centi_amt;
double degree_amt;
double pi = 2 * acos(0.0);


//// Stepper motors
#define enable_pin 8                                      // define the enable pin to make the drive high and low (All stepper motor share this pin)
AccelStepper feed_stepper(AccelStepper::DRIVER, 2, 5);    // X Driver ((Don't Change), stepPin, dirPin)
AccelStepper rotate_stepper(AccelStepper::DRIVER, 3, 6);  // Y Driver ((Don't Change), stepPin, dirPin)
AccelStepper bend_stepper(AccelStepper::DRIVER, 4, 7);    // Z Driver ((Don't Change), stepPin, dirPin)

// Convert degree into steps (bender only cause its micro steps)
float unit_to_step(int motor, float amt) {
  if (motor == BENDER) return ((amt / 360) * 200 * 8) / 17 * 31;
  if (motor == ROTATER) return (((amt / 360) * 200) / 20) * 65;
  if (motor == FEEDER) return (amt / (33.33 * pi)) * 200.0;
}

// Convert steps to degree (bender only cause its micro steps)
float step_to_unit(int motor, float amt) {
  if (motor == BENDER) return ((amt / 200 / 8) * 360) / 17 * 31;
  if (motor == ROTATER) return (((amt / 200) * 360) / 24) * 60;
  if (motor == FEEDER) return (amt / 200.0 * (33.33 * pi));
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
void fixBend(float bend_angle) {
  //EVERYTHING IS IN STEP NOT DEG
  float kp = 1.8;
  float kd = 0.3;
  int offset = unit_to_step(BENDER, 5);  //how much to move away to clear the wire !todo measure this shit
  int additonal_offset = unit_to_step(BENDER, 10);
  int threshold = unit_to_step(BENDER, 1);
  int max_tries = -1;

  int touch, dir, underbend;
  long error = threshold + 1;  // plus 1 so that the while loop will run at least once
  int tries = 0;


  dir = bend_angle / abs(bend_angle);  //the sign of bend angle

  while (true) {
    if (max_tries > 0 && max_tries <= tries) break;  // limit the number of attempts
    tries++;
    touch = 0;
    underbend = 0;

    //find the wire
    myservo.write(SERVO_MEASURE);
    bend_stepper.setSpeed(dir * 250);
    while (touch != 1) {
      touch = digitalRead(TOUCH_PIN);
      bend_stepper.run();
      underbend += 1;
      delay(10);
    }
    error = abs(bend_angle) - underbend - offset;  // minus offset to make the angle centered to the center of the pin
    float error_dir = error / abs(error);
    float move_amt = ((error * kp) + (offset * 2 * error_dir)) * dir;  //offset *2 so that minimally there is still like 5 deg of bending going on, like a minimum bend type shi

    Serial.println("hit");
    Serial.print("angle measured: ");
    Serial.println(step_to_unit(BENDER, underbend));
    Serial.print("error: ");
    Serial.println(step_to_unit(BENDER, error));
    Serial.print("movement: ");
    Serial.println(step_to_unit(BENDER, move_amt));

    //quit if the error is small already
    if (abs(error) < threshold) break;


    //back off so that we can retract without catching the wire
    Serial.println("backing off");
    bend_stepper.setSpeed(100);
    bend_stepper.move(-1 * (offset)*dir);
    bend_stepper.runToPosition();

    // if its overbent, we need to go to the other side of the wire to reverse bend it
    if (error_dir == -1) {
      Serial.println("Overbent, moving to the other side");
      myservo.write(SERVO_DOWN);
      delay(1000);
      bend_stepper.move((offset + additonal_offset) * dir * 2);
      bend_stepper.runToPosition();
      delay(1000);
    }
    myservo.write(SERVO_UP);
    delay(1000);

    //move to the edge of the wire
    bend_stepper.move(additonal_offset * error_dir * dir);
    bend_stepper.runToPosition();
    Serial.println("At the edge, ready to correct");
    delay(1000);

    //Do the actual correction bend
    bend_stepper.move(move_amt);
    bend_stepper.runToPosition();
    Serial.println("bent");
    delay(1000);

    // move away before retracting the bend pin
    bend_stepper.move((offset + additonal_offset) * dir * error_dir);
    bend_stepper.runToPosition();
    delay(1000);

    myservo.write(SERVO_DOWN);
    delay(1000);  // mandatory

    // go home
    bend_stepper.moveTo(0);
    bend_stepper.runToPosition();
    delay(1000);
  }
}

void setup() {
  Serial.begin(9600);

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
  bend_stepper.setMaxSpeed(unit_to_step(BENDER, 45));     // Max speed of stepper
  bend_stepper.setAcceleration(1000);  //Set acceleration for stepper
  bend_stepper.setSpeed(unit_to_step(BENDER, 45));    //Set the constant speed that the stepper will move


  //make the all the motors zeroed
  rotate_stepper.setCurrentPosition(0);
  feed_stepper.setCurrentPosition(0);

  // Reader Initialization
  initialise();

  // Function to make pin center
  setPin(118);

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

  //main code
  while (file.available()) {
    String line = file.readStringUntil('\n'); // Read the instructions line by line
    Serial.println(line); // Print each line

    funct = line[0]; // Check the first character of the line for F, B, R (To decide which function to do; Feed, Rotate, Bend)
    Serial.println(funct);
    dir = line[1]; // Check second character of line for + or - (To decide which direction the motor will turn, + is anti-clockwise, - is clockwise)

    if (funct == "F") {
      // Feeding Code

      //// Get Amount to feed
      float centi_amt = line.substring(1).toFloat(); // Convert second character onwards till the last character (How much to feed in milimetres) onwards from a string to a float 
      float feed_amt = unit_to_step(FEEDER, centi_amt); // Calculate the amount of steps to feed (Formula is based on the gear size used)
      Serial.println(feed_amt); 

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
      Serial.println(rotate_amt);

      int dir_sign = 1;
      if (dir == "-") dir_sign = -1; // find the direction of rotating

      rotate_stepper.move(rotate_amt * dir_sign); //rotate
      rotate_stepper.runToPosition();

      delay(2000);

    } else if (funct == "B") {
      // Bending Code

      //// Get amount to bend 
      degree_amt = line.substring(2).toFloat(); // Convert third character onwards to till the last character (How much to bend in degree) onwards from a string to a float 
      float bend_amt(unit_to_step(BENDER, degree_amt));

      int dir_sign = 1;
      if (dir == "-") int dir_sign = -1; // find the direction of turning

      bend_stepper.move(-1 * dir_sign * unit_to_step(BENDER, 15.0)); // Offset pin by 15 degrees
      bend_stepper.runToPosition();
      delay(1000);

      myservo.write(SERVO_UP); // Bring pin up 
      delay(1000);

      bend_stepper.move(1 * dir_sign * bend_amt);  // Move pin by bend_amt
      bend_stepper.runToPosition();
      delay(500);
      
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