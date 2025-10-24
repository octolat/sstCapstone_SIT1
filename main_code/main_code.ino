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
float deg_step(float deg) {
  return (((deg / 360) * 200) * 8);
}

// Convert steps to degree (bender only cause its micro steps)
float step_deg(float step) {
  return ((step * 360) / 200) / 8;
}


//Test to find center of pin
void setPin(float angle) {
  myservo.write(SERVO_DOWN);
  bend_stepper.moveTo(deg_step(angle));  // Move forward again
  bend_stepper.runToPosition();
  bend_stepper.setCurrentPosition(0);
}

// PID to fix overbend and underbend of wire
void fixBend(float bend_angle) {
  //EVERYTHING IS IN STEP NOT DEG
  float kp = 1.8;
  float kd = 0.3;
  int offset = deg_step(5);  //how much to move away to clear the wire !todo measure this shit
  int additonal_offset = deg_step(10);
  int threshold = deg_step(1);
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
    Serial.println(step_deg(underbend));
    Serial.print("error: ");
    Serial.println(step_deg(error));
    Serial.print("movement: ");
    Serial.println(step_deg(move_amt));

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
  feed_stepper.setMaxSpeed(2000);     // Max speed of stepper
  feed_stepper.setAcceleration(700);  //Set acceleration for stepper
  feed_stepper.setSpeed(1000);        //Set the constant speed that the stepper will move

  //// Rotating Stepper Motor Settings
  rotate_stepper.setMaxSpeed(2000);     // Max speed of stepper
  rotate_stepper.setAcceleration(700);  //Set acceleration for stepper
  rotate_stepper.setSpeed(1200);        //Set the constant speed that the stepper will move

  //// Bending Stepper Motor Settings
  bend_stepper.setMaxSpeed(10000);     // Max speed of stepper
  bend_stepper.setAcceleration(1000);  //Set acceleration for stepper
  bend_stepper.setSpeed(deg_step(45));         //Set the constant speed that the stepper will move


  //make the all the motors zeroed
  rotate_stepper.setCurrentPosition(0);
  feed_stepper.setCurrentPosition(0);

  // Make pin center
  setPin(120);


  // Reader Initialization
  SD.begin(chipselect);
  Serial.println("Initializing SD Card...");
  if (!SD.begin(chipselect)) {
    while (true) {
      Serial.println("Initialization failed");
      delay(100);
    }
  } else {
    Serial.println("Initialization done");
  }

  // Reading SD Card
  File file = SD.open("square.txt");
  if (!file) {
    Serial.println("Error opening file!");   // File does not exist
  }
  if (file.size() == 0) {
    Serial.println("File exists but is empty!");
  } else {
    while (file.available()) {
      String line = file.readStringUntil('\n');
      Serial.println(line);  
      funct = line[0];
      dir = line[1];
      Serial.println(funct);
      if (funct == "F") {
        //Feeding Code
        centi_amt = line.substring(1).toFloat();
        Serial.println((centi_amt / (33.33*pi)) * 200);
        feed_stepper.move((-((centi_amt / (33.33*pi)) * 200)));
        feed_stepper.runToPosition();
        feed_stepper.setCurrentPosition(0);
        delay(2000);
      } else if (funct == "R") {
        //Rotating Code
        degree_amt = line.substring(2).toFloat();
        Serial.println((((degree_amt / 360) * 200) / 24) * 60);
        if (dir == "-") {
          rotate_stepper.move((((degree_amt / 360) * 200) / 24) * 60);
          rotate_stepper.runToPosition();
        } else {
          rotate_stepper.move((-(((degree_amt / 360) * 200) / 24) * 60));
          rotate_stepper.runToPosition();
        }
        // rotate_stepper.runToPosition();
        delay(2000);
      } else if (funct == "B") {
        //Bending Code
        degree_amt = line.substring(2).toFloat();
        if (dir == "-") {
          // Offset pin 
          bend_stepper.move(-1 * deg_step(15.0));
          bend_stepper.runToPosition();
          delay(1000);

          // Bending
          if (bend_stepper.currentPosition() != 0) {
            myservo.write(SERVO_UP);
            delay(1000);
            float bend_angle = deg_step(degree_amt);
            bend_stepper.move(bend_angle);  // Move forward again
            bend_stepper.runToPosition();
            bend_stepper.moveTo(0); //turn back
            bend_stepper.runToPosition();
            delay(500);
            myservo.write(SERVO_DOWN);
            }

        } else {
          // Offset pin 
          bend_stepper.move(deg_step(10.0));
          bend_stepper.runToPosition();
          delay(1000);

          // Bending
          if (bend_stepper.currentPosition() != 0) {
            myservo.write(SERVO_UP);
            delay(1000);
            float bend_angle = deg_step(degree_amt);
            bend_stepper.move(-1 * bend_angle);  // Move forward again
            bend_stepper.runToPosition();
            bend_stepper.moveTo(0); //turn back
            bend_stepper.runToPosition();
            delay(500);
            myservo.write(SERVO_DOWN);
          }
        }
        rotate_stepper.moveTo(0);
        rotate_stepper.runToPosition();
        Serial.println(bend_stepper.currentPosition());
        delay(2000);
      }
    }
  }
}

void loop() {
}