// Libraries

#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdio.h>
#include <math.h> 
using namespace std;

#define SERVO_UP 150
#define SERVO_MEASURE 60
#define SERVO_DOWN 0

//Global Variables
const int chipselect = 53;
String funct = "";
String dir = "";
double centi_amt;
double degree_amt;
double pi = 2 * acos(0.0); 
Servo myservo;


//Define Stepper motors 
#define enable_pin 8 // define the enable pin to make the drive high and low (All stepper motor share this pin)
AccelStepper feed_stepper(AccelStepper::DRIVER, 2, 5); // X Driver ((Don't Change), stepPin, dirPin)
AccelStepper rotate_stepper(AccelStepper::DRIVER, 3, 6); // Y Driver ((Don't Change), stepPin, dirPin)
AccelStepper bend_stepper(AccelStepper::DRIVER, 4, 7); // Z Driver ((Don't Change), stepPin, dirPin)


void fixBend();
void setPin (float angle);
void run (AccelStepper motor);

void setup() {
  Serial.begin(9600);

  myservo.attach(48); // Attaches servo to pin

  myservo.write(0); // Maker servo run to zero

  //Stepper Settings
  pinMode(enable_pin, OUTPUT); // Set up enable pin
  digitalWrite(enable_pin, LOW); // Enable Drivers


  //// Feeding Stepper Motor Settings
  feed_stepper.setMaxSpeed(2000); // Max speed of stepper
  feed_stepper.setAcceleration(700); //Set acceleration for stepper
  feed_stepper.setSpeed(1000); //Set the constant speed that the stepper will move

  //// Rotating Stepper Motor Settings
  rotate_stepper.setMaxSpeed(2000); // Max speed of stepper
  rotate_stepper.setAcceleration(700); //Set acceleration for stepper
  rotate_stepper.setSpeed(1200); //Set the constant speed that the stepper will move

  //// Bending Stepper Motor Settings
  bend_stepper.setMaxSpeed(10000); // Max speed of stepper
  bend_stepper.setAcceleration(7000); //Set acceleration for stepper
  bend_stepper.setSpeed(8000); //Set the constant speed that the stepper will move

  


  // Make pin center 
  setPin(125);
  Serial.println("in the middled");
  fixBend();
  /* 
  
  // SD Card Reader
  //// Reader Initialization
  SD.begin(chipselect);
  Serial.println("Initializing SD Card...");
  if (!SD.begin(chipselect)) {
    Serial.println("Initialization failed");
  } else {
    Serial.println("Initialization done");
  }
  //// Reading SD Card
  File file = SD.open("square.txt");
  if (!file) {
    Serial.println("Error opening file!");   // File does not exist
  } else {
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
          run(feed_stepper);
          feed_stepper.setCurrentPosition(0);
          delay(2000);
        } else if (funct == "R") {
          //Rotating Code
          degree_amt = line.substring(2).toFloat();
          Serial.println((((degree_amt / 360) * 200) / 24) * 60);
          if (dir == "-") {
            rotate_stepper.move((((degree_amt / 360) * 200) / 24) * 60);
            run(rotate_stepper);
          } else {
            rotate_stepper.move((-(((degree_amt / 360) * 200) / 24) * 60));
            run(rotate_stepper);
          }
          // rotate_stepper.runToPosition();
          delay(2000);
        } else if (funct == "B") {
          //Bending Code
          degree_amt = line.substring(2).toFloat();
          if (dir == "-") {
            // Offset pin 
            bend_stepper.move(-1 * deg_step(9.0));
            bend_stepper.runToPosition();
            delay(1000);

            // Bending
            if (bend_stepper.currentPosition() != 0) {
              myservo.write(120);
              delay(500);
              float bend_angle = deg_step(degree_amt);
              bend_stepper.moveTo(bend_angle);  // Move forward again
              run(bend_stepper);
              myservo.write(0); //servo down
              delay(500);
              bend_stepper.moveTo(0); //turn back
              run(bend_stepper);

              }
            }
          } else {
            // Offset pin 
            bend_stepper.move(deg_step(9.0));
            bend_stepper.runToPosition();
            delay(1000);

            // Bending
            if (bend_stepper.currentPosition() != 0) {
              myservo.write(120);
              delay(500);
              bend_stepper.moveTo(-1 * deg_step(degree_amt));  // Move forward again
              run(bend_stepper);
              myservo.write(0);
              delay(500);
              bend_stepper.moveTo(0);
              run(bend_stepper);
            }
          }
          delay(2000);
        }
      }
    }
  }
  */
}

void loop () {

}

float deg_step (float deg) {
  return (((((deg / 360) * 200) / 17 ) * 31) * 8);
  
}
float step_deg (float step) {
  return ((((step*360) / 200) *17) /31) / 8;
}


void run (AccelStepper motor) {
  while (motor.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
    motor.runSpeedToPosition();
  }
}

//Test to find center of pin
void setPin (float angle) {
  myservo.write(SERVO_UP);
  bend_stepper.moveTo((-((((angle / 360) * 200) / 17 ) * 31) * 8));  // Move forward again

  while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
    bend_stepper.runSpeedToPosition();
  }
  bend_stepper.setCurrentPosition(0);
}

void fixBend() {
  float bend_angle = deg_step(45);
  int touch = digitalRead(47);
  int underbend = 0;
  long error = deg_step(1) + 1;
  int dir = 0;
  float kp = 1.8;
  float kd = 0.3;
  int offset = 25;
  int threshold = deg_step(1);
  
  if (bend_angle > 0) {
    dir = 1;
  } else {
    dir = -1;
  }

  while (abs(error) > threshold) {
    touch = 0;
    underbend = 0;
    myservo.write(SERVO_MEASURE);
    while (touch != 1) {
      touch = digitalRead(47);
      bend_stepper.setSpeed(dir * 250);
      bend_stepper.run();
      underbend += 1;
      delay(10);
    }
    Serial.println("hit");

    error = abs(bend_angle) - underbend;
    float error_dir = error / abs(error);
    Serial.println(step_deg(underbend));
    Serial.println(step_deg(error));
    Serial.println(step_deg((error * kp + offset*error_dir)* dir));


    bend_stepper.move(-dir * deg_step(offset));
    bend_stepper.runToPosition();
    if (error_dir == -1) {
      myservo.write(SERVO_DOWN);
      bend_stepper.move(dir * deg_step(offset*2));
      bend_stepper.runToPosition();
    }
    myservo.write(SERVO_UP);
    delay(1000);

    bend_stepper.setSpeed(100);
    bend_stepper.move((error * kp + (deg_step(offset))*error_dir)* dir);
    bend_stepper.runToPosition();

    // move away before retracting the bend pin
    bend_stepper.move(deg_step(offset) * -dir * error_dir);
    bend_stepper.runToPosition();

    myservo.write(SERVO_DOWN);
    delay(1000);



    bend_stepper.moveTo(0);
    bend_stepper.runToPosition();
    delay(1000);
  }



  bend_stepper.moveTo(0);
  run(bend_stepper);
}