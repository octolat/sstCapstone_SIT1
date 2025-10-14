// Libraries
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <stdio.h>

using namespace std;


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


//Test to find center of pin
void setPin (float angle) {
  bend_stepper.moveTo((-((((angle / 360) * 200) / 17 ) * 31) * 8));  // Move forward again

  while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
    bend_stepper.runSpeedToPosition();
  }
  bend_stepper.setCurrentPosition(0);
}

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
          centi_amt = line.substring(1).toFloat();
          Serial.println((centi_amt / (33.33*pi)) * 200);
          feed_stepper.move((-((centi_amt / (33.33*pi)) * 200)));
          while (feed_stepper.distanceToGo() != 0) {
            feed_stepper.runSpeedToPosition();
          }
          feed_stepper.setCurrentPosition(0);
          delay(2000);
        } else if (funct == "R") {
          degree_amt = line.substring(2).toFloat();
          Serial.println((((degree_amt / 360) * 200) / 24) * 60);
          if (dir == "-") {
            rotate_stepper.move((((degree_amt / 360) * 200) / 24) * 60);
            while (rotate_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              rotate_stepper.runSpeedToPosition();
            }
          } else {
            rotate_stepper.move((-(((degree_amt / 360) * 200) / 24) * 60));
            while (rotate_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              rotate_stepper.runSpeedToPosition();
            }
          }
          // rotate_stepper.runToPosition();
          delay(2000);
        } else if (funct == "B") {
          degree_amt = line.substring(2).toFloat();
          if (dir == "-") {
            // Offset pin 
            bend_stepper.move((-((((9.0 / 360.0) * 200.0) / 17.0 ) * 31.0) * 8.0));
            bend_stepper.runToPosition();
            delay(1000);

            // Bending
            if (bend_stepper.currentPosition() != 0) {
              myservo.write(120);
              delay(500);
              float bend_angle = (((((degree_amt / 360) * 200) / 17) * 31) * 8);
              bend_stepper.moveTo(bend_angle);  // Move forward again
              while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
                bend_stepper.runSpeedToPosition();
              }
              myservo.write(0); //servo down
              delay(500);

              //read flex sensor
              // int underbend = analogRead(A8);
              //int value = analogRead(A1);
              // values[it] = value;
              // if (it == num-1) {
              //   it = 0;
              // } else {
              //   it++;
              // }


              // int sum = 0;
              // for (int i =0; i < num; i++) {
              //   sum += values[i];
              // }

              // Serial.println(sum/num);

              //if underbend
              // while (underbend >= 100){
              //   myservo.write(120);
              //   bend_stepper.moveTo(bend_angle+4.44);
              //   delay(500);
              //   myservo.write(0);
              //   bend_stepper.moveTo(bend_angle+4.44+10);
              //   int underbend = analogRead(A8);
              //   delay(500);
              // }

              bend_stepper.moveTo(0); //turn back
              while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
                bend_stepper.runSpeedToPosition();
              }
            }
          } else {
            // Offset pin 
            bend_stepper.move(((((9.0 / 360.0) * 200.0) / 17.0 ) * 31.0) * 8.0);
            bend_stepper.runToPosition();
            delay(1000);

            // Bending
            if (bend_stepper.currentPosition() != 0) {
              myservo.write(120);
              delay(500);
              bend_stepper.moveTo((-(((degree_amt / 360) * 200) / 17) * 31) * 8);  // Move forward again
              while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
                bend_stepper.runSpeedToPosition();
              }
              myservo.write(0);
              delay(500);
              bend_stepper.moveTo(0);
              while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
                bend_stepper.runSpeedToPosition();
              }
            }
          }
          delay(2000);
        }
      }
    }
  }
}

void loop () {

}