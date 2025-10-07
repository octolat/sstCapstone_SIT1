// Libraries

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


//Define Stepper motors 
#define enable_pin 8 // define the enable pin to make the drive high and low (All stepper motor share this pin)
AccelStepper feed_stepper(AccelStepper::DRIVER, 2, 5); // X Driver ((Don't Change), stepPin, dirPin)
AccelStepper rotate_stepper(AccelStepper::DRIVER, 3, 6); // Y Driver ((Don't Change), stepPin, dirPin)
AccelStepper bend_stepper(AccelStepper::DRIVER, 4, 7); // Z Driver ((Don't Change), stepPin, dirPin)

void setup() {
  Serial.begin(9600);

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
  bend_stepper.setMaxSpeed(1000); // Max speed of stepper
  bend_stepper.setAcceleration(700); //Set acceleration for stepper
  bend_stepper.setSpeed(1000); //Set the constant speed that the stepper will move

  // SD Card Reader
  //// Reader Initialization
  SD.begin(chipselect);
  Serial.println("Initializing SD Card...");
  if (!SD.begin(chipselect)) {
    Serial.println("Initialization failed");
    // while (1);
  } else {
    Serial.println("Initialization done");
  }

  //// Reading SD Card
  File file = SD.open("rotate.txt");
  if (!file) {
    Serial.println("Error opening file!");   // File does not exist
  } else {
    if (file.size() == 0) {
      Serial.println("File exists but is empty!");
    } else {
      while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line);
        // Serial.println(line[0]);
        // Serial.println(line.substring(1));
        funct = line[0];
        dir = line[1];
        Serial.println(funct);
        if (funct == "F") {
          centi_amt = line.substring(2).toFloat();
          Serial.println((centi_amt / (33.33*pi)) * 200);
          feed_stepper.moveTo((-(centi_amt / (33.33*pi)) * 200)); // (2.76 * pi) / 200
          while (feed_stepper.distanceToGo() != 0) {
            feed_stepper.runSpeedToPosition();
          }
          delay(2000);
        } else if (funct == "R") {
          degree_amt = line.substring(2).toFloat();
          Serial.println((((degree_amt / 360) * 200) / 24) * 60);
          if (dir == "-") {
            rotate_stepper.moveTo((((degree_amt / 360) * 200) / 24) * 60);
            while (rotate_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              rotate_stepper.runSpeedToPosition();
            }
          } else {
            rotate_stepper.moveTo((-(((degree_amt / 360) * 200) / 24) * 60));
            while (rotate_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              rotate_stepper.runSpeedToPosition();
            }
          }
          // rotate_stepper.runToPosition();
          delay(2000);
        } else if (funct == "B") {
          degree_amt = line.substring(2).toFloat();
          if (dir == "-") {
            bend_stepper.moveTo((degree_amt/360)*200);
            while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              bend_stepper.runSpeedToPosition();
            }
          } else {
            bend_stepper.moveTo((-(degree_amt/360)*200));
            while (bend_stepper.distanceToGo() != 0) { // Make stepper turn until it reaches 0 (it starts from whatever u setted in moveTo())
              bend_stepper.runSpeedToPosition();
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