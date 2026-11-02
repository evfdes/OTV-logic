#include <Arduino.h>
#include "sensors.h"

//One time setup - connects to the vision system and usb to computer
void setup() {
  Serial.begin(9600);
  setupSensors();
}

void loop() {
  //Ethan - USE THIS AREA FOR LOOPING CODE
   getFrontDistance();
}