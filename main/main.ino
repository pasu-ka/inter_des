/*
  Short name or description of project
  Long description of the project. What does this project do and why was is made for example.

  Pinout
  | Arduino | Hardware            |
  |---------|---------------------|
  | D3      | Leaf Servo          |
  | A0      | Microphone          |

  Created 12 October 2019
  https://github.com/pasu-ka/inter_des

*/

// include libraries
#include <Servo.h>


// initialize globals
Servo leafServo;

int servoPin = 3;
int micPinAnalogue = A0;
int leafWiggleAngleAmount[] = {120, 60, 100, 0};
int soundLevelThreshold = 700;

bool debugMode = false;


// run setup code
void setup() {
  startDebug();
  setupLeafServo();
}


// run loop (forever)
void loop() {
  if(debugMode) {
    microPhoneTest();
  }
}


int readSoundLevel() {
  return analogRead(micPinAnalogue);
}


void setupLeafServo() {
  leafServo.attach(servoPin);
  leafServo.write(0);
}


void wiggleLeaf(int wiggleRepetition) {
  for(int i = 0; i < wiggleRepetition; i++) {
    for(int wiggleAngle : leafWiggleAngleAmount) {
      leafServo.write(wiggleAngle);
      delay(300);
    }
  }
}


void startDebug() {
  debugMode = true;
  Serial.begin(9600);
}


void microPhoneTest() {
  if (readSoundLevel() > soundLevelThreshold){
    logDebug("HIGH");
  } else {
    logDebug("LOW");
  }
}


void logDebug(String text) {
  Serial.println(text);
}
