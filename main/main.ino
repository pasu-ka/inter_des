/*
  Short name or description of project
  Long description of the project. What does this project do and why was is made for example.

  Pinout
  | Arduino | Hardware            |
  |---------|---------------------|
  | D3      | Leaf Servo          |

  Created 12 October 2019
  https://github.com/pasu-ka/inter_des

*/

// include libraries
#include <Servo.h>


// initialize globals
int servoPin = 3;
Servo leafServo;
int leafWiggleAngleAmount[] = {120, 60, 100, 0};


// run setup code
void setup() {
  setupLeafServo();
}


// run loop (forever)
void loop() {

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
