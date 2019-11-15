/*
  Mr. Bamboo
  A friendly little companion


  Pinout
  | Arduino | Hardware            |
  |---------|---------------------|
  | D1      | Gyro INT            |
  | D3      | Leaf Servo SIG      |
  | D5      | Vibra 1 INT         |
  | D6      | Vibra 2 INT         |
  | D9      | Leaf LED            |
  | A0      | Mic OUT             |
  | A4      | I2C Data            |
  | A5      | I2C Clock           |


  Created 12 October 2019
  https://github.com/pasu-ka/inter_des
*/


// include libraries

#include <Servo.h>
// #include <clock-arch.h>
// #include <lc.h>
// #include <lc-addrlabels.h>
// #include <pt-sem.h>
#include <timer.h>
// #include <clock.h>
// #include <lc-switch.h>
#include <pt.h> // threading


// pin declaration

int gyroPinDigital  = 1;
int leafServoPinPwm = 3;
int vibraOnePinPwm  = 5;
int vibraTwoPinPwm  = 6;
int leafLedPinPwm   = 9;
int micPinAnalogue  = A0;
int i2cDataPin      = A4;
int i2cClockPin     = A5;


// initialize globals

Servo leafServo;
struct timer timer;
// every protothread needs an own struct pt variable
static struct pt pt1, pt2, pt3;
unsigned int soundSample;
static unsigned long timeoutDur;
int currentState;
enum states {
  sleep,
  awake,
  happy,
  scared,
  listening
};
static bool stateChanging = false;

enum soundLevel {
  quiet,
  talking,
  bang
};


// configurations


float soundLevelThresholdWakeup     = 1.45;
float soundLevelThresholdScared     = 2.43;
int leafWiggleAngleAmount[]   = {40, 180, 0};
bool debugMode                = true;

// sample window width in mS (50 mS = 20Hz)
const int SOUND_SAMPLE_WINDOW         = 50;
const static int STATE_CHANGE_TIMEOUT = 5000;
const int VIBRA_STRONG                = 250;
const int VIBRA_MIDDLE                = 160;
const int VIBRA_LOW                   = 80;


// function declaration, because dark side of C

int soundThresholdReached();
void logDebug(String, double);
void logDebug(String);
void startDebug();
void setupLeafServo();
void goAwakeFromSleep();
void goSleep();
void goAwakeFromSleep();
void goAwakeFromListening();
void goHappy();
void goScared();
void goListening();


// run setup code

void setup() {
  if (debugMode) {
    startDebug();
  }

  currentState = sleep;

  setupLeafServo();
  /* Initialize the protothread state variables with PT_INIT(). */
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
}


// run loop (forever)
void loop() {
  listenThread(&pt1);
  timeoutThread(&pt2);
  //  moveThread(&pt3);
}


static void listenThread(struct pt *pt) {
  PT_BEGIN(pt);
  int soundLevel = 3;
  while (1) {
    PT_WAIT_UNTIL(pt, soundLevel = soundThresholdReached());
    if (!stateChanging) {
      if (soundLevel == talking) {
        if (currentState == sleep) {
          goAwakeFromSleep();
        }
      } else if (soundLevel == bang) {
        goScared();
      }
    }
  }
  PT_END(pt);
}


static void timeoutThread(struct pt *pt) {
  PT_BEGIN(pt);
  do {
    timer_set(&timer, STATE_CHANGE_TIMEOUT);
    PT_WAIT_UNTIL(pt, timer_expired(&timer));
    if (debugMode) {
      int tmp = millis();
      logDebug("timer: ", (tmp - timeoutDur) / 1000);
      timeoutDur = tmp;
    }
    logDebug("timer RINGIDINGI");
    if (!stateChanging) {
      if (currentState == awake) {
        goSleep();
        //      sleepNow();
      } else if (currentState == listening) {
        currentState = awake;
        goAwakeFromListening();
      }
    }

    // PT_WAIT_UNTIL(pt, timeoutReached());
    // do_something();
  } while (1);
  PT_END(pt); // TODO not necessary?
}


static void resetTimer() {
  logDebug("reset timer");
  //  PT_BEGIN(&pt2);
  PT_EXIT(&pt2);
  //  timeoutThread(&pt2);
}


static int moveThread(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    // TODO move code here
    // PT_WAIT_UNTIL(pt, function_returns_true());
    // do_something();
  }
  PT_END(pt);
}


void goSleep() {
  stateChanging = true;
  logDebug("going to sleep");
  currentState = sleep;
  // TODO close eyes
  // TODO lower tail
  if (debugMode) {
    vibrate(VIBRA_MIDDLE, 40, true);
  }
  resetTimer();
  stateChanging = false;
}


void goAwakeFromSleep() {
  stateChanging = true;
  logDebug("going to awake from sleep");
  currentState = awake;
  if (debugMode) {
    vibrate(VIBRA_STRONG, 20, true);
  }
  resetTimer();
  stateChanging = false;
}


void goAwakeFromListening() {

}


void goHappy() {

}


void goScared() {
  logDebug("going to scared");
}


void goListening() {

}


void vibrate(int strength, int duration, bool decreaseOverTime) {
  if (decreaseOverTime) {
    for (int i = 0; i < duration; i++) {
      analogWrite(vibraOnePinPwm, strength);
      strength = strength - 2;
      if (strength < 0) {
        break;
      }
      delay(100);
    }
  } else {

  }
  analogWrite(vibraOnePinPwm, 0);
}


// TODO two thresholds, talking, bang
int soundThresholdReached() {
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < SOUND_SAMPLE_WINDOW)
  {
    soundSample = analogRead(0);
    if (soundSample < 1024)  // toss out spurious readings
    {
      if (soundSample > signalMax)
      {
        signalMax = soundSample;  // save just the max levels
      }
      else if (soundSample < signalMin)
      {
        signalMin = soundSample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

  if (volts > soundLevelThresholdScared) {
    return bang;
  } else if (volts > soundLevelThresholdWakeup) {
    return talking;
  } else {
    return 0;
  }
}


int readSoundLevel() {
  int lvl = analogRead(micPinAnalogue);
  return analogRead(micPinAnalogue);
}


void setupLeafServo() {
  leafServo.attach(leafServoPinPwm);
  leafServo.write(0);
}


// TODO make protothread
void wiggleLeaf(int wiggleRepetition) {
  for (int i = 0; i < wiggleRepetition; i++) {
    for (int wiggleAngle : leafWiggleAngleAmount) {
      leafServo.write(wiggleAngle);
      delay(150);
    }
  }
}


static void startDebug() {
  Serial.begin(9600);
  logDebug("starting debug");
}


void microPhoneTest() {
  if (readSoundLevel() > soundLevelThresholdWakeup) {
    logDebug("HIGH");
  } else {
    logDebug("LOW");
  }
}


static void logDebug(String text) {
  if (debugMode) {
    Serial.println("### debug ###");
    Serial.println(text);
    Serial.println();
  }
}


static void logDebug(String text, double val) {
  if (debugMode) {
    Serial.println("### debug ###");
    Serial.print(text);
    Serial.println(val);
    Serial.println();
  }
}
