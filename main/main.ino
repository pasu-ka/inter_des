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
int currentState;
enum states {
  sleep,
  awake,
  happy,
  scared,
  listening
};
static bool stateChanging = false;


// configurations


float soundLevelThreshold     = 2.4;
int leafWiggleAngleAmount[]   = {40, 180, 0};
bool debugMode                = true;

// sample window width in mS (50 mS = 20Hz)
const int SOUND_SAMPLE_WINDOW   = 100;
const static int STATE_CHANGE_TIMEOUT = 5000;
const int VIBRA_STRONG = 250;
const int VIBRA_MIDDLE = 160;
const int VIBRA_LOW = 80;


// function declaration, because dark side of C

bool soundThresholdReached();
void logDebug(String text, int val);
void logDebug(String text);
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
  while (1) {
    PT_WAIT_UNTIL(pt, soundThresholdReached());
    if (!stateChanging) {
      if (currentState == sleep) {
        logDebug("waking up");
        goAwakeFromSleep();
      } else if (currentState == listening) {
        logDebug("go to wake");
        goAwakeFromListening();
      }

    }
  }
  PT_END(pt);
}

static unsigned long timeoutDur;

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
        logDebug("going to sleep from awake");
        //      sleepNow();
      } else if (currentState == listening) {
        currentState = awake;
        logDebug("going to awake from listening");
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

//void changeState(state) {
//  stateChanging = true;
//  switch (state) {
//  case sleep:
//    goSleep();
//    break;
//  case awake:
//    goAwake();
//    break;
//  case happy:
//    goHappy();
//  case scared:
//    goScared();
//  case listening:
//    goListening();
//  } final {
//    stateChanging = false;
//  }
//}


void goSleep() {
  stateChanging = true;
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


//void checkState() {
//  switch (currentState) {
//    case sleep:
//      checkSleepState();
//      break;
//    case awake:
//      checkAwakeState();
//      break;
//    case happy:
//      break;
//    case scared:
//      break;
//    case listening:
//      break;
//  }
//  // TODO implement
//}

void timeoutReached() {
  return;
}

//void checkSleepState() {
//  //TODO pickup || noise
//  if (soundThresholdReached()) {
//    wakeUp();
//  }
//}


//void checkAwakeState() {
//  //TODO do right
//  if (!soundThresholdReached()) {
//    sleepNow();
//  }
//}


//void sleepNow() {
//  currentState = sleep;
//  resetTimer();
//}


//void wakeUp() {
//  currentState = awake;
//  wiggleLeaf(1);
//  //  resetTimer();
//}

//void goAwake() {
//  currentState = awake;
//  resetTimer();
//}


// TODO two thresholds, talking, bang
bool soundThresholdReached() {
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

  //  Serial.println(volts);
  return volts > soundLevelThreshold;
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
  if (readSoundLevel() > soundLevelThreshold) {
    logDebug("HIGH");
  } else {
    logDebug("LOW");
  }
}


static void logDebug(String text) {
  if (debugMode) {
    Serial.println("### debug ###");
    Serial.println(text);
  }
}

static void logDebug(String text, int val) {
  if (debugMode) {
    Serial.println("### debug ###");
    Serial.print(text);
    Serial.println(val);
  }
}

//int main(void) {
//  setup();
//  startDebug();
//  setupLeafServo();
//  /* Initialize the protothread state variables with PT_INIT(). */
//  PT_INIT(&pt1);
//  PT_INIT(&pt2);
//  PT_INIT(&pt3);
//  /* Then we schedule the two protothreads by repeatedly calling their
//    protothread functions and passing a pointer to the protothread
//    state variables as arguments.     */
//  // while(1) {
//  listenThread(&pt1);
//  timeoutThread(&pt2);
//  moveThread(&pt3);
//  // }
//}
