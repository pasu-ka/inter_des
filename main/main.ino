/*
  Short name or description of project
  Long description of the project. What does this project do and why was is made for example.

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


// initialize globals
Servo leafServo;

int servoPin = 3;
int micPinAnalogue = A0;

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

int leafWiggleAngleAmount[] = {40, 180, 0};
int soundLevelThreshold = 80;
enum states {
  sleep,
  awake,
  happy,
  scared,
  listening
};
int currentState;

bool debugMode = false;

static int TIMEOUT = 5000;
struct timer timer;


// every protothread needs an own struct pt variable
static struct pt pt1, pt2, pt3;

// function declaration, because C is...

bool soundTresholdReached();
void logDebug(String text, int val);
void logDebug(String text);

// run setup code
 void setup() {
 currentState = sleep;

 startDebug();
  setupLeafServo();
  /* Initialize the protothread state variables with PT_INIT(). */
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
  
 }


// run loop (forever)
 void loop() {
//   if(debugMode) {
//     microPhoneTest();
//     if(currentState == awake) {
//       wiggleLeaf(1);
//     }
//   }
//   checkState();
  listenThread(&pt1);
  timeoutThread(&pt2);
//  moveThread(&pt3);
//  sampleAudio();

  analogWrite(5, 200);                                        // 진동모터를 200/255의 파워로 작동시킵니다.

  delay(3000);                                                    // 3초간 대기

  analogWrite(5, 100);                                       // 진동모터를 100/255의 파워로 작동시킵니다.

  delay(3000);                                                    // 3초간 대기

  analogWrite(5, 0);                                           // 진동모터를 0의 파워로 작동시킵니다. (OFF)

  delay(3000);      
 }

//void sampleAudio() {
//  unsigned long startMillis= millis();  // Start of sample window
//   unsigned int peakToPeak = 0;   // peak-to-peak level
// 
//   unsigned int signalMax = 0;
//   unsigned int signalMin = 1024;
// 
//   // collect data for 50 mS
//   while (millis() - startMillis < sampleWindow)
//   {
//      sample = analogRead(0);
//      if (sample < 1024)  // toss out spurious readings
//      {
//         if (sample > signalMax)
//         {
//            signalMax = sample;  // save just the max levels
//         }
//         else if (sample < signalMin)
//         {
//            signalMin = sample;  // save just the min levels
//         }
//      }
//   }
//   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
//   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
// 
//   Serial.println(volts);
//}

static int listenThread(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, soundTresholdReached());
    logDebug("waking up");
    wakeUp();
  }
  PT_END(pt);
}

static int timeoutThread(struct pt *pt) {
  PT_BEGIN(pt);
  do {
    timer_set(&timer, TIMEOUT);
    PT_WAIT_UNTIL(pt, timer_expired(&timer));
    if (currentState == awake) {
      currentState = sleep;
     logDebug("going to sleep from awake");
//      sleepNow();
    } else if (currentState == listening) {
      currentState = listening;
     logDebug("going to awake from listening");
//      goAwake();
    }

    // PT_WAIT_UNTIL(pt, timeoutReached());
    // do_something();
  } while (1);
  PT_END(pt); // TODO not necessary?
}

static void resetTimer() {
 logDebug("reset timer");
 // PT_END(&pt2);
 // timeoutThread(&pt2);
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

void checkState() {
  switch (currentState) {
    case sleep:
      checkSleepState();
      break;
    case awake:
      checkAwakeState();
      break;
    case happy:
      break;
    case scared:
      break;
    case listening:
      break;
  }
  // TODO implement
}

void timeoutReached() {
  return;
}

void checkSleepState() {
  //TODO pickup || noise
  if (soundTresholdReached()) {
    wakeUp();
  }
}


void checkAwakeState() {
  //TODO do right
  if (!soundTresholdReached()) {
    sleepNow();
  }
}


void sleepNow() {
  currentState = sleep;
  resetTimer();
}


void wakeUp() {
  currentState = awake;
  wiggleLeaf(1);
//  resetTimer();
}

void goAwake() {
  currentState = awake;
  resetTimer();
}


// TODO two thresholds, talking, bang
bool soundTresholdReached() {
  return readSoundLevel() > soundLevelThreshold;
}

int readSoundLevel() {
  int lvl = analogRead(micPinAnalogue);
  return analogRead(micPinAnalogue);
}


void setupLeafServo() {
  leafServo.attach(servoPin);
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
  debugMode = true;
  Serial.begin(9600);
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
