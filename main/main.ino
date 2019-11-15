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

#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <Wire.h>
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


// LED matrix vars
#define MATRIX_EYES         0
#define MATRIX_MOUTH_LEFT   1
#define MATRIX_MOUTH_MIDDLE 2
#define MATRIX_MOUTH_RIGHT  3
Adafruit_8x8matrix matrix[4] = { // Array of Adafruit_8x8matrix objects
  Adafruit_8x8matrix(), Adafruit_8x8matrix(),
  Adafruit_8x8matrix(), Adafruit_8x8matrix()
};

// Rather than assigning matrix addresses sequentially in a loop, each
// has a spot in this array.  This makes it easier if you inadvertently
// install one or more matrices in the wrong physical position --
// re-order the addresses in this table and you can still refer to
// matrices by index above, no other code or wiring needs to change.
static const uint8_t matrixAddr[] = { 0x70, 0x71, 0x72, 0x73 };

static const uint8_t PROGMEM // Bitmaps are stored in program memory
blinkImg[][8] = {    // Eye animation frames
  { B00111100,         // Fully open eye
    B01111110,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B01111110,
    B00111100
  },
  { B00000000,
    B01111110,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B01111110,
    B00111100
  },
  { B00000000,
    B00000000,
    B00111100,
    B11111111,
    B11111111,
    B11111111,
    B00111100,
    B00000000
  },
  { B00000000,
    B00000000,
    B00000000,
    B00111100,
    B11111111,
    B01111110,
    B00011000,
    B00000000
  },
  { B00000000,         // Fully closed eye
    B00000000,
    B00000000,
    B00000000,
    B10000001,
    B01111110,
    B00000000,
    B00000000
  }
},
mouthImg[][24] = {                 // Mouth animation frames
  { B00000000, B00000000, B00000000, // Mouth position A
    B00000000, B00000000, B00000000,
    B01111111, B11111111, B11111110,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000
  },
  { B00000000, B00000000, B00000000, // Mouth position B
    B00000000, B00000000, B00000000,
    B00111111, B11111111, B11111100,
    B00000111, B00000000, B11100000,
    B00000000, B11111111, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000
  },
  { B00000000, B00000000, B00000000, // Mouth position C
    B00000000, B00000000, B00000000,
    B00111111, B11111111, B11111100,
    B00001000, B00000000, B00010000,
    B00000110, B00000000, B01100000,
    B00000001, B11000011, B10000000,
    B00000000, B00111100, B00000000,
    B00000000, B00000000, B00000000
  },
  { B00000000, B00000000, B00000000, // Mouth position D
    B00000000, B00000000, B00000000,
    B00111111, B11111111, B11111100,
    B00100000, B00000000, B00000100,
    B00010000, B00000000, B00001000,
    B00001100, B00000000, B00110000,
    B00000011, B10000001, B11000000,
    B00000000, B01111110, B00000000
  },
  { B00000000, B00000000, B00000000, // Mouth position E
    B00000000, B00111100, B00000000,
    B00011111, B11000011, B11111000,
    B00000011, B10000001, B11000000,
    B00000000, B01111110, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000
  },
  { B00000000, B00111100, B00000000, // Mouth position F
    B00000000, B11000011, B00000000,
    B00001111, B00000000, B11110000,
    B00000001, B00000000, B10000000,
    B00000000, B11000011, B00000000,
    B00000000, B00111100, B00000000,
    B00000000, B00000000, B00000000,
    B00000000, B00000000, B00000000
  }
};

uint8_t
blinkIndex[] = { 1, 2, 3, 4, 3, 2, 1 }, // Blink bitmap sequence
               blinkCountdown = 100, // Countdown to next blink (in frames)
               gazeCountdown  =  75, // Countdown to next eye movement
               gazeFrames     =  50, // Duration of eye movement (smaller = faster)
               mouthPos       =   0, // Current image number for mouth
               mouthCountdown =  10; // Countdown to next mouth change
int8_t
eyeX = 3, eyeY = 3,   // Current eye position
newX = 3, newY = 3,   // Next eye position
dX   = 0, dY   = 0;   // Distance from prior to new position


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
void setupEyeLedMatrix();
void playWithEyesDebug();


// run setup code

void setup() {
  if (debugMode) {
    startDebug();
  }

  currentState = sleep;

  setupLeafServo();
  setupEyeLedMatrix();
  /* Initialize the protothread state variables with PT_INIT(). */
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
}


// run loop (forever)
void loop() {
  if (debugMode) {
    playWithEyesDebug();
  }
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

  // collect data for x mS
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

void setupEyeLedMatrix() {
  // Seed random number generator from an unused analog input:
  randomSeed(analogRead(A1));

  // Initialize each matrix object:
  for (uint8_t i = 0; i < 4; i++) {
    matrix[i].begin(matrixAddr[i]);
    // If using 'small' (1.2") displays vs. 'mini' (0.8"), enable this:
    // matrix[i].setRotation(3);
  }
}

// Draw mouth image across three adjacent displays
void drawMouth(const uint8_t *img) {
  for (uint8_t i = 0; i < 3; i++) {
    matrix[MATRIX_MOUTH_LEFT + i].clear();
    matrix[MATRIX_MOUTH_LEFT + i].drawBitmap(i * -8, 0, img, 24, 8, LED_ON);
  }
}

void playWithEyesDebug() {
  // Draw eyeball in current state of blinkyness (no pupil).  Note that
  // only one eye needs to be drawn.  Because the two eye matrices share
  // the same address, the same data will be received by both.
  matrix[MATRIX_EYES].clear();
  // When counting down to the next blink, show the eye in the fully-
  // open state.  On the last few counts (during the blink), look up
  // the corresponding bitmap index.
  matrix[MATRIX_EYES].drawBitmap(0, 0,
                                 blinkImg[
                                   (blinkCountdown < sizeof(blinkIndex)) ? // Currently blinking?
                                   blinkIndex[blinkCountdown] :            // Yes, look up bitmap #
                                   0                                       // No, show bitmap 0
                                 ], 8, 8, LED_ON);
  // Decrement blink counter.  At end, set random time for next blink.
  if (--blinkCountdown == 0) blinkCountdown = random(5, 180);

  // Add a pupil (2x2 black square) atop the blinky eyeball bitmap.
  // Periodically, the pupil moves to a new position...
  if (--gazeCountdown <= gazeFrames) {
    // Eyes are in motion - draw pupil at interim position
    matrix[MATRIX_EYES].fillRect(
      newX - (dX * gazeCountdown / gazeFrames),
      newY - (dY * gazeCountdown / gazeFrames),
      2, 2, LED_OFF);
    if (gazeCountdown == 0) {   // Last frame?
      eyeX = newX; eyeY = newY; // Yes.  What's new is old, then...
      do { // Pick random positions until one is within the eye circle
        newX = random(7); newY = random(7);
        dX   = newX - 3;  dY   = newY - 3;
      } while ((dX * dX + dY * dY) >= 10);     // Thank you Pythagoras
      dX            = newX - eyeX;             // Horizontal distance to move
      dY            = newY - eyeY;             // Vertical distance to move
      gazeFrames    = random(3, 15);           // Duration of eye movement
      gazeCountdown = random(gazeFrames, 120); // Count to end of next movement
    }
  } else {
    // Not in motion yet -- draw pupil at current static position
    matrix[MATRIX_EYES].fillRect(eyeX, eyeY, 2, 2, LED_OFF);
  }

  // Draw mouth, switch to new random image periodically
  drawMouth(mouthImg[mouthPos]);
  if (--mouthCountdown == 0) {
    mouthPos = random(6); // Random image
    // If the 'neutral' position was chosen, there's a 1-in-5 chance we'll
    // select a longer hold time.  This gives the appearance of periodic
    // pauses in speech (e.g. between sentences, etc.).
    mouthCountdown = ((mouthPos == 0) && (random(5) == 0)) ?
                     random(10, 40) : // Longer random duration
                     random(2, 8);    // Shorter random duration
  }

  // Refresh all of the matrices in one quick pass
  for (uint8_t i = 0; i < 4; i++) matrix[i].writeDisplay();

  delay(20); // ~50 FPS
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
