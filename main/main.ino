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

#include "includes.h"
#include "declarations.h"
#include "definition.h"


// pin mapping

int gyroPinDigital  = 1;
int leafServoPinPwm = 3;
int vibraOnePinPwm  = 5;
int vibraTwoPinPwm  = 6;
int leafLedPinPwm   = 9;
int micPinAnalogue  = A0;
int i2cDataPin      = A4;
int i2cClockPin     = A5;


// configurations

bool debugMode                          = true;
// max lvl = 1024
static bool isActive = false;
double soundLevelThresholdWakeup        = 2.2;
double soundLevelThresholdScared        = 2.71;
int leafWiggleAngleAmount[]             = {15, 90};
const int WAKEUP_G_FORCE                = 25;
// sample window width in mS (50 mS = 20Hz)
const int SOUND_SAMPLE_WINDOW           = 50;
const static int STATE_CHANGE_TIMEOUT   = 10000;
const static int ACTIVE_STATE_TIMEOUT   = 3000;
const int VIBRA_STRONG                  = 250;
const int VIBRA_MIDDLE                  = 160;
const int VIBRA_LOW                     = 80;
static const uint8_t eyeMatrixAddr[]    = {0x70, 0x71};
int8_t
eyeX = 3, eyeY = 3,   // Current eye position
newX = 3, newY = 3,   // Next eye position
dX   = 0, dY   = 0;   // Distance from prior to new position


void setup() {
  Serial.begin(9600);

  setupI2C();
  setupLeafServo();
  setupEyeLedMatrix();
  setupMpu();

  if (debugMode) {
    pinMode(leafLedPinPwm, OUTPUT);
    digitalWrite(leafLedPinPwm, HIGH);
  }
  currentState = sleep;

  /* Initialize the protothread state variables with PT_INIT(). */
  PT_INIT(&listenPt);
  PT_INIT(&timeoutPt);
  PT_INIT(&movePt);
  PT_INIT(&activePt);
}


void loop() {
  if (debugMode) {
    //    playWithEyesDebug();
    //    wiggleLeaf(1);
//    eyeMatrix[0].clear();
//
//      eyeMatrix[0].drawBitmap(0, 0, blinkImg2[0], 8, 8, LED_ON);
//
//      eyeMatrix[0].writeDisplay();
//    delay(5000);
//      eyeMatrix[0].clear();
//
//      eyeMatrix[0].drawBitmap(0, 0, blinkImg2[3], 8, 8, LED_ON);
//
//      eyeMatrix[0].writeDisplay();
//      
//      analogWrite(vibraOnePinPwm, 220);
//      analogWrite(vibraTwoPinPwm, 220);
//      leafServo.write(90);
//      delay(1500);
//      analogWrite(vibraOnePinPwm, 0);
//      analogWrite(vibraTwoPinPwm, 0);
//      leafServo.write(45);
      
  }
  listenThread(&listenPt);
  timeoutThread(&timeoutPt);
  moveThread(&movePt);
  activeThread(&activePt);

}


static void listenThread(struct pt *pt) {
  PT_BEGIN(pt);
  int soundLevel;
  while (1) {
    PT_WAIT_UNTIL(pt, soundLevel = soundThresholdReached());
    if(!isActive) {
      resetTimer();
    }
    if (!stateChanging) {
      if (soundLevel == talking) {
        if (currentState == sleep) {
          goAwakeFromSleep();
        } else if (currentState == awake) {
          goListening();
        } else if (currentState == listening) {
          resetTimer(); // TODO don't reset active timer here
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
    timer_set(&timeoutTimer, STATE_CHANGE_TIMEOUT);
    PT_WAIT_UNTIL(pt, timer_expired(&timeoutTimer));
    if (debugMode) {
      int tmp = millis();
//      logDebug("timeoutTimer: ", (tmp - timeoutDur) / 1000);
      timeoutDur = tmp;
    }
        logDebug("timeoutTimer RINGIDINGI");
    if (!stateChanging) {
      if (currentState == awake) {
        goSleep();
        //      sleepNow();
      } else if (currentState == listening) {
        goAwakeFromListening();
      } else if (currentState == scared) {
        goAwakeFromListening();
      } else if (currentState == happy) {
        goAwakeFromListening();
      }
    }
  } while (1);
  PT_END(pt);
}

static void activeThread(struct pt *pt) {
  PT_BEGIN(pt);
  do {
    timer_set(&activeTimer, ACTIVE_STATE_TIMEOUT);
    PT_WAIT_UNTIL(pt, timer_expired(&activeTimer));
    if (debugMode) {
      int tmp = millis();
//      logDebug("activeTimer: ", (tmp - activeDur) / 1000);
      activeDur = tmp;
    }
    isActive = true;
    if (!stateChanging) {
      if (currentState == awake) {
        for (int i = 0; i < sizeof(blinkImg) / sizeof(*blinkImg); i++) {
          eyeMatrix[0].clear();

          eyeMatrix[0].drawBitmap(0, 0, blinkImg[i], 8, 8, LED_ON);

          eyeMatrix[0].writeDisplay();

          if (i % 4 == 0) {
            for (int wiggleAngle : leafWiggleAngleAmount) {
              leafServo.write(wiggleAngle);
              delay(50);
            }
          }
        }
        leafServo.write(60);
      } else if (currentState == listening) {
        for (int i = 0; i < sizeof(smileImg) / sizeof(*smileImg); i++) {
          eyeMatrix[0].clear();

          eyeMatrix[0].drawBitmap(0, 0, blinkImg[i], 8, 8, LED_ON);

          eyeMatrix[0].writeDisplay();

          if (i % 4 == 0) {
            for (int wiggleAngle : leafWiggleAngleAmount) {
              leafServo.write(wiggleAngle);
              delay(50);
            }
          }
        }
        for (int i = sizeof(smileImg) / sizeof(*smileImg); i >= 0; i--) {
          eyeMatrix[0].clear();

          eyeMatrix[0].drawBitmap(0, 0, smileImg[i], 8, 8, LED_ON);

          eyeMatrix[0].writeDisplay();

          if (i % 6 == 0) {
            for (int wiggleAngle : leafWiggleAngleAmount) {
              leafServo.write(wiggleAngle);
              delay(50);
            }
          }
        }
      } else if (currentState == scared) {
        logDebug("scared little shit");
        
//        analogWrite(vibraOnePinPwm, 100);
//        analogWrite(vibraTwoPinPwm, 100);
        for (int i = 0; i < sizeof(surprisedImg) / sizeof(*surprisedImg); i++) {
          eyeMatrix[0].clear();

          eyeMatrix[0].drawBitmap(0, 0, surprisedImg[i], 8, 8, LED_ON);

          eyeMatrix[0].writeDisplay();

          if (i % 2 == 0) {
            for (int wiggleAngle : leafWiggleAngleAmount) {
              leafServo.write(wiggleAngle);
              delay(50);
            }
          }
        }
      }
    }
    isActive = false;
  } while (1);
  PT_END(pt);
}


static void resetTimer() {
  logDebug("reset timeoutTimer");
  PT_EXIT(&timeoutPt);
  PT_EXIT(&activePt);
}


static int moveThread(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, movingThresholdReacher());
    
    if (currentState == sleep) {
      goAwakeFromSleep();
    } else if (currentState == scared) {
      goHappy();
    }
  }
  PT_END(pt);
}


void goSleep() {
  stateChanging = true;
  logDebug("going to sleep");
  currentState = sleep;
  int vibra = 150;
  int leafPos = 80;
  int brightness = 14;
  for (int i = 0; i < (sizeof(sleepImg) / sizeof(*sleepImg)); i++) {
    eyeMatrix[0].clear();

    eyeMatrix[0].drawBitmap(0, 0, sleepImg[i], 8, 8, LED_ON);
    
    eyeMatrix[0].setBrightness(brightness);
    eyeMatrix[0].writeDisplay();
    analogWrite(vibraOnePinPwm, vibra);
    analogWrite(vibraTwoPinPwm, vibra);
    if (leafPos > 15) {
      leafServo.write(leafPos);
    }
    brightness = brightness - 2;
    leafPos = leafPos - 6;
    vibra = vibra - 20;
    delay(300);
    logDebug("are we here?!?!");
  }
  leafServo.write(15);
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);
  resetTimer();
  stateChanging = false;
  logDebug("finished");
}


void goAwakeFromSleep() {
  stateChanging = true;
  currentState = awake;
  logDebug("going to awake from sleep");

  int duration = 10;
  int strength = VIBRA_MIDDLE;
  bool uglyBool = true;

  for (int i = 0; i < duration; i++) {
    analogWrite(vibraOnePinPwm, strength);
    analogWrite(vibraTwoPinPwm, strength);
    strength = strength - 10;
    eyeMatrix[0].setBrightness(16);
    if (i < (sizeof(blinkImg) / sizeof(*blinkImg)) && uglyBool) {
      eyeMatrix[0].clear();

      eyeMatrix[0].drawBitmap(0, 0, blinkImg[i], 8, 8, LED_ON);

      eyeMatrix[0].writeDisplay();
    }
    for (int wiggleAngle : leafWiggleAngleAmount) {
      leafServo.write(wiggleAngle);
      delay(100);
    }
    if (strength < 0) {
      analogWrite(vibraOnePinPwm, 0);
      analogWrite(vibraTwoPinPwm, 0);
      break;
    }
  }
  leafServo.write(60);
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);
  resetTimer();
  stateChanging = false;
}


void goAwakeFromListening() {
  stateChanging = true;
  currentState = awake;
  resetTimer();
  logDebug("going to awake from listening");
  stateChanging = false;
}


void goHappy() {
  stateChanging = true;
  currentState = happy;
  eyeMatrix[0].clear();

      eyeMatrix[0].drawBitmap(0, 0, smileImg[6], 8, 8, LED_ON);

      eyeMatrix[0].writeDisplay();
  logDebug("going to happy");
  stateChanging = false;
}


void goScared() {
  stateChanging = true;

  currentState = scared;
  logDebug("going to scared");
  int vibra = 250;
  for (int i = 0; i < (sizeof(surprisedImg) / sizeof(*surprisedImg)); i++) {
    eyeMatrix[0].clear();

    eyeMatrix[0].drawBitmap(0, 0, surprisedImg[i], 8, 8, LED_ON);

    eyeMatrix[0].writeDisplay();
    analogWrite(vibraOnePinPwm, vibra);
    analogWrite(vibraTwoPinPwm, vibra);
    vibra = vibra - 50;
    delay(100);
  }
  resetTimer();
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);

  stateChanging = false;
}


void goListening() {
  stateChanging = true;
  currentState = listening;
  logDebug("going to listening");
  stateChanging = false;
}


void vibrate(int strength, int duration, bool decreaseOverTime) {
  if (decreaseOverTime) {
    for (int i = 0; i < duration; i++) {
      analogWrite(vibraOnePinPwm, strength);
      analogWrite(vibraTwoPinPwm, strength);
      strength = strength - 2;
      if (strength < 0) {
        break;
      }
      delay(100);
    }
  } else {

  }
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);
}


static void wiggleLeaf(int wiggleRepetition) {
  for (int i = 0; i < wiggleRepetition; i++) {
    for (int wiggleAngle : leafWiggleAngleAmount) {
      leafServo.write(wiggleAngle);
      delay(250);
    }
  }
}


bool movingThresholdReacher() {
  // read raw accel/gyro measurements from device
  //  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  //     int val = map(gz, -32768, +32767, 0, 255);

  //    leafServo.write(val);
  //    delay(1);

  // these methods (and a few others) are also available
  //  accelgyro.getAcceleration(&ax, &ay, &az);
//  logDebug("fuck");
  accelgyro.getRotation(&gx, &gy, &gz);
//  logDebug("this");

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  //        Serial.print("a/g:\t");
  //        Serial.print(ax); Serial.print("\t");
  //        Serial.print(ay); Serial.print("\t");
  //        Serial.print(az); Serial.print("\t");
  //        Serial.println();
  //  Serial.print(gx); Serial.print("\t");
  //  Serial.print(gy); Serial.print("\t");
  //  Serial.println(gz);

  // map to angleº
  int mappedX = map(gx, -32768, +32767, -180, 180);
  int mappedY = map(gy, -32768, +32767, -180, 180);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  //        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  //        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  //        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  //  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  //  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  //  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  //  Serial.println();
#endif
  //  logDebug("X angle: ", mappedX);
  //  logDebug("Y angle: ", mappedY);
//  delay(100);
  //  logDebug("gggggggg: ", gx);
  return (mappedX < -WAKEUP_G_FORCE || mappedX > WAKEUP_G_FORCE) || (mappedY < -WAKEUP_G_FORCE || mappedY > WAKEUP_G_FORCE);
}


int soundThresholdReached() {
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;  // peak-to-peak level
//  unsigned int mean = 0;  // average level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for x mS
  while (millis() - startMillis < SOUND_SAMPLE_WINDOW)
  {
    soundSample = analogRead(micPinAnalogue);
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
//  mean = (signalMax + signalMin) / 2;
    double mean = (peakToPeak * 5.0) / 1024;  // convert to volts
  //    logDebug("bleb",volts);
//  logDebug("sound level: ", mean);
  if (mean >= soundLevelThresholdScared) {
    return bang;
  } else if (mean >= soundLevelThresholdWakeup) {
    return talking;
  } else {
    return 0;
  }
}


void setupLeafServo() {
  leafServo.attach(leafServoPinPwm);
  leafServo.write(15);
}


void setupEyeLedMatrix() {
  // Seed random number generator from an unused analog input:
  // randomSeed(analogRead(A1)); not used anymore TODO maybe future

  // Initialize each matrix object:
  for (uint8_t i = 0; i < 4; i++) {
    eyeMatrix[i].begin(eyeMatrixAddr[i]);
    // If using 'small' (1.2") displays vs. 'mini' (0.8"), enable this:
    // eyeMatrix[i].setRotation(3);
  }
}


void debugMpu() {
  // read raw accel/gyro measurements from device
  //  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //    int val = map(ax, -32768, +32767, 0, 255);
  //     int val = map(gz, -32768, +32767, 0, 255);

  //    leafServo.write(val);
  //    delay(1);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  accelgyro.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  //        Serial.print("a/g:\t");
  //        Serial.print(ax); Serial.print("\t");
  //        Serial.print(ay); Serial.print("\t");
  //        Serial.print(az); Serial.print("\t");
  //        Serial.println();
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  //        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  //        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  //        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  Serial.println();
#endif
  delay(50);
}


void playWithEyesDebug() {
  logDebug("fuck ", (sizeof(blinkImg) / sizeof(*blinkImg)));
  for (int i = 0; i < (sizeof(blinkImg) / sizeof(*blinkImg)); i++) {
    eyeMatrix[0].clear();

    eyeMatrix[0].drawBitmap(0, 0, blinkImg[i], 8, 8, LED_ON);

    eyeMatrix[0].writeDisplay();

    delay(500);
  }
}


// void playWithEyesDebug2() {
//   // Draw eyeball in current state of blinkyness (no pupil).  Note that
//   // only one eye needs to be drawn.  Because the two eye matrices share
//   // the same address, the same data will be received by both.
//   matrix[MATRIX_EYES].clear();
//   // When counting down to the next blink, show the eye in the fully-
//   // open state.  On the last few counts (during the blink), look up
//   // the corresponding bitmap index.
//   matrix[MATRIX_EYES].drawBitmap(0, 0,
//                                  blinkImg[
//                                    (blinkCountdown < sizeof(blinkIndex)) ? // Currently blinking?
//                                    blinkIndex[blinkCountdown] :            // Yes, look up bitmap #
//                                    0                                       // No, show bitmap 0
//                                  ], 8, 8, LED_ON);
//   // Decrement blink counter.  At end, set random time for next blink.
//   if (--blinkCountdown == 0) blinkCountdown = random(5, 180);
//
//   // Add a pupil (2x2 black square) atop the blinky eyeball bitmap.
//   // Periodically, the pupil moves to a new position...
//   if (--gazeCountdown <= gazeFrames) {
//     // Eyes are in motion - draw pupil at interim position
//     matrix[MATRIX_EYES].fillRect(
//       newX - (dX * gazeCountdown / gazeFrames),
//       newY - (dY * gazeCountdown / gazeFrames),
//       2, 2, LED_OFF);
//     if (gazeCountdown == 0) {   // Last frame?
//       eyeX = newX; eyeY = newY; // Yes.  What's new is old, then...
//       do { // Pick random positions until one is within the eye circle
//         newX = random(7); newY = random(7);
//         dX   = newX - 3;  dY   = newY - 3;
//       } while ((dX * dX + dY * dY) >= 10);     // Thank you Pythagoras
//       dX            = newX - eyeX;             // Horizontal distance to move
//       dY            = newY - eyeY;             // Vertical distance to move
//       gazeFrames    = random(3, 15);           // Duration of eye movement
//       gazeCountdown = random(gazeFrames, 120); // Count to end of next movement
//     }
//   } else {
//     // Not in motion yet -- draw pupil at current static position
//     matrix[MATRIX_EYES].fillRect(eyeX, eyeY, 2, 2, LED_OFF);
//   }
//   // Refresh all of the matrices in one quick pass
//   for (uint8_t i = 0; i < 4; i++) matrix[i].writeDisplay();
//
//   delay(20); // ~50 FPS
// }


void setupI2C() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}


void setupMpu() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  //    Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
  */
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
