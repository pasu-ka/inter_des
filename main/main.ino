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


#include <Arduino.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Servo.h>
#include <timer.h>
// Multithreading capabilities for Arduino
#include <pt.h>
// MPU Gyro
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "declarations.h"
#include "definition.h"


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
    if (!isActive) {
      resetTimers();
    }
    if (!stateChanging) {
      if (soundLevel == talking) {
        if (currentState == sleep) {
          goAwakeFromSleep();
        } else if (currentState == awake) {
          goListening();
        } else if (currentState == listening) {
          resetTimeoutTimer(); // TODO don't reset active timer here
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
        activeAwake();
      } else if (currentState == listening) {
        activeListening();
      } else if (currentState == scared) {
        activeScared();
      } else if (currentState == happy) {
        goAwakeFromListening();
      }
    }
    isActive = false;
  } while (1);
  PT_END(pt);
}


void activeAwake() {
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
  for (int i = 60; i > 40; i--) {
    leafServo.write(i);
    delay(10);
  }
  for (int i = 40; i <= 75; i++) {
    leafServo.write(i);
    delay(10);
  }
  for (int i = 75; i >= 60; i--) {
    leafServo.write(i);
    delay(10);
  }
  leafServo.write(60);
}

void activeListening() {
  for (int i = 0; i < sizeof(smileImg) / sizeof(*smileImg); i++) {
    eyeMatrix[0].clear();

    eyeMatrix[0].drawBitmap(0, 0, blinkImg[i], 8, 8, LED_ON);

    eyeMatrix[0].writeDisplay();

    if (i % 6 == 0) {
      for (int wiggleAngle : leafWiggleAngleAmount) {
        leafServo.write(wiggleAngle);
        delay(100);
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
        delay(100);
      }
    }
  }
  for (int i = 90; i > 15; i--) {
    leafServo.write(i);
    delay(20);
  }
  for (int i = 15; i <= 90; i++) {
    leafServo.write(i);
    delay(20);
  }
}

void activeScared() {
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


static void resetTimers() {
  logDebug("reset timers");
  PT_EXIT(&timeoutPt);
  PT_EXIT(&activePt);
}

static void resetTimeoutTimer() {
  logDebug("reset timeoutTimer");
  PT_EXIT(&timeoutPt);
}


static int moveThread(struct pt *pt) {
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, movingThresholdReacher());

    if (currentState == sleep) {
      goAwakeFromSleep();
    } else if (currentState == scared) {
      goHappy();
    } else if (currentState == awake) {
      resetTimeoutTimer();
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
  }
  leafServo.write(15);
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);
  resetTimers();
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
  resetTimers();
  stateChanging = false;
}


void goAwakeFromListening() {
  stateChanging = true;
  currentState = awake;
  resetTimers();
  logDebug("going to awake from listening");
  stateChanging = false;
}


void goHappy() {
  stateChanging = true;
  currentState = happy;
  int vibraPower = 250;
  analogWrite(vibraOnePinPwm, vibraPower);
  analogWrite(vibraTwoPinPwm, vibraPower);
  eyeMatrix[0].clear();

  eyeMatrix[0].drawBitmap(0, 0, smileImg[6], 8, 8, LED_ON);

  eyeMatrix[0].writeDisplay();

  for (int wiggleAngle : leafWiggleAngleAmount) {
    vibraPower = vibraPower - 50;
    analogWrite(vibraOnePinPwm, vibraPower);
    analogWrite(vibraTwoPinPwm, vibraPower);
    leafServo.write(wiggleAngle);
    delay(100);
  }
  for (int wiggleAngle : leafWiggleAngleAmount) {
    vibraPower = vibraPower - 50;
    analogWrite(vibraOnePinPwm, vibraPower);
    analogWrite(vibraTwoPinPwm, vibraPower);
    leafServo.write(wiggleAngle);
    delay(50);
  }
  analogWrite(vibraOnePinPwm, 0);
  analogWrite(vibraTwoPinPwm, 0);
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
    vibra = vibra - 30;
    delay(100);
  }
  resetTimers();
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
  // (38400 works as well at 8MHz as it does at 16MHz)

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
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
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
