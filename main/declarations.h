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

bool debugMode                          = false;
// max lvl = 1024
static bool isActive = false;
double soundLevelThresholdWakeup        = 2.1;
double soundLevelThresholdScared        = 2.68;
int leafWiggleAngleAmount[]             = {5, 50};
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


// vars

Servo leafServo;
struct timer timeoutTimer, activeTimer;
// every protothread needs their own struct pt variable
static struct pt listenPt, timeoutPt, movePt, activePt;
unsigned int soundSample;
static unsigned long timeoutDur;
static unsigned long activeDur;
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

// led matrix vars

#define MATRIX_EYES 2
Adafruit_8x8matrix eyeMatrix[2] = { // Array of Adafruit_8x8matrix objects
  Adafruit_8x8matrix(),
  Adafruit_8x8matrix()
};

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO


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
void playWithEyesDebug2();
void setupMpu();
bool movingThresholdReacher();
void setupI2C();
void activeAwake();
void activeListening();
void activeScared();
