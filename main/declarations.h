Servo leafServo;
struct timer timeoutTimer, activeTimer;
// every protothread needs an own struct pt variable
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

// uint8_t
// blinkIndex[] = { 1, 2, 3, 4, 3, 2, 1 }, // Blink bitmap sequence
//                blinkCountdown = 100, // Countdown to next blink (in frames)
//                gazeCountdown  =  75, // Countdown to next eye movement
//                gazeFrames     =  50, // Duration of eye movement (smaller = faster)
//                mouthPos       =   0, // Current image number for mouth
//                mouthCountdown =  10; // Countdown to next mouth change


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
