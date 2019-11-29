#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Arduino.h>
// #include <Wire.h>
#include <Servo.h>
// #include <clock-arch.h>
// #include <lc.h>
// #include <lc-addrlabels.h>
// #include <pt-sem.h>
#include <timer.h>
// #include <clock.h>
// #include <lc-switch.h>
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
