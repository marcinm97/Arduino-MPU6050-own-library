/*
 Name:		testing.ino
 Created:	5/21/2018 10:35:59 PM
 Author:	Marcin
*/

// the setup function runs once when you press reset or power the board
#include <Servo.h>
#include "AutoSending.h"
#include "I2Cdev.h"

#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
void setup() {
	send.setConnectionPort();
}

// the loop function runs over and over again until power down or reset
void loop() {
	//send.setRawData();
	//send.printRawData();
	send.FIFOInitialize();
	//send.setServo();
}
