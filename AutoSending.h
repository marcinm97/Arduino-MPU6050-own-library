// AutoSending.h

#ifndef _AUTOSENDING_h
#define _AUTOSENDING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include "I2Cdev.h"
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

struct Vector {
	int16_t X;
	int16_t Y;
	int16_t Z;
};
extern volatile bool mpuInterrupt;
void dmpDataReady();
class AutoSending
{
public:
	enum class mode { YAW = 1, PITCH, ROLL, STD, DFT };   // STD(standard) < -- only Yaw and Pitch, because our gimbal will be using this axies  
	// DFT (default) every axis
 private:
	 MPU6050 mpu;
	 Vector gyr;
	 Vector acc; 
	 bool blinkState;
	 mode state;
	  
		 // MPU control/status vars
	 bool dmpReady;  // set true if DMP init was successful
	 uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	 uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	 uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	 uint16_t fifoCount;     // count of all bytes currently in FIFO
	 uint8_t fifoBuffer[64]; // FIFO storage buffer
	 Quaternion q;           // [w, x, y, z]         quaternion container
	 VectorFloat gravity;    // [x, y, z]            gravity vector
	 float ypr[3];	     // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	 static bool ifSetConnection;
	 static bool ifDownload;
	 Servo yawX;
	 Servo pitchY;
 public:
	 AutoSending();
	 float getGyrX();
	 float getGyrY();
	 float getGyrZ();
	 float getAccX();
	 float getAccY();
	 float getAccZ();
	 float getYaw();
	 float getPitch();
	 float getRoll();
	 // switch and enum to rotate which component we're using
	 void setRawData();              // downloading raw data from port
	 void setConnectionPort();       // setting a port
	 void printRawData();           // printing raw data at ardu monitor
	 void FIFOInitialize();
	 void printYawPitchRoll();
	 void setServo(mode status = mode::STD);  // default is STD (YAW and PITCH) (we need it to our gimbal)
	 // function to send data from one module to other one
};


extern AutoSending send;

#endif

