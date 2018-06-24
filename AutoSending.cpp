// 
// 
// 

#include "AutoSending.h"
bool AutoSending::ifSetConnection = false;
bool AutoSending::ifDownload = false;
AutoSending::AutoSending(): mpu(), q(), gravity() {
	gyr.X = 0.0f;
	gyr.Z = 0.0f;
	gyr.Y = 0.0f;
	acc.X = 0.0f;
	acc.Z = 0.0f;
	acc.Y = 0.0f;
	blinkState = false;
	dmpReady = false;
}
float AutoSending::getGyrX() {
	return (float)gyr.X; // change data to float
}
float AutoSending::getGyrY() {
	return (float)gyr.Y;
}
float AutoSending::getGyrZ() {
	return (float)gyr.Z;
}

float AutoSending::getAccX() {
	return (float)acc.X;
}
float AutoSending::getAccY() {
	return (float)acc.Y;
}
float AutoSending::getAccZ() {
	return (float)acc.Z;
}
volatile bool mpuInterrupt = false;
void dmpDataReady() {
	mpuInterrupt = true;
}
void AutoSending::setConnectionPort() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	yawX.attach(9);
	//pitchY.attach(9);


	// initialize serial communication
	
	// really up to you depending on your project)
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

							  
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		ifSetConnection = true;
	}
	else {
		// ERROR!
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
		ifSetConnection = false;
	}

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
}
void AutoSending::setRawData() {
	if (ifSetConnection == true) {
		mpu.getMotion6(&acc.X, &acc.Y, &acc.Z, &gyr.X, &gyr.Y, &gyr.Z);
		ifDownload = true;
	}
	else{
		Serial.print("Downloading data error!");
		Serial.println();
		ifDownload = false;
	}
}
void AutoSending::printRawData(){
	if (ifDownload == true && ifSetConnection == true) {
		Serial.print("Xgyr: ");
		Serial.print(gyr.X);
		Serial.print("  ");
		Serial.print("Ygyr: ");
		Serial.print(gyr.Y);
		Serial.print("  ");
		Serial.print("Zgyr: ");
		Serial.print(gyr.Z);
		Serial.print("  \n");

		Serial.print("Xacc: ");
		Serial.print(acc.X);
		Serial.print("  ");
		Serial.print("Yacc: ");
		Serial.print(acc.Y);
		Serial.print("  ");
		Serial.print("Zacc: ");
		Serial.print(acc.Z);
		Serial.print("  \n");
		delay(300);
	}
	else
	{
		Serial.print("Data is not declared!");
		Serial.println();
	}
}
void AutoSending::FIFOInitialize() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180 / M_PI); // YAW
		Serial.print("\t");
		Serial.print(ypr[1] * 180 / M_PI); // PITCH
		Serial.print("\t");
		Serial.println(ypr[2] * 180 / M_PI); // ROLL

		// set a servo
		yawX.write(map(ypr[0] * 180 / M_PI, -180, 180, 0, 180));
#endif 
	}

	
}
float AutoSending::getYaw() {
	return ypr[0];
}
float AutoSending::getPitch() {
	return ypr[1];
}
float AutoSending::getRoll() {
	return ypr[2];
}
void AutoSending::setServo(mode status) {  // it depends on the number of servos
	/*
	if (status == mode::STD) {
		yawX.write(map(ypr[0] * 180 / M_PI, -180, 180, 0, 180));  // yaw axis
		pitchY.write(map(ypr[1] * 180 / M_PI, -90, 90, 0, 180));  // pitch axis
	}
	else
	{
		Serial.print("Servo error!");
		Serial.println();
	}
	*/
}

AutoSending send;

