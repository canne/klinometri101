// NMEA 0183 ROLL PITH YAW for Genuino101
// https://github.com/canne/klinometri101
// Work derived from CurieIMU library and from Madgwick library visualizer, see
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
// Please get the above example working first, then continue with this sketch
// which is best debugged and tested with PuTTY and used on a USB serial line 
// petri38-github@yahoo.com
// GPL v3 - see LICENSE

#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include <malloc.h>

// Button/switch connecting this pin to GND requests calibration
#define PIN_CALIBRATE_BUTTON 7
// Button or a momentary switch connecting this pin to GND makes current heading new YAW 0 in NMEA output
#define PIN_YAWZERO_BUTTON 8

Madgwick *filter = NULL;
unsigned long microsPerReading, microsNow, microsPrevious;
boolean debug;
boolean calibrate;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float rollStart, pitchStart, headingStart;
boolean blinkState;
int filtertraining;
int calibrationSwitchAntiBounce, calibrationSwitchAntiBounceCnt;
int yawZeroSwitchAntiBounce, yawZeroSwitchAntiBounceCnt;
float yawZero, yaw;
int eeAddress;

struct settings_t
{
	unsigned long start;
	float acc_offset_x;
	float acc_offset_y;
	float acc_offset_z;
	float gyr_offset_x;
	float gyr_offset_y;
	float gyr_offset_z;
	float yawZero;
	unsigned long end;
} settings;


void setup() {
	char c; // read here from the serial line a command before the init
	int waitcmdsec = 5; // How long time to wait for a command
	int i;
	boolean validsettings = false;

	eeAddress = 0; // Memory location where calibration settings are stored, maybe needs to be changed after a while?

	pinMode(LED_BUILTIN, OUTPUT); // configure Arduino LED for activity indicator
	pinMode(PIN_CALIBRATE_BUTTON, INPUT_PULLUP); // Input for button/switch for calibration
	blinkState = false; // initial state of the LED is: no activity
	digitalWrite(LED_BUILTIN, blinkState);	

	microsPerReading = 40; // microseconds between readings == 25Hz

	Serial.begin(4800); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open - THERE _MUST_ BE A DEVICE LISTENING!

	blinkState = !blinkState; // we've got a listener!
	digitalWrite(LED_BUILTIN, blinkState);	

	debug = false;
	while (waitcmdsec > 0) { // wait for an optional commands from the serial port right after serial start
		for (i = 0; i < 4; i++) {
			if (Serial.available())  {
				c = Serial.read();  //gets one byte from serial buffer
				if ((c == 'd') || (c == 'D')) debug = true; // type key D to switch on debug mode
				if ((c == 'c') || (c == 'C')) calibrate = true; // type key C to force calibration
			} // then character(s)
			if (debug && calibrate) break;
			delay(250);
		} // 250ms scan
		if (debug && calibrate) break;
		waitcmdsec--;
	} // while() wait for a key w/ timeout

	if (debug) Serial.println("Welcome to klinometri101 v0.904");
	if (debug) Serial.println("===============================");
	if (debug) Serial.println("Running in debug mode - output is not usable in navigation");

	// Check if the calibration request push button is pushed / switch is turned on
	boolean bounce = true;
	if (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
		blinkState = !blinkState; // we've got a calibration request by a button / switch ?
		digitalWrite(LED_BUILTIN, blinkState);
		delay(250);
		if (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
			delay(250);
			if (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
				delay(250);
				if (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
					bounce = false;
				} // then request to calibrate by button/switch?
			} // then request to calibrate by button/switch?
		} // then request to calibrate by button/switch?
		if (bounce) {
			if (debug) Serial.println("Detected 0 on the calibration request input pin - not consistent - ignoring.");
			blinkState = !blinkState; // we did not accept the calibration request input, back to Serial() only state
			digitalWrite(LED_BUILTIN, blinkState);
		} // then 0 detected but either bouncing or floating (pull-up not working?)
		else {
			if (debug) Serial.println("Detected 0 on the calibration request input pin - calibration will start soon.");
			calibrate = true;
		} // then 0 detected through a period of 1s four times, OK
	}// then request to calibrate by button/switch?

	// read settings from the EEPROM (actually Flash here)
	EEPROM.get(eeAddress, settings);
	if ((settings.start == 0xBBCCAADD) && (settings.end == 0xDDAACCBB)) {
		validsettings = true;
		if (debug) Serial.println("Stored settings found.");
	}
	else
		if (debug) Serial.println("Stored settings are garbage, ignoring.");

	if (!validsettings) {
		calibrate = true;
		if (debug) Serial.println("No valid settings found from persistent memory. Forcing calibration.");
	} // no valid settings
	else {
		if (calibrate) {
			if (debug) Serial.println("Valid settings found from persistent memory but calibration forced.");
			if (debug) Serial.println("These values will be overwritten:");
		} // valid data but calibration requested
		else {
			if (debug) Serial.println("Valid settings found from persistent memory:");
		} // else valid data and no calibration asked
		if (debug){
			Serial.print(settings.acc_offset_x); Serial.print("\t");			
			Serial.print(settings.acc_offset_y); Serial.print("\t");
			Serial.print(settings.acc_offset_z); Serial.print("\t");
			Serial.print(settings.gyr_offset_x); Serial.print("\t");
			Serial.print(settings.gyr_offset_y); Serial.print("\t");
			Serial.print(settings.gyr_offset_z); Serial.print("\t");
			Serial.print(settings.yawZero);      Serial.print("\t");
			Serial.println("");
		} // debug
	} // else valid settings

	// initialize the IMU device
	if (debug) Serial.println("Initializing IMU device...");
	if (CurieIMU.begin()) {
		if (debug) Serial.println("CurieIMU initialized successfully");
	}
	else {
		Serial.println("CurieIMU initialization failed");
		while (1) {
			digitalWrite(LED_BUILTIN, blinkState);
			delay(2000);                       // wait for 2 s
			blinkState = !blinkState;
			digitalWrite(LED_BUILTIN, blinkState);
			delay(2000);                       // wait for 2 s
			blinkState = !blinkState;
		} // not much reason to continue, try to alert the user by blinking a LED
	} // else a problem with the IMU unit, perhaps not Arduino/Genuino101?

	if (debug) {
		Serial.println("Internal sensor offsets at RESET...");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getGyroOffset(X_AXIS)); Serial.print("\t");
		Serial.print(CurieIMU.getGyroOffset(Y_AXIS)); Serial.print("\t");
		Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
	} // debug

	if (calibrate) {
		calibrateAccGyro();
	} // then calibration is necessary or requested
	else {
		if (debug) Serial.println("Restoring settings stored in persistent memory into the IMU device");
		CurieIMU.setAccelerometerOffset(X_AXIS, settings.acc_offset_x);
		CurieIMU.setAccelerometerOffset(Y_AXIS, settings.acc_offset_y);
		CurieIMU.setAccelerometerOffset(Z_AXIS, settings.acc_offset_z);
		CurieIMU.setGyroOffset(X_AXIS, settings.gyr_offset_x);
		CurieIMU.setGyroOffset(Y_AXIS, settings.gyr_offset_y);
		CurieIMU.setGyroOffset(Z_AXIS, settings.gyr_offset_z);
		yawZero = settings.yawZero;
		setupFrequencyDomain();
	} // else no calibration, use values from persistent settings

	if (digitalRead(PIN_CALIBRATE_BUTTON) == 0)
		if (debug) Serial.println("Please release the calibration request push button / switch");
	while (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		delay(1000);
	} // while the calibration push button / switch is on, çannot allow loop() to start

	if (debug) {
		Serial.print("setup() terminated - Press a key to continue or wait 15s...");
		waitcmdsec = 15;
		while (waitcmdsec > 0) { // wait for a key from the serial port w/ timeout
			if (Serial.available()) {
				(void)Serial.read();  //gets one byte from serial buffer
				break;
			} // character available in serial
			delay(1000); // wait for 1s
			waitcmdsec--;
		} // while()
		Serial.println("");
	} // debug

	// Prepare the counters for YAW zeroing button detection
	yawZeroSwitchAntiBounce = 25;
	yawZeroSwitchAntiBounceCnt = yawZeroSwitchAntiBounce;
	
} // setup()

void setupFrequencyDomain() {

	if (debug) Serial.println("setupFrequencyDomain(): Setting the frequency domain at 25Hz");
	CurieIMU.setGyroRate(25); // 25Hz
	CurieIMU.setAccelerometerRate(25); // must be the same as Gyro here, see CurieIMU.h, 25Hz is the lowest common nominator
	if (debug) Serial.print("Free RAM: ");
	if (debug) Serial.println(freeMemory());
	if (filter){
		delete (filter); // there is no destructor in MadgwickAHRS.h, this is supposed to release the memory allocated with "new"
		if (debug) Serial.print("Deleted Madgwick filter, free RAM now: ");
		if (debug) Serial.println(freeMemory());
	} // there was a Madgwick filter object, delete it
	filter = new Madgwick(); // this will make a new object, with fresh start (previous orientation will be lost)
	if (debug) Serial.print("Instantiated a new Madgwick filter, free RAM now: ");
	if (debug) Serial.println(freeMemory());
	filter->begin(25); // instantiate the Madgwick filter also to the same frequency domain

	if (debug) Serial.println("Setting the accelerometer range to 2G");
	CurieIMU.setAccelerometerRange(2);
	if (debug) Serial.println("Setting the gyroscope range to 250 degrees/second");
	CurieIMU.setGyroRange(250);

	microsPerReading = 1e6 / 25; // Let's try to keep the 25Hz here, albeit micros() is not a single micro tick but 4 or 8 micros min.
	microsPrevious = micros(); // No more actions in setup() - we deal with the eventual apprx. 70s overflow of the micros() count in the loop()
	filtertraining = 100; // IMU algorithm updates requested before considered good for NMEA data (i.e. @25Hz 100 = 4s)
	calibrationSwitchAntiBounce = 25; // (i.e. @25Hz 25 = 1s)
	calibrationSwitchAntiBounceCnt = calibrationSwitchAntiBounce;

} // setupFrequencyDomain()

void loop() {

	microsNow = micros();
	if ((unsigned long)(microsNow - microsPrevious) >= microsPerReading) { // cast subtraction to deal with overflow of micros()

		CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);

		// convert from raw data to gravity and degrees/second units
		ax = convertRawAcceleration(aix);
		ay = convertRawAcceleration(aiy);
		az = convertRawAcceleration(aiz);
		gx = convertRawGyro(gix);
		gy = convertRawGyro(giy);
		gz = convertRawGyro(giz);

		filter->updateIMU(gx, gy, gz, ax, ay, az); // feed data into the Madgwick filter's IMU algorithm
		if (filtertraining > 0) filtertraining--;

		roll = filter->getRoll() ;
		pitch = filter->getPitch();
		heading = filter->getYaw();

		if (debug) {
			Serial.print("Orientation:");
			Serial.print(" Yaw: "); printFloat(heading,1);
			Serial.print(" Pitch: "); printFloat(pitch,1);
			Serial.print(" Roll: "); printFloat(roll,1);
			Serial.println("");
		} // debug

		// Check if calibration request button / switch is on
		if (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
			if (debug) {
				Serial.print("Calibration request button / switch on - checking against bounce: (");
				Serial.print(calibrationSwitchAntiBounceCnt);
				Serial.println(")");
			}
			if (calibrationSwitchAntiBounceCnt <= 0) {
				if (debug) {
					Serial.println("");
					Serial.println("Calibration request button / switch on - calibration confirmed");
				}
				calibrate = true;
			} // then it is real, stop operation, reset the board which will lead to setup()
			else {
				calibrationSwitchAntiBounceCnt--;
			} // else not yet sure, wait for antibounce time to elapse
		} // perhaps there is a request to recalibrate - antibounce applied
		else {
			calibrationSwitchAntiBounceCnt = calibrationSwitchAntiBounce;
		} // else there is no calibration request button / switch on

		// Check if yaw reset button / temporary switch is on
		if (digitalRead(PIN_YAWZERO_BUTTON) == 0) {
			if (debug) {
				Serial.print("Yaw zeroing request button / switch on - checking against bounce: (");
				Serial.print(yawZeroSwitchAntiBounceCnt);
				Serial.println(")");
			}
			if (yawZeroSwitchAntiBounceCnt <= 0) {
				yawZero = heading;
				if (debug) {
					Serial.println("");
					Serial.print("Yaw zeroing request button / switch on - zeroing confirmed: ");
					Serial.println(yawZero);
				}
				yawZeroSwitchAntiBounceCnt = yawZeroSwitchAntiBounce;
			} // then it is real switch button push...
			else {
				yawZeroSwitchAntiBounceCnt--;
			} // else not yet sure, wait for antibounce time to elapse
		} // perhaps there is a request to zero to yaw - antibounce applied
		else {
			yawZeroSwitchAntiBounceCnt = yawZeroSwitchAntiBounce;
		} // else there is no yaw zeroing request button / switch on

		microsPrevious = microsNow;
	} // if time to read the sensors to keep up within the frequency domain

	// Check if calibration has been requested by button/switch
	if (calibrate)
		calibrateAccGyro();

	// check if time to send the NMEA sentences


} // loop()


void calibrateAccGyro() {

	if (debug) Serial.println("calibrateAccGyro()");
	if (debug) Serial.println("Restarting CurieIMU...");
	CurieIMU.end();
	delay(1000); // wait for 1s
	if (CurieIMU.begin()) {
		if (debug) Serial.println("CurieIMU initialized successfully");
	}
	else {
		Serial.println("CurieIMU initialization failed");
		while (1) {
			digitalWrite(LED_BUILTIN, blinkState);
			delay(2000);                       // wait for 2 s
			blinkState = !blinkState;
			digitalWrite(LED_BUILTIN, blinkState);
			delay(2000);                       // wait for 2 s
			blinkState = !blinkState;
		} // not much reason to continue, try to alert the user by blinking a LED
	} // else a problem with the IMU unit, perhaps not Arduino/Genuino101?

	if (debug) Serial.println("About to calibrate. Make sure your board is stable and upright");
	delay(5000); // wait for 5 s

	if (debug) Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
	CurieIMU.autoCalibrateGyroOffset();
	if (debug) Serial.println(" Done");

	if (debug) Serial.print("Starting Acceleration calibration and enabling offset compensation...");
	CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
	CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
	CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
	if (debug) Serial.println(" Done");

	settings.acc_offset_x = CurieIMU.getAccelerometerOffset(X_AXIS);
	settings.acc_offset_y = CurieIMU.getAccelerometerOffset(Y_AXIS);
	settings.acc_offset_z = CurieIMU.getAccelerometerOffset(Z_AXIS);
	settings.gyr_offset_x = CurieIMU.getGyroOffset(X_AXIS);
	settings.gyr_offset_y = CurieIMU.getGyroOffset(Y_AXIS);
	settings.gyr_offset_z = CurieIMU.getGyroOffset(Z_AXIS);
	yawZero = 180.0;
	settings.yawZero = yawZero;


	if (debug) {
		Serial.println("Internal sensor offsets AFTER calibration...");
		Serial.print(settings.acc_offset_x); Serial.print("\t");
		Serial.print(settings.acc_offset_y); Serial.print("\t");
		Serial.print(settings.acc_offset_z); Serial.print("\t");
		Serial.print(settings.gyr_offset_x); Serial.print("\t");
		Serial.print(settings.gyr_offset_y); Serial.print("\t");
		Serial.print(settings.gyr_offset_z); Serial.print("\t");
		Serial.print(settings.yawZero);      Serial.print("\t");
		Serial.println("");
		Serial.print("Press a key to continue or wait 15s...");
		int waitcmdsec = 15;
		while (waitcmdsec > 0) { // wait for a key from the serial port w/ timeout
			if (Serial.available()) {
				(void)Serial.read();  //gets one byte from serial buffer
				break;
			} // character available in serial
			delay(1000); // wait for 1s
			waitcmdsec--;
		} // while()
		Serial.println("");
	} // debug

	if (debug) Serial.println("Writing calibration values into the persistent memory...");
	settings.start = 0xBBCCAADD;
	settings.end = 0xDDAACCBB;
	EEPROM.put(eeAddress, settings);
	if (debug) Serial.println("calibrateAccGyro() - Done.");

	setupFrequencyDomain();

	// Let's avoid to calibrate in a loop
	calibrate = false;
	if (digitalRead(PIN_CALIBRATE_BUTTON) == 0)
		if (debug) Serial.println("Please release the calibration request push button / switch");
	while (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		delay(1000);
	} // while the calibration push button / switch is on, çannot allow loop() to start

} // calibrateAccGyro()

float convertRawAcceleration(int aRaw) {
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767

	float a = (aRaw * 2.0) / 32768.0;
	return a;
} // convertRawAcceleration()

float convertRawGyro(int gRaw) {
	// since we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = (gRaw * 250.0) / 32768.0;
	return g;
} // convertRawGyro()

void printFloat(float val, byte precision){
	// see http://forum.arduino.cc/index.php?topic=44216.msg320131#msg320131

	Serial.print(int(val));  //prints the int part
	if (precision > 0) {
		Serial.print("."); // print the decimal point
		unsigned long frac;
		unsigned long mult = 1;
		byte padding = precision - 1;
		while (precision--)
			mult *= 10;

		if (val >= 0)
			frac = (val - int(val)) * mult;
		else
			frac = (int(val) - val) * mult;
		unsigned long frac1 = frac;
		while (frac1 /= 10)
			padding--;
		while (padding--)
			Serial.print("0");
		Serial.print(frac, DEC);
	} // if decimal digits
} // printFloat()

// Memory functions for Arduino101 compiler chain courtesy to https://github.com/01org/corelibs-arduino101/pull/396/files
extern char __start_heap;
extern char __end_heap;
extern char __stack_size;
extern char __stack_start;
int freeStack() {
	int stack_end;
	int mark;
	stack_end = ((int)&__stack_start) - ((int)&__stack_size);
	return ((int)&mark) - stack_end;
} // freeStack()
int freeHeap(void) {
	int hsize;
	struct mallinfo mi;
	mi = mallinfo();
	hsize = (int)&__end_heap - (int)&__start_heap;
	return (hsize - mi.arena) + mi.fordblks;
} // freeHeap()
int freeMemory(void) {
	int heap = freeHeap();
	int stack = freeStack();
	return (stack < 0) ? heap : stack + heap;
} // freeMemory()
