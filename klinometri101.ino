// NMEA 0183 ROLL PITH YAW for Genuino101
// Work derived from CurieIMU library and from Madgwick library visualizer, see
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
// Please get the above example working first, then continue with this sketch
// which is best debugged and tested with PuTTY and used on a USB serial line 
// petri38-github@yahoo.com
// GPL v3 - see LICENSE

#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long microsPerReading, microsNow, microsPrevious;
boolean debug;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float rollStart, pitchStart, headingStart;
boolean blinkState;


void setup() {
	char c; // read here from the serial line a command before the init
	int waitcmdsec = 5; // How long time to wait for a command

	pinMode(LED_BUILTIN, OUTPUT); // configure Arduino LED for activity indicator
	boolean blinkState = false; // initial state of the LED is no activity

	microsPerReading = 40; // microseconds between readings == 25Hz

	Serial.begin(4800); // initialize Serial communication
	while (!Serial);    // wait for the serial port to open - THERE MUST BE A DEVICE LISTENING!

	debug = false;
	while (waitcmdsec > 0) { // wait for an optional command from the serial port
		if (Serial.available())  {
			char c = Serial.read();  //gets one byte from serial buffer
			if ((c == 'd') || (c == 'D')) {// type a key D to switch on debug mode
				debug = true;
				break;
			} // Debug requested
		}
		delay(1000); // wait for 1s
		waitcmdsec--;
	} // while() wait for a key w/ timeout

	if (debug) Serial.println("Welcome to klinometri101 v0.901 running in debug mode - output is not usable with instruments");

	// initialize the IMU device
	if (debug) Serial.println("Initializing IMU device...");
	if (CurieIMU.begin()) {
		if (debug) Serial.println("CurieIMU initialized successfully");
	}
	else {
		Serial.println("CurieIMU initialization failed");
		while (1) {
			digitalWrite(LED_BUILTIN, LOW);
			delay(1000);                       // wait for 1 s
			digitalWrite(LED_BUILTIN, HIGH);
			delay(1000);                       // wait for a second
		} // not much reason to continue, try to alert the user by blinking a LED
	} // else a problem with the IMU unit, perhaps not Arduino/Genuino101?

	if (debug) {
		Serial.println("Internal sensor offsets BEFORE calibration...");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
		Serial.print("\t"); // -76
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
		Serial.print("\t"); // -235
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
		Serial.print("\t"); // 168
		Serial.print(CurieIMU.getGyroOffset(X_AXIS));
		Serial.print("\t"); // 0
		Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
		Serial.print("\t"); // 0
		Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
	} // debug

	if (debug) Serial.println("\nAbout to calibrate. Make sure your board is stable and upright");
	delay(5000);

	// The board must be resting in a horizontal position for
	// the following calibration procedure to work correctly!
	if (debug) Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
	CurieIMU.autoCalibrateGyroOffset();
	if (debug) Serial.println(" Done");

	if (debug) Serial.print("Starting Acceleration calibration and enabling offset compensation...");
	CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
	CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
	CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
	if (debug) Serial.println(" Done");

	if (debug) {
		Serial.println("Internal sensor offsets AFTER calibration...");
		Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
		Serial.print("\t"); // -76
		Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
		Serial.print("\t"); // -2359
		Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
		Serial.print("\t"); // 1688
		Serial.print(CurieIMU.getGyroOffset(X_AXIS));
		Serial.print("\t"); // 0
		Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
		Serial.print("\t"); // 0
		Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
		Serial.print("\nPress a key to continue...");
		waitcmdsec = 15;
		while (waitcmdsec > 0) { // wait for a key from the serial port w/ timeout
			if (Serial.available()) {
				char c = Serial.read();  //gets one byte from serial buffer
				break;
			} // character available in serial
			delay(1000); // wait for 1s
			waitcmdsec--;
		} // while()
	} // debug

	CurieIMU.setGyroRate(25); // 25Hz
	CurieIMU.setAccelerometerRate(25); // must be the same as Gyro here, see CurieIMU.h, 25Hz is the lowest common nominator
	filter.begin(25); // instantiate the Madgwick filter also to the same frequency domain

	// Set the accelerometer range to 2G
	CurieIMU.setAccelerometerRange(2);
	// Set the gyroscope range to 250 degrees/second
	CurieIMU.setGyroRange(250);

	microsPerReading = 1e6 / 25; // Let's try to keep the 25Hz here, albeit micros() is not a single micro tick but 4 or 8 micros min.
	microsPrevious = micros(); // No more actions in setup() - we deal with the eventual apprx. 70s overflow of the micros() count in the loop()
	
} // setup()

void loop() {

	int filtertraining = 100; // IMU algorithm updates requested before considered good for NMEA data (i.e. @25Hz 100 = 4s)

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

		filter.updateIMU(gx, gy, gz, ax, ay, az); // feed data into the Madgwick filter's IMU algorithm
		if (filtertraining > 0) filtertraining--;

		roll = filter.getRoll() ;
		pitch = filter.getPitch();
		heading = filter.getYaw();

		if (debug) {
			Serial.print("Orientation:");
			Serial.print(" Yaw: "); printFloat(heading,1);
			Serial.print(" Pitch: "); printFloat(pitch,1);
			Serial.print(" Roll: "); printFloat(roll,1);
			Serial.println("");
		} // debug

		microsPrevious = microsNow;
	} // if time to read the sensors to keep up within the frequency domain

	// check if time to send the NMEA sentences

} // loop()

float convertRawAcceleration(int aRaw) {
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767

	float a = (aRaw * 2.0) / 32768.0;
	return a;
}

float convertRawGyro(int gRaw) {
	// since we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = (gRaw * 250.0) / 32768.0;
	return g;
}

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
	}
}


