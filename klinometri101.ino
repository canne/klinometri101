// NMEA 0183 ROLL PITH ROT (and yaw) for Arduino/Genuino101
// https://github.com/canne/klinometri101
// Work derived from CurieIMU library and from Madgwick library visualizer, see
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
// Please get the above example working first, then continue with this sketch
// which is best debugged and tested with PuTTY and used on a USB serial line 
// petri38-github@yahoo.com
// GPL v3 - see LICENSE

// Bluetooth Low Energy (BLE) support: comment out the directive below to disable BLE support
#define BLESERVICE 1

#include <CurieIMU.h>
#if defined(BLESERVICE)
#include <CurieBLE.h>
#endif
#include <MadgwickAHRS.h>
#include <EEPROM.h>
#include <MemoryFree.h>

// Button/switch connecting this pin to GND requests calibration
#define PIN_CALIBRATE_BUTTON 7
// Switch or jumper connecting this pin to GND indicater that Arduino101's USB connector is pointing to the back of the boat
#define PIN_USBPOINTS_BACK 8

#if defined(BLESERVICE)
// Create my own UUIDs; used https://www.uuidgenerator.net/
#define IMU_SERVICE_UUID "02522741-5a1c-46e6-842a-2a35f362f25c"
#define ROT_CHAR_UUID "06933f3b-7610-4ccf-85f5-102a0ef3effe"
#define PIT_CHAR_UUID "552a698e-97ae-4793-925c-bf50fbab9707"
#define ROL_CHAR_UUID "33daadb5-a48c-47f0-8438-496c915a2962"

BLEPeripheral blePeripheral; // Arduino/Genuino 101 acts as a BLE peripheral
BLEService nmeaBleService(IMU_SERVICE_UUID); // Service provided by this device, datapoints:
BLEFloatCharacteristic rotChar(ROT_CHAR_UUID, BLERead | BLENotify);
BLEFloatCharacteristic pitChar(PIT_CHAR_UUID, BLERead | BLENotify);
BLEFloatCharacteristic rolChar(ROL_CHAR_UUID, BLERead | BLENotify);
BLECentral centralBLE = blePeripheral.central(); // This is the central we are reporting to
#endif

char const *prefixNMEA = "KM"; // Feel free to change this if not suiting your needs
Madgwick *filter = NULL;
unsigned long microsPerReading, microsNow, microsPrevious;
unsigned long microsPerRollPitchNMEA, microsRollPitchNMEAPrevious;
unsigned long microsPerRotNMEA, microsPerRotNMEAPrevious;
#if defined(BLESERVICE)
unsigned long microsPerCentral, microsPerCentralPrevious;
#endif
boolean debug;
boolean calibrate;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading, rotation;
float rollStart, pitchStart, headingStart;
boolean blinkState;
int filtertraining;
int calibrationSwitchAntiBounce, calibrationSwitchAntiBounceCnt;
int backwardsSwitchAntiBounce, backwardsSwitchAntiBounceCnt;
boolean backwardsSwitch, backwards;
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
	boolean backwards;
	unsigned long end;
} settings;


void setup() {
	char c; // read here from the serial line a command before the init
	int waitcmdsec; // How long time to wait for a command or port to open
	int i;
	boolean validsettings = false;

	eeAddress = 0; // Memory location where calibration settings are stored, maybe needs to be changed after a while?

	pinMode(LED_BUILTIN, OUTPUT); // configure Arduino LED for activity indicator
	pinMode(PIN_CALIBRATE_BUTTON, INPUT_PULLUP); // Input for button/switch for calibration
	blinkState = false; // initial state of the LED is: no activity
	digitalWrite(LED_BUILTIN, blinkState);	

	microsPerReading = 40; // microseconds between readings == 25Hz

	Serial.begin(4800); // initialize Serial communication
#if defined(BLESERVICE)
	waitcmdsec = 10;
	while (waitcmdsec > 0) {
		if (!Serial)
			delay(1000); // wait 1s
		else
			break;
		waitcmdsec--;
	} // while waiting for serila line (USB) - maybe there is but Bluetooth BLE?
#else
	while (!Serial);    // wait for the serial port to open - THERE _MUST_ BE A DEVICE LISTENING!
#endif

	blinkState = !blinkState; // we've got a listener! (or not, if this is Bluetooth BLE-only
	digitalWrite(LED_BUILTIN, blinkState);	

	waitcmdsec = 5; // we maybe got a listener, wait this time for optional commands
#if defined(BLESERVICE)
	if (!Serial)
		waitcmdsec = 0; // No need to wait, nothing will come, anyway
#endif
	debug = false; calibrate = false;
	while (waitcmdsec > 0) { // wait for an optional commands from the serial port right after serial start
		for (i = 0; i < 4; i++) {
			if (Serial.available())  {
				c = Serial.read();  //gets one byte from serial buffer
				if ((c == 'd') || (c == 'D')) {
					debug = true; // type key D to switch on debug mode
					if (calibrate)
						Serial.print("CALIBRATE DEBUG");
					else
						Serial.print("DEBUG ");
				} // then debug command
				if ((c == 'c') || (c == 'C')) {
					calibrate = true; // type key C to force calibration
					if (debug) Serial.print("CALIBRATE");
				} // then force calibration command
			} // then character(s)
			if (debug && calibrate) break;
			delay(250);
		} // 250ms scan
		if (debug && calibrate) break;
		waitcmdsec--;
	} // while() wait for a key w/ timeout

	if (debug) {
		Serial.println(""); Serial.println("");
		Serial.println("Welcome to klinometri101 v1.1.0");
#if defined(BLESERVICE)
		Serial.println("Bluetooth Low Energy peripheral");
#endif
		Serial.println("===============================");
		Serial.println("Running in debug mode - output is not usable in navigation");
		Serial.println("");
	} // if debug

#if defined(BLESERVICE)
	if (debug) Serial.print("Initialing BLE peripheral... ");
	blePeripheral.setLocalName("KME"); // Initialize BLE peripheral
	blePeripheral.setAdvertisedServiceUuid(nmeaBleService.uuid());
	blePeripheral.addAttribute(nmeaBleService);
	blePeripheral.addAttribute(rotChar);
	blePeripheral.addAttribute(pitChar);
	blePeripheral.addAttribute(rolChar);
	if (debug) Serial.println("Done.");
#endif

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
		backwards = false;
		settings.backwards = false;
		if (debug) Serial.println("No valid settings found from persistent memory. Forcing calibration.");
	} // no valid settings
	else {
		backwards = settings.backwards;
		if (calibrate) {
			if (debug) Serial.println("Valid settings found from persistent memory but calibration forced.");
			if (debug) Serial.println("The settings will be overwritten:");
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
			if (settings.backwards)
				Serial.println("TRUE");
			else
				Serial.println("FALSE");
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
	} // else no calibration, use values from persistent settings

	setupFrequencyDomain();

	if (digitalRead(PIN_CALIBRATE_BUTTON) == 0)
		if (debug) Serial.println("Please release the calibration request push button / switch");
	while (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		delay(1000);
	} // while the calibration push button / switch is on, çannot allow loop() to start

#if defined(BLESERVICE)
	if (debug) Serial.print("activating Bluetooth device... ");
	blePeripheral.begin();
	if (debug) Serial.println("OK, waiting for connections.");
	if (debug) Serial.print("Checking for central... ");
	centralBLE = blePeripheral.central();
	if (debug) {
		if (centralBLE) {
			Serial.print("Connected to central: ");
			Serial.println(centralBLE.address());
		} // then central found
		else {
			Serial.println("No central found");
		} // else no central found
	} // debug
#endif

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
		Serial.println("setup() - Done.");
	} // debug

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

	// Prepare the counters for orientation turning request button detection
	backwardsSwitchAntiBounce = 25;
	backwardsSwitchAntiBounceCnt = backwardsSwitchAntiBounce;
	backwardsSwitch = false;

	microsPerRollPitchNMEA = 2e6; // every two seconds
	microsRollPitchNMEAPrevious = micros();
	microsPerRotNMEA = 1e6; // every second
	microsPerRotNMEAPrevious = micros();

#if defined(BLESERVICE)
	microsPerCentral = 3e6; // every three seconds
	microsPerCentralPrevious = micros();
#endif

} // setupFrequencyDomain()

void loop() {

	char nmeaSentence[50];
	microsNow = micros();
	if ((unsigned long)(microsNow - microsPrevious) >= microsPerReading) { // cast subtraction to deal with overflow of micros()

		if (debug) Serial.print("Read - ");
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
		rotation = -CurieIMU.readGyroScaled(Z_AXIS); // NEMA ROT sentence: negative is bow turning to port side

		if (debug) {
			Serial.print("Orientation:");
			Serial.print(" Yaw: "); printFloat(heading,1);
			Serial.print(" Pitch: "); printFloat(pitch,1);
			Serial.print(" Roll: "); printFloat(roll,1);
			Serial.print(" Rot: "); printFloat(rotation, 1);
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

		// Check if switch or jumper telling that USB connector is pointing backwards is on
		if (digitalRead(PIN_USBPOINTS_BACK) == 0) {
			if (debug) {
				Serial.print("Backwards installation switch on - checking against bounce: (");
				Serial.print(backwardsSwitchAntiBounceCnt);
				Serial.println(")");
			}
			if (backwardsSwitchAntiBounceCnt <= 0) {
				backwardsSwitch = true;
				if (debug) {
					Serial.println("");
					Serial.println("Backwards installation switch on - confirmed.");
				}
			} // then it is real switch or jumper set...
			else {
				backwardsSwitchAntiBounceCnt--;
			} // else not yet sure, wait for antibounce time to elapse
		} // perhaps a jumper / switch tells that USB connector is pointing backwards - antibounce applied
		else {
			backwardsSwitch = false;
			backwardsSwitchAntiBounceCnt = backwardsSwitchAntiBounce;
		} // else there is no switch/jumper on telling that USB connector is pointing backwards

		microsNow = micros();
		microsPrevious = microsNow;
	} // if time to read the sensors to keep up within the frequency domain

	// Check if calibration has been requested by button/switch
	if (calibrate)
		calibrateAccGyro();

	// Check if request to change the orientation to 'backwards'
	if (backwardsSwitch)
		turnBackwards();

#if defined(BLESERVICE)
	// check if it is time to check for a central BLE
	microsNow = micros();
	if ((unsigned long)(microsNow - microsPerCentralPrevious) >= microsPerCentral) {
		if (debug) Serial.print("centralBLE check... ");
		if (centralBLE) {
			if (debug) Serial.print("Connected to central: ");
			if (debug) Serial.println(centralBLE.address());
		} // then there is a valid central
		else {
			centralBLE = blePeripheral.central();
			if (debug) {
				if (centralBLE) {
					Serial.print("NEW! Connected to central: ");
					Serial.println(centralBLE.address());
				} // then central found
				else {
					Serial.println("No central found");
				} // else no central found
			} // debug
		}  // else there is no valid central
		microsNow = micros();
		microsPerCentralPrevious = microsNow;
	} // then time to check for BLE Central
#endif

	// check if time to send the NMEA sentence(s) for roll and pitch
	microsNow = micros();
	if ( ((unsigned long)(microsNow - microsRollPitchNMEAPrevious) >= microsPerRollPitchNMEA) &&
		(filtertraining <= 0) ) {
		// OpenCPN understanding of the dreaded XDR,
		// cf. http://www.cruisersforum.com/forums/f134/heel-and-pitch-180056.html#post2327822
		//
		// XDR - Transducer Measurement
		//
		//        1 2   3 4            n
		//        | |   | |            |
		// $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
		//
		// Field Number:
		//  1) Transducer Type
		//  2) Measurement Data
		//  3) Unit of Measurement, Celcius
		//  4) Name of transducer
		//  ...
		//  n) Checksum
		// There may be any number of quadruplets like this, each describing a sensor. The last field will be a checksum as usual.
		// Ex. 1) Type is "A" and
		// 4) Name is "PTCH" or
		// 4) Name is "ROLL" corresponding
		// 2) data (degrees) will be shown in the Dashboard instruments Pitch and/or Heel.
		// Positive Pitch is Nose up. Positive Roll is to Starboard.
		// e.g. $KMXDR,A,5.0,,PTCH,A,12.0,,ROLL,*hh
		// Will give you 5 degr pitch and 12 degr. heel.
		// It has 37 characters including CR/LF: 12bits/char(max),total12x37=444bits@4800baud takes 92.5ms < 100 ms
		//
		float rollOut = getDotZeroFive(roll);
		if (!backwards)
			rollOut = -rollOut;
		float pitchOut = getDotZeroFive(pitch);
		if (backwards)
			pitchOut = -pitchOut;
#define makeXDR() {sprintf(&nmeaSentence[0], "$%sXDR,A,%.1f,,PTCH,A,%.1f,,ROLL,*", prefixNMEA, pitchOut, rollOut); \
		byte chk = checksum(&nmeaSentence[0]); \
		int appendStrIdx = strlen(&nmeaSentence[0]); \
		sprintf(&nmeaSentence[appendStrIdx], "%02X", chk);}
#if defined(BLESERVICE)
		if (centralBLE) {
			rolChar.setValue(rollOut);
			pitChar.setValue(pitchOut);
		} // then Central BLE
		if (Serial) {
			makeXDR()
			Serial.println(nmeaSentence);
		} // then some reason to build a NMEA string and to send it out
#else
		makeXDR();
		Serial.println(nmeaSentence);
#endif
		microsNow = micros();
		microsRollPitchNMEAPrevious = microsNow;
	} // if is time to build and send the Roll and Pitch NMEA sentence if the filter is sufficiently trained

	// check if time to send the NMEA sentence(s) for yaw rate (i.e. rate of turn or ROT)
	microsNow = micros();
	if ((unsigned long)(microsNow - microsPerRotNMEAPrevious) >= microsPerRotNMEA) {
		// Rate of turn  https://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_ROT.html
		//
		// An example of the ROT string is :
		// $--ROT,x.x,A*hh
		// ROT message fields
		// Field 	Meaning
		//   0 	Message ID, ex. $GPROT
		//   1 	Rate of turn, degrees / minutes, “–” indicates bow turns to port
		//   2 	A: Valid data
		//      V: Invalid data
		//	 3 	The checksum data, always begins with *
		float rotationOut = getDotZeroFive(rotation);
#define makeROT() {sprintf(&nmeaSentence[0], "$%sROT,%.1f,A*", prefixNMEA, rotationOut); \
		byte chk = checksum(&nmeaSentence[0]); \
		int appendStrIdx = strlen(&nmeaSentence[0]); \
		sprintf(&nmeaSentence[appendStrIdx], "%02X", chk); \
		Serial.println(nmeaSentence);}
#if defined(BLESERVICE)
		if (centralBLE) {
			rotChar.setValue(rotationOut);
		} // then Central BLE
		if (Serial) {
			makeROT()
			Serial.println(nmeaSentence);
		} // then some reason to build a NMEA string and to send it out
#else
		makeROT();
		Serial.println(nmeaSentence);
#endif
		microsNow = micros();
		microsPerRotNMEAPrevious = microsNow;
	} // then send time out for rotation data, no need to wait for filter, reading from gyroscope, scaled


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

	if (debug) {
		Serial.println("Internal sensor offsets AFTER calibration...");
		printSettings();
		Serial.print("Press a key to continue or wait 15s (next: writing EEPROM)...");
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

	if (debug) Serial.println("Writing new, calibrated values into the persistent memory...");
	settings.start = 0xBBCCAADD;
	settings.end = 0xDDAACCBB;
	EEPROM.put(eeAddress, settings);
	if (debug) Serial.println("calibrateAccGyro() - Done.");

	// Let's avoid to calibrate in a loop
	calibrate = false;
	if (digitalRead(PIN_CALIBRATE_BUTTON) == 0)
		if (debug) Serial.println("Please release the calibration request push button / switch");
	while (digitalRead(PIN_CALIBRATE_BUTTON) == 0) {
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		delay(1000);
	} // while the calibration push button / switch is on, çannot allow loop() to continue

} // calibrateAccGyro()

void turnBackwards() {

	if (debug) Serial.println("turnBackwards()");
	if (settings.backwards)
		backwards = false; // flip-flop
	else
		backwards = true;
	settings.backwards = backwards;
	if (debug) {
		Serial.println("New settings to be stored...");
		printSettings();
		Serial.print("Press a key to continue or wait 15s (next: writing EEPROM)...");
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

	if (debug) Serial.println("Writing new values into the persistent memory...");
	settings.start = 0xBBCCAADD;
	settings.end = 0xDDAACCBB;
	EEPROM.put(eeAddress, settings);
	if (debug) Serial.println("turnBackwards() - Done.");

	// Let's avoid to change the orientation value in a loop
	backwardsSwitch = false;
	backwardsSwitchAntiBounceCnt = backwardsSwitchAntiBounce;
	if (digitalRead(PIN_USBPOINTS_BACK) == 0)
		if (debug) Serial.println("Please release the orientation change request switch/button/jumper...");
	while (digitalRead(PIN_USBPOINTS_BACK) == 0) {
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		delay(1000);
	} // while the orientation change request switch/button/jumper is on, çannot allow loop() to continue

} // turnBackwards()

void printSettings(void) {
	Serial.print(settings.acc_offset_x); Serial.print("\t");
	Serial.print(settings.acc_offset_y); Serial.print("\t");
	Serial.print(settings.acc_offset_z); Serial.print("\t");
	Serial.print(settings.gyr_offset_x); Serial.print("\t");
	Serial.print(settings.gyr_offset_y); Serial.print("\t");
	Serial.print(settings.gyr_offset_z); Serial.print("\t");
	if (settings.backwards)
		Serial.println("TRUE");
	else
		Serial.println("FALSE");
} //printSettings()

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
	// courtesy http://forum.arduino.cc/index.php?topic=44216.msg320131#msg320131

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

float getDotZeroFive(float val) {
	unsigned long frac;
	float fracZeroFive;
	int wholeNum = (int)val;
	if (val >= 0)
		frac = (unsigned long)((val - (float)wholeNum) * 10.0);
	else
		frac = (unsigned long)((float(wholeNum) - val) * 10.0);
	if (frac <= 2) {
		fracZeroFive = 0.0;
	} // then zero
	else {
		if (frac <= 7) {
			fracZeroFive = 0.5;
		} // then five
		else {
			fracZeroFive = 1.0;
		} // else zero but round up
	} // else not zero
	return ((float)wholeNum + fracZeroFive);
} // getDotZeroFive()

byte checksum(char* str)
{
	byte cs = 0;
	for (unsigned int n = 1; n < strlen(str) - 1; n++)
	{
		cs ^= str[n];
	}
	return cs;
} // checksum()