// NMEA 0183 ROLL PITH YAW for Genuino101
// Work derived from CurieIMU visualizer, see
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
// Please get the above example working first, then continue with this sketch
// petri38-github@yahoo.com

#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Madgwick filter;
unsigned long millisPerReading, millisNow, millisPrevious;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
float rollStart, pitchStart, headingStart;


void setup() {
  int avgcnt = 100; // "n" for averaging
  float n = (float) avgcnt;
  millisPerReading = 40; // milliseconds between readings == 25Hz
  
  Serial.begin(9600);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25); // 25Hz
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  millisPrevious = millis(); // good for 50 days w/o reset
  roll = 0.0;
  pitch = 0.0;
  heading = 0.0;

  while ( avgcnt > 0 ) {
    millisNow = millis();
    if (millisNow - millisPrevious >= millisPerReading) {
      UpdateIMU ();
      roll = roll + filter.getRoll();
      pitch = pitch + filter.getPitch();
      heading = heading + filter.getYaw();
      avgcnt--;
      millisPrevious = millisPrevious + millisPerReading;
    } // if (millis)
  } // while()

  rollStart = roll / n;
  pitchStart = pitch / n;
  headingStart = heading / n;
  millisPrevious = millis();
  
} // setup()

void loop() {

  // check if it's time to read data and update the filter
  millisNow = millis();
  if (millisNow - millisPrevious >= millisPerReading) {
    UpdateIMU();

    // print the heading, pitch and roll
    roll = filter.getRoll() - rollStart;
    pitch = filter.getPitch() - pitchStart;
    heading = filter.getYaw() - headingStart;
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    millisPrevious = millisPrevious + millisPerReading;
  } // if (millis)
}

void UpdateIMU () {
  
    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);
  
} // UpdateIMU

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
