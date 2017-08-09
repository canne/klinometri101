# klinometri101
NMEA 0183 ROLL PITCH YAW Genuino101

// Work derived from CurieIMU library and from Madgwick library visualizer, see

// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser

// Please get the above example working first, then continue with this sketch

// which is best debugged and tested with PuTTY and used on a USB serial line 

// petri38-github@yahoo.com

// GPL v3 - see LICENSE

Program to use Genuino101 as inclination sensor with NMEA 0183 output
Gyroscope and accelerator sensors are sampled at 25 Hz.
Please note that the board has no magnetic field related sensor,
therefore it cannot be used as a magnetic compass.

The NMEA output is truncated to one significant decimal with a step of
0.5 degrees. The dreaded NMEA XDR-sentence is used in output
like in this example: KMXDR,A,-13.5,D,ROLL*checksum

While the position algorithm is maintained at 25 Hz with the measured
sensor data the NMEA interface is limited to 4800 baud, with purpose
to maintain the compatibility with the standard NMEA 0183. If higher
update frequency is required, the source code allows quite easily to
increase the USB serial communication speed and the update frequency.

At startup of the board, the setup function waits, for ever if necessary
that the serial line communication becomes available. In other words,
some program needs to open the USB serial device before the program starts.
Usually, this program is a chart plotter, like OCPN with the excellent
tactics-plugin. For debugging purposes it can be a serial console, such
as PuTTY.

Once the serial communication becomes available, the gyroscope and
accelerometer are calibrated. The startup of the board takes about five
seconds, and the calibration another five seconds. During the calibration,
the board should not move. Admittedly, this is not very practical
when conditions are difficult. I can but suggest to complete the startup
while still moored or when motoring with sea not being too rough.

There is a possibility to observe the calibration procedure:
If you hit key "D" in the serial line console within the five seconds
before the calibration starts, the program switches into debug mode.
In debug mode, no NMEA data is sent but the calibration procedure and
the raw data values fed into the position algorithm are displayed. To
leave the debug mode, one needs to reset the board.

The board's activity LED signals the activity:

LED unlit - program is waiting for a listener on the USB serial line

LED blinking every second - program cannot detect an IMU unit

LED blinking very fast - the IMU unit is read




