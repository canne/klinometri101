# klinometri101
NMEA 0183 ROLL PITCH YAW Genuino101

// https://github.com/canne/klinometri101

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
as PuTTY.exe on Windows or minicom on Linux.

Once the serial communication becomes available, the gyroscope and
accelerometer are calibrated unless valid calibration values are found
from the persistent memory. The startup of the board takes about five
seconds, and the calibration another five seconds. During the calibration,
the board should not move. Complete the calibration preferably
while still moored or when motoring with sea not being too rough.

The recalibration can be forced using a suitable serial line console
such as PuTTY.exe on the COM-port of the USB serial line. The chart
plotter or other programs should be disconnected from the USB serial
line at that time. If you hit key "C" within the five first seconds
after the serial line is connected, the calibration procedure is
forced to be executed and the result overwritest the values in the
persistent memory.

There is a possibility to observe the calibration procedure:
If you hit key "D" in the serial line console within the five seconds
before the calibration starts, the program switches into debug mode.
In debug mode, the calibration procedure and the raw data values fed
into the position algorithm are displayed along the NMEA data. To
leave the debug mode, one needs to reset the board.

The board's activity LED signals the activity:

LED unlit - program is waiting for a listener on the USB serial line

LED lit - USB serial line is available - waiting for commands (5s)

LED blinking every second - calibration done plese release the switch

LED blinking every 2s - program cannot detect an IMU unit

LED blinking very fast - the IMU unit is read




