# klinometri101

## NMEA 0183 ROLL PITCH YAW Genuino101

[Hosted on GitHub](https://github.com/canne/klinometri101)

Work derived from CurieIMU library and from Madgwick library visualizer, get
started with your Arduino/Genuino101 for this purpose with:
https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser

Please get the above example working first, then continue with this sketch
which is best debugged and tested with PuTTY and used on a USB serial line 

petri38-github@yahoo.com
GPL v3 - see LICENSE

## Introduction
Program to use Genuino101 as inclination and ratio of turn sensor of a boat
with NMEA 0183 output. Gyroscope and accelerator sensors are sampled at 25 Hz.
Please note that the board has but 6-degree of freedom sensor, therefore it
cannot be used as a magnetic compass. Because most of navigation systems
provide even several heading values, the yaw value is not sent out to since
it is a relative value only. It can be observed in debug mode (see below).

## Ratio of Turn

The Ratio of Turn (ROT) sentences are sent out once per second from gyroscope
measured data. The sentence looks like this: `$KMROT,-1.5,A*hh` (hh=checksum),
negative values indicate bow turning to port side.

## Pitch and Roll

The pitch and roll values are sent out once every two seconds after a short
learning period. The unfortunately badly standarized XDR-sentence is sent
out in format known to be recognized by most of OpenCPN chart plotter
plugins: `$KMXDR,A,5.0,,PTCH,A,12.0,,ROLL,*hh` (hh=checksum),
negative values in pitch indicates bow down, in roll port side.

## Accuracy

The NMEA output is truncated to one significant decimal with a step of
0.5 degrees: 1.1, 1.2 becomes 1.0; 1.3, 1.4, 1.6, 1.7 becomes 1.5; 1.8 and
1.9 becomes 2.0. Tests have shown that even placing the sensor on marble no
better accuracy can be obtained.

## NMEA 0183 Serial interface on USB

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

## Calibration

Once the serial communication becomes available, the gyroscope and
accelerometer are calibrated unless valid calibration values are found
from the persistent memory. The startup of the board takes about five
seconds, and the calibration another five seconds if it is needed or
if it is explicitly requested. During the calibration, the board should
not move. Complete the calibration preferably while still moored or
when motoring on a calm sea...

### Forcing recalibration using serial line

The recalibration can be forced using a suitable serial line console
such as PuTTY.exe on the COM-port of the USB serial line. The chart
plotter or other programs should be disconnected from the USB serial
line at that time. If you hit key "C" within the five first seconds
after the serial line is connected, the calibration procedure is
forced to be executed and the results overwrite the values in the
persistent memory.

### Observation of the calibration and operation details

There is a possibility to observe the calibration procedure:
If you hit key "D" in the serial line console within the five seconds
before the calibration starts, the program switches into debug mode.
In debug mode, the calibration procedure and the raw data values fed
into the position algorithm are displayed along the NMEA data. To
leave the debug mode, one needs to reset the board.

### Forcing recalibration using a jumper, push button or a switch

There is a possibility to calibrate the board using a push button,
momentary switch or simply a jumper between GND pin and pin #7.
Internal pull-up resistor is used to pull up the pin when no switch.
The method is available both at the startup or any time during
the operation. At the end of the calibration procedure of the
accelerometer and gyroscope an on-board LED starts to blink once
per second if the jumper or switch is left on. After removing it
the values are stored in the permanent memory of the board and
fetched at next boot. Therefore, it is possible to correct
installation inaccuracies with single calibration.


## Selecting the general orientation of the installation

Another switch/jumper can be used to change the installation
orientation of the board. By default it is assumed that the
USB connector of Arduino 101 is pointing towards the bow of
the boat. By installing a jumper or a closed switch between
GND pin and pin #8, it is assumed that the USB connector is
pointing to the aft. The jumper or switch can be used any time.

## LED Diagnostic messages

The board's on-board LED nearest to the USB connector signals
the activity as follows:


* LED unlit
  - program is waiting for a listener on the USB serial line
* LED lit
 - USB serial line is available
 - waiting for commands (5s)
* LED blinking every second
  - calibration done please release the switch
* LED blinking every 2s
  - program cannot detect an IMU unit
* LED blinking very fast
  - the IMU unit is read at @25Hz
