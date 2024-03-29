# Epson Sensing Units (IMU or Accelerometer) SPI Driver for Arduino

______________________________________________________________________

This is an example test library for the Epson M-Gxxx/M-A352
Sensing Units (SU) using the SPI interface.
It was specifically developed for use with the Arduino Teensy 3.6
development board (Teensyduino) and includes example applications that
can be used within the Arduino Sketch IDE.

For detailed information on the Epson Sensing Units, refer to the datasheet at
----> http://global.epson.com/products_and_drivers/sensing_system/

For updates to this software package, visit
----> http://vdc.epson.com

For further information on the Arduino, refer to their website at
----> http://www.arduino.cc

This software is released under the BSD license (see license.txt).
All text must be included in any redistribution.

# Hardware Considerations

______________________________________________________________________

This library assumes that the user has the following:

- Epson M-Gxxx or M-A352
- Arduino Teensy 3.6 (or compatible) development board
- Arduino Sketch IDE software v1.8.1 or greater
- This software package
- Micro USB cable (to power the Teensy and use the Serial Monitor)

The default configuration of the driver assumes that the Epson SU is connected to the SPI interface on the Teensy board.
Additionally, the RESET# and DataReady lines must be connected to available pins on the Arduino.
The following table shows the default pin mapping used by the M-Gxxx/M-A352 driver.

Circuit Pinmapping:

| Host Signal | Teensy 3.6 | Arduino DUE | Other 3.3V Arduino | M-Gseries | M-Vseries |
| ----------- | ---------- | ----------- | ------------------ | --------- | --------- |
| DRDY        | pin 6      | pin 6       | pin 6              | pin 13    | pin 14    |
| CSB         | pin 4      | pin 4       | pin 4              | pin 6     | pin 8     |
| MOSI        | pin 11     | SPI-4       | ICSP-4             | pin 5     | pin 2     |
| MISO        | pin 12     | SPI-1       | ICSP-1             | pin 2     | pin 6     |
| SCK         | pin 13     | SPI-3       | ICSP-3             | pin 1     | pin 4     |
| RST#        | pin 7      | pin-7       | pin 7              | pin 16    | pin 18    |

**NOTE**: For Teensy 3.6, the SPI interface may glitch during initialization.
The errant glitch can cause SPI communication error during flash
programming and reboot using "program" pushbutton on Teensy 3.6.
To recover, unplug and plug the USB cable between PC and Teensy 3.6

**CAUTION**: The Epson SU I/O interface is 3.3V CMOS.
Be sure to use only Arduino devices that are 3.3V I/O!

# Installation Instructions

______________________________________________________________________

To use the Epson SU Arduino driver and examples, the following steps are required.

1. Install Arduino Sketch IDE (if not already installed)
2. Install the Arduino board
3. Install Epson SU Arduino Library and example sketches
4. Use the Sketch IDE to compile example sketches and upload to the Arduino board

## 1. Install Arduino Sketch IDE

______________________________________________________________________

The Epson SU Arduino driver is designed to work with the Arduino Sketch IDE.
It was developed and tested using Sketch v1.8.1.
Sketch requires a platform running Windows, Mac OS X, or Linux.
If you do not have Sketch installed on your development platform, please visit the Arduino
website and download the version of Sketch compatible with your operating system.
Once Sketch is installed on your development platform, proceed to the next step.
For specific requirements and installation instructions, refer to the Arduino website at www.arduino.cc.

## 2. Install the Teensy 3.6 board

______________________________________________________________________

The default installation of Arduino Sketch may not include support for the Teensy 3.6.
To confirm whether Teensy support is installed click on “Tools->Board->Boards Manager...” on the Sketch menu.

Confirm whether the “Teensyduino-> Teensy 3.6" board package is installed. If the
package is not installed, install the latest Teensyduino software. Once the Teensyduino package is installed,
proceed to the next step.

NOTE: On Teensy 3.6, the SPI interface pins can glitch during re-programming and cause SPI communication error.
To recover unplug/plug USB cable.

## 3. Install the Arduino SAM boards

______________________________________________________________________

The default installation of Arduino Sketch may not include support for the Arduino Zero.
To confirm whether Zero support is installed click on “Tools->Board->Boards Manager...” on the Sketch menu.

Confirm whether the “Arduino SAM Boards (32-bits ARM Cortex-M3)” board package is installed. If the
package is not installed, install the latest version that matches your version of the Sketch IDE.
Once the SAM Boards package is installed, proceed to the next step

## 4. Install Epson SU Library and example sketches

______________________________________________________________________

The Epson SU Arduino driver is available as a .zip archive that includes the driver and a folder of example sketches.
The Sketch IDE can directly import the driver from a .zip file, so on the Sketch manu click on “Sketch->Include Library->Add .ZIP Library...”.
Then select the Epson SU driver package. This will install the driver and examples.
To confirm that the driver is installed, click on “Sketch->Include Library” and look for the “Epson SU...” entry
at the bottom of the list of available libraries.

## 5. Use the Sketch IDE to compile example sketches and upload to the board

______________________________________________________________________

Before compiling the example sketches, set the Board and Port settings in the Sketch IDE.
The Board and Port settings tell the Sketch IDE which Arduino product is being used and how to communicate with it.
To set the Board for Teensy 3.6, click "Tools->Board->Teensy 3.6".
To set the Port for Teensy 3.6, click "Tools->Port->COMx (Teensy)".

The port that the Teensy 3.6 is located on may differ according to the operating system of the development system.
For issues regarding USB port connections, please refer to the Arduino website at www.arduino.cc.

There are x examples sketches:

- su_library_test_example.ino is designed to test the library functions. It will test the functions, report, and then stop executing.
- su_sampling_example.ino is designed to demonstrate how to read the sensor. It will report the current sensor values in a loop.

To open the example sketches click on “File->Examples", find the "Epson SU SPI...", and then select one of the example sketches.
Once the example sketch is loaded, it can be compiled and uploaded to the Arduino.
Note that the Upload stage will fail if the Sketch “Board” and “Port” settings are not configured correctly.
If the upload to the Arduino completes successfully, the output from the example sketch can be viewed using
the Serial Monitor available in "Tools->Serial Monitor".

# Change Record:

| Date       | Ver  | Comment                                                                   |
| ---------- | ---- | ------------------------------------------------------------------------- |
| 2019-02-22 | v1.0 | - Initial release                                                         |
| 2021-03-30 | v1.1 | - Unite IMU and Accelerometers under common Sensing Units                 |
| 2023-01-17 | v1.2 | - Cleanup, refactor, deprecate old models, add new models G370S/G330/G366 |
| 2023-08-16 | v1.3 | - Added support G370PDG0, G370PDT0, cleanup, minor fixes                  |
