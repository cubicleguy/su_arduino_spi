# Table of Contents

______________________________________________________________________

<!---toc start-->

- [Table of Contents](#table-of-contents)
- [Epson Sensing Unit SPI Driver for Arduino](#epson-sensing-unit-spi-driver-for-arduino)
- [Hardware Considerations](#hardware-considerations)
- [Installation Instructions](#installation-instructions)
  - [1. Install Arduino IDE](#1-install-arduino-ide)
  - [2. Install the Teensy 3.6 board](#2-install-the-teensy-36-board)
  - [3. Install the Arduino SAM boards](#3-install-the-arduino-sam-boards)
  - [4. Install Epson SU Library with example sketches](#4-install-epson-su-library-with-example-sketches)
  - [5. Use the IDE to compile example sketches and upload to the board](#5-use-the-ide-to-compile-example-sketches-and-upload-to-the-board)
- [Example Serial Console Output](#example-serial-console-output)
  - [Accl Example](#accl-example)
  - [IMU Example](#imu-example)
  - [Vibe Example](#vibe-example)
- [Change Record:](#change-record)

<!---toc end-->

# Epson Sensing Unit SPI Driver for Arduino

______________________________________________________________________

This is an example test library for the Epson M-Gxxx/M-Axxx
Sensing Units (SU) using the SPI interface.
It is developed on the Arduino Zero or Teensy 3.6
development board (Teensyduino) and includes example applications that
can be used within the Arduino IDE.
The library requires one SPI port and one UART port on the Arduino (one UART for serial
console output, and one SPI for connection to Epson device).

For detailed information on the Epson Sensing Units, refer to the datasheet at
----> https://www.epsondevice.com/sensing/en/

For further information on the Arduino, refer to their website at
----> http://www.arduino.cc

This software is released under the BSD license (see license.txt).
All text must be included in any redistribution.

# Hardware Considerations

______________________________________________________________________

This library assumes that the user has the following:

- Epson M-Gxxx or M-Axxx sensor device
- Arduino Teensy 3.6 or Arduino Zero (or compatible) development board
- Arduino IDE software v2.3.6 or greater
- This software package
- Micro USB cable to power/connect to development board and use Serial Monitor

The default configuration of the driver assumes that:

- Epson device is connected to `SPI`
- serial console output is connected to `Serial`

Additionally, the Epson device `RESET#` and `DataReady (optional)` is connected to available
pins on the Arduino.

The following table shows the default pin mapping used by the M-Gxxx/M-A352 driver.

Circuit Pinmapping:

| Signal on Host | Teensy 3.6 | Arduino Zero/DUE | M-Gseries |
| -------------- | ---------- | ---------------- | --------- |
| DRDY           | pin 6      | pin 6            | pin 13    |
| CSB            | pin 4      | pin 4            | pin 6     |
| MOSI           | pin 11     | SPI-4            | pin 5     |
| MISO           | pin 12     | SPI-1            | pin 2     |
| SCK            | pin 13     | SPI-3            | pin 1     |
| RST#           | pin 7      | pin-7            | pin 16    |

**CAUTION**: The Epson device I/O interface is 3.3V CMOS.
Be sure to use only Arduino devices that are 3.3V I/O!

# Installation Instructions

______________________________________________________________________

To use the Epson SU Arduino driver and examples, the following steps are required.

1. Install Arduino IDE (if not already installed)
2. Install the Arduino board
3. Install Epson SU Arduino Library and example sketches
4. Use the IDE to compile example sketches and upload to the Arduino board

## 1. Install Arduino IDE

______________________________________________________________________

The Epson SU Arduino driver is designed to work with the Arduino IDE.
The IDE requires a platform running Windows, Mac OS X, or Linux.
If you do not have the IDE installed on your development platform, please visit the Arduino
website and download the version of the IDE compatible with your operating system.
Once the IDE is installed on your development platform, proceed to the next step.
For specific requirements and installation instructions, refer to the Arduino website at www.arduino.cc.

## 2. Install the Teensy 3.6 board

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Teensy 3.6.
To confirm whether Teensy support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Serach for "Teensy" and confirm whether the `Teensy (for Arduino IDE 2.0.4 or later)` board package is installed. If the
package is not installed, install it the latest version. Once the Teensy package is installed,
proceed to the next step.

## 3. Install the Arduino SAM boards

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Arduino Zero.
To confirm whether Zero support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Confirm whether the `Arduino SAM Boards (32-bits ARM Cortex-M0+)` board package is installed. If the
package is not installed, install the latest version that matches your version of the IDE.
Once the SAM Boards package is installed, proceed to the next step

## 4. Install Epson SU Library with example sketches

______________________________________________________________________

The Epson SU driver for Arduino is available for install within the Arduino IDE if connected to the internet.
In the Arduino IDE click on `Tools->Manage Libraries->Library Manager search for Epson_SU_SPI...`.
Then select the Epson SU driver package to install the driver and examples.

The Epson SU Arduino driver is available as a .zip archive from https://github.com/cubicleguy/su_arduino_spi/releases.
The IDE can directly import the driver from a .zip file, so on the IDE menu click on `Sketch->Include Library->Add .ZIP Library...`.
Then select the Epson SU driver ZIP package. This will install the driver and examples.

## 5. Use the IDE to compile example sketches and upload to the board

______________________________________________________________________

Before compiling the example sketches, set the Board and Port settings in the IDE.
The Board and Port settings tell the IDE which Arduino product is being used and how to communicate with it.
To set the Board, click `Tools->Board->select the Teensy 3.6, Arduino Zero, or other compatible board`.
To set the Port, click `Tools->Port->select the proper serial port`.

The port that your Arduino board is located on may differ according to the operating system on the development system.
For issues regarding USB port connections, please refer to the Arduino website at http://www.arduino.cc.

The following are examples sketches included in the library:

- su_epson_accl_sampling.ino is designed to demonstrate initializing and reading accelerometer data.
- su_epson_imu_sampling.ino is similar but for the IMU sensor.
- su_epson_vibe_sampling.ino is similar but for the vibration sensor.

To open the example sketches click on `File->Examples`, find the `Epson SU SPI...`, and then select one of the example sketches.
Once the example sketch is loaded, it can be compiled and uploaded to the Arduino.
**NOTE:** The Upload stage will fail if the IDE `Board` and `Port` settings are not configured correctly.

If the upload to the Arduino completes successfully, the output from the example sketch can be viewed using
the Serial Monitor available in `Tools->Serial Monitor`.
**NOTE:** You may have to set the serial baudrate on the Serial Monitor to match the baudrate setting in the sketch.

# Example Serial Console Output

______________________________________________________________________

## Accl Example

```
Platform: Teensy 3.6
Open SPI Port for Epson device: nCS on 4 @ 1000000 Hz
nRST on 7
HW Reset asserted
Asserting HW Reset
DRDY on pin 6
Check DRDY
Checking device is present...device responded to ID read
Detected PROD_ID:A352AD10

Sample#, Accl X, Accl Y, Accl Z, TempC, Count, ND_EA, Checksum
0, -0.06348842, -0.02866757, 1.00212002, 24.943, 00040, 8e00, 36046
1, -0.06348693, -0.02867037, 1.00221539, 24.943, 00080, 8e00, 37664
2, -0.06348908, -0.02866864, 1.00229537, 24.943, 00120, 8e00, 39040
3, -0.06349915, -0.02866590, 1.00229788, 24.943, 00160, 8e00, 38998
4, -0.06351471, -0.02866644, 1.00222373, 24.943, 00200, 8e00, 37525
5, -0.06352711, -0.02867121, 1.00213218, 24.943, 00240, 8e00, 35742
6, -0.06352901, -0.02867746, 1.00209105, 24.943, 00280, 8e00, 34954
7, -0.06351882, -0.02867746, 1.00212646, 24.943, 00320, 8e00, 35759
8, -0.06350094, -0.02866453, 1.00220299, 24.943, 00360, 8e00, 37601
9, -0.06348157, -0.02864075, 1.00225258, 24.943, 00400, 8e00, 39195
10, -0.06346488, -0.02861685, 1.00222874, 24.943, 00440, 8e00, 39517
...
491, -0.06351680, -0.02863538, 1.00207305, 24.946, 19680, 8e00, 54962
492, -0.06350660, -0.02864295, 1.00202107, 24.946, 19720, 8e00, 54173
493, -0.06349522, -0.02864605, 1.00209832, 24.946, 19760, 8e00, 55650
494, -0.06348842, -0.02863610, 1.00224733, 24.946, 19800, 8e00, 58469
495, -0.06349003, -0.02861458, 1.00234604, 24.946, 19840, 8e00, 60500
496, -0.06349891, -0.02859330, 1.00230825, 24.946, 19880, 8e00, 60114
497, -0.06351066, -0.02858490, 1.00215948, 24.946, 19920, 8e00, 57602
498, -0.06352121, -0.02859223, 1.00201881, 24.950, 19960, 8e00, 54981
499, -0.06352830, -0.02860677, 1.00199842, 24.950, 20000, 8e00, 54317

*****************************************************************
PROD_ID: A352AD10	SERIAL_ID: E0000086	VERSION: 12
DOUT_RATE: 100.000	FILTER_SEL: KAISER512FC9
ND_EA: ON	TempC: ON
X: AccX	Y: AccY	Z: AccZ
Count: ON	Chksm: ON

*****************************************************************
Done
```

## IMU Example

```
Platform: Teensy 3.6
Open SPI Port for Epson device: nCS on 4 @ 1000000 Hz
nRST on 7
HW Reset asserted
Asserting HW Reset
DRDY on pin 6
Check DRDY
Checking device is present...device responded to ID read
Detected PROD_ID:G355QDG0
Warning: Device does not support attitude or quaternion output
Warning: Device does not support delta output

Sample#	Gx	Gy	Gz	Ax	Ay	Az	TempC	Counter	Flags
0, 0.221993, 0.023791, -0.019222, 30.947449, 2.106979, 252.888412, 24.196, 00008, fe00
1, 0.180059, 0.038334, -0.012205, 69.952187, 0.894554, 499.704376, 23.384, 00016, fe00
2, 0.135385, 0.041800, -0.003006, 109.063774, -0.279167, 746.379456, 22.573, 00024, fe00
3, 0.086797, 0.053506, -0.008386, 148.169464, -1.592720, 993.475708, 21.761, 00032, fe00
4, 0.087227, 0.056623, -0.015156, 148.682465, -1.439888, 993.495117, 21.754, 00040, fe00
5, 0.077471, 0.051895, -0.017759, 148.746841, -1.613403, 993.903442, 21.766, 00048, fe00
6, 0.069781, 0.064434, -0.010113, 148.574051, -1.840134, 994.294800, 21.777, 00056, fe00
7, 0.066439, 0.051766, 0.000422, 148.374802, -1.795654, 994.291565, 21.789, 00064, fe00
8, 0.063788, 0.045316, 0.008797, 148.172760, -1.893852, 994.558594, 21.801, 00072, fe00
9, 0.059684, 0.058967, 0.010780, 148.363739, -1.650070, 994.627563, 21.802, 00080, fe00
10, 0.057188, 0.059772, 0.004546, 148.674469, -1.480873, 994.959473, 21.806, 00088, fe00
...
491, 0.048596, 0.065235, -0.001613, 148.338959, -1.229668, 994.598389, 22.121, 03936, fe00
492, 0.056763, 0.061815, -0.006560, 148.579132, -1.439896, 995.055725, 22.123, 03944, fe00
493, 0.074203, 0.045676, -0.009294, 148.323059, -1.582047, 995.400635, 22.125, 03952, fe00
494, 0.086822, 0.033184, -0.009831, 148.265686, -1.638115, 994.992859, 22.124, 03960, fe00
495, 0.095341, 0.041091, 0.001526, 148.451340, -1.928848, 994.750732, 22.120, 03968, fe00
496, 0.095320, 0.051348, -0.001538, 148.164642, -1.634010, 994.377258, 22.116, 03976, fe00
497, 0.095033, 0.058544, 0.001985, 148.124512, -1.569275, 993.792114, 22.112, 03984, fe00
498, 0.085120, 0.065553, 0.006217, 147.902588, -1.576401, 993.798218, 22.109, 03992, fe00
499, 0.090150, 0.068667, 0.008590, 147.742126, -1.515244, 993.788879, 22.108, 04000, fe00

*****************************************************************
PROD_ID: G355QDG0	SERIAL_ID: T0000004	VERSION: 4610
DOUT_RATE: 250.000	FILTER_SEL: MVAVG_TAP32
ND_EA: ON	TempC: 32	Gyro: 32	Accl: 32
GPIO: OFF	Count: ON	Chksm: OFF
*****************************************************************
Done
```

## Vibe Example

```
Platform: Teensy 3.6
Open SPI Port for Epson device: nCS on 4 @ 1000000 Hz
nRST on 7
HW Reset asserted
Asserting HW Reset
DRDY on pin 6
Check DRDY
Checking device is present...device responded to ID read
Detected PROD_ID:A342VD10
ERROR: Unsupported product model.
Error. Unsupported device detected. Halting...
Platform: Teensy 3.6
Open SPI Port for Epson device: nCS on 4 @ 1000000 Hz
nRST on 7
HW Reset asserted
Asserting HW Reset
DRDY on pin 6
Check DRDY
Checking device is present...device responded to ID read
Detected PROD_ID:A342VD10
Sample#, DispX RAW, DispY RAW, DispZ RAW, TempC, errEXI, errALARM, 2bit, Count
0, -0.000015020, -0.000000477, 0.000041485, 24.309, 0, 0, 1, 00002
1, -0.000015020, -0.000000715, 0.000040054, 24.309, 0, 0, 2, 00004
2, -0.000014544, -0.000000715, 0.000038385, 24.309, 0, 0, 3, 00006
3, -0.000014305, -0.000000954, 0.000037193, 24.309, 0, 0, 0, 00008
4, -0.000014067, -0.000000954, 0.000036001, 24.309, 0, 0, 1, 00010
5, -0.000013828, -0.000000954, 0.000035763, 24.309, 0, 0, 2, 00012
6, -0.000013590, -0.000001192, 0.000035763, 24.309, 0, 0, 3, 00014
7, -0.000013590, -0.000001192, 0.000036478, 24.309, 0, 0, 0, 00016
8, -0.000013590, -0.000001431, 0.000037432, 24.309, 0, 0, 1, 00018
9, -0.000013828, -0.000001192, 0.000039101, 24.309, 0, 0, 2, 00020
10, -0.000014067, -0.000000954, 0.000040770, 24.309, 0, 0, 3, 00022
...
491, 0.000003815, -0.000013590, 0.000036478, 24.309, 0, 0, 0, 00984
492, 0.000003815, -0.000013828, 0.000037193, 24.309, 0, 0, 1, 00986
493, 0.000003576, -0.000013828, 0.000038624, 24.309, 0, 0, 2, 00988
494, 0.000003576, -0.000014067, 0.000040531, 24.309, 0, 0, 3, 00990
495, 0.000003576, -0.000014067, 0.000042439, 24.309, 0, 0, 0, 00992
496, 0.000003099, -0.000014067, 0.000044346, 24.309, 0, 0, 1, 00994
497, 0.000003099, -0.000014305, 0.000046253, 24.309, 0, 0, 2, 00996
498, 0.000002861, -0.000014067, 0.000047684, 24.309, 0, 0, 3, 00998
499, 0.000002861, -0.000014067, 0.000048399, 24.309, 0, 0, 0, 01000

*****************************************************************
PROD_ID: A342VD10	SERIAL_ID: 00000094	VERSION: 280
DOUT_RATE: 300.0000	UPDATE_RATE: 300.0000
ND_EA: OFF		TempC: ON
X: Disp RAW		Y: Disp RAW		Z: Disp RAW
Count: ON		Chksm: OFF

*****************************************************************
Done
```

# Change Record:

| Date       | Ver    | Comment                                                                                     |
| ---------- | ------ | ------------------------------------------------------------------------------------------- |
| 2019-02-22 | v1.0   | - Initial release                                                                           |
| 2021-03-30 | v1.1   | - Unite IMU and Accelerometers under common Sensing Units                                   |
| 2023-01-17 | v1.2   | - Cleanup, refactor, deprecate old models, add new models G370S/G330/G366                   |
| 2023-08-16 | v1.3   | - Added support G370PDG0, G370PDT0, cleanup, minor fixes                                    |
| 2024-06-17 | v1.4   | - Remove column for "Other Arduinos" in Circuit Pinmapping because it varies based on board |
|            |        | - Remove support for V340                                                                   |
| 2025-11-27 | v2.0   | - Major revamp the driver, add support for M-G355QDG0, M-G570PR20, M-A370AD10, M-A342VD10   |
| 2026-04-11 | v2.0.1 | - Minor patch for Arduino Zero or similar not printing floating point values                |
