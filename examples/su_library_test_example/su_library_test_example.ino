/***********************************************************************
  su_library_test_example.ino

  This Arduino sketch example tests some of the library functions.
  This is tested for support on Teensy 3.6 or Arduino DUE.
  However, this software is expected to work on any 3.3V Arduino platform.

  For more information on the Epson Sensing Inits (SU) visit:

  --> http://global.epson.com/products_and_drivers/sensing_system/

  This test application assumes that the SU SPI signals are connected
  to the Teensy 3.6 header (hardware SPI). The pins are mapped as shown below.

  Circuit Pinmapping:
  Signal Teensy 3.6 Arduino DUE    Other 3.3V Arduino   M-Gseries M-Vseries
  --------------------------------------------------------------------------
  DRDY   pin 6      pin 6          pin 6                pin 13    pin 14
  CSB    pin 4      pin 4          pin 4                pin 6     pin 8
  MOSI   pin 11     SPI-4          ICSP-4               pin 5     pin 2
  MISO   pin 12     SPI-1          ICSP-1               pin 2     pin 6
  SCK    pin 13     SPI-3          ICSP-3               pin 1     pin 4
  RST#   pin 7      pin-7          pin 7                pin 16    pin 18

  NOTE: For Teensy 3.6, the SPI interface glitches during initialization.
        The errant glitch can cause SPI communication error during flash
        programming and reboot when using "program" pushbutton on Teensy 3.6.
        To recover, unplug and plug the USB cable between PC and Teensy 3.6

************************************************************************

  @section LICENSE

  Software License Agreement (BSD License, see license.txt)

  Copyright (c) 2019-2023 Seiko Epson Corporation.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***********************************************************************/

// 1. Set the device model type in the epson_devices.h file
// 2. Set the user specific settings in the xxxx_epson_user_def.h file
// Epson SPI Device Arduino Library Definitions
#include <spi_epson_common.h>

// Chipselect,DataReady, Reset, and EXT must be assigned as GPIO
// as per pin assignment listed in the header description above.
const uint8_t EPSON_CS = 4;     // Arduino pin 4
const uint8_t EPSON_DRDY = 6;   // Arduino pin 6
const uint8_t EPSON_RESET = 7;  // Arduino pin 7

// When using HW SPI interface, instantiate class with CS, DRDY, RESET pins
EPSON_DEV su(EPSON_CS, EPSON_DRDY, EPSON_RESET);  // Hardware SPI

// SCK, MOSI, MISO are assigned to GPIO when using "software" emulated SPI
// const uint8_t EPSON_SCK  = 13;  //Arduino pin 13
// const uint8_t EPSON_MOSI = 11;  //Arduino pin 11
// const uint8_t EPSON_MISO = 12;  //Arduino pin 12

// When using SW emulated SPI interface, instantiate class with SCK, MISO, MOSI,
// CS, DRDY, RESET pins
// EPSON_DEV su(EPSON_SCK, EPSON_MISO, EPSON_MOSI, EPSON_CS, EPSON_DRDY,
// EPSON_RESET);  // Software SPI

/*------------------------------------------------------------------------------
 * setup()
 *
 *------------------------------------------------------------------------------*/
void setup() {
  SerialConsole.begin(250000);  // Setup Serial communications for 250000bps
  while (!SerialConsole) {
    ;  // Wait for serial port to connect. Needed for native USB port only
  }

  if (!su.begin()) {
    SerialConsole.println(
        "Error. Can not communicate properly with " EPSON_UNIT_TYPE);
    while (1)
      ;
  }

  // Toggle SU Reset Line & wait for reset delay time
  su.sensorHWReset();

  // Initialize SU with settings
  if (!su.sensorInit(CMD_RATEX, CMD_FILTERX)) {
    SerialConsole.println("Error. Can not initialize " EPSON_UNIT_TYPE);
    while (1)
      ;
  }
  SerialConsole.println(EPSON_UNIT_TYPE " Init Done");
}

/*------------------------------------------------------------------------------
 * loop()
 *
 *------------------------------------------------------------------------------*/
void loop() {
  uint16_t readData[64];  // Set array large enough to store sensor read burst
  uint32_t sampleCount = 0;
  const uint32_t MAXSAMPLE = 10;

  su.sensorConfigDump();
  su.sensorScaleFactorsPrint();

  SerialConsole.print("Selftest Result (0 is pass) = ");
  SerialConsole.println(su.sensorSelfTest(), HEX);

  SerialConsole.print("Flashtest Result (1 is pass) = ");
  SerialConsole.println(su.sensorFlashTest(), HEX);

  SerialConsole.print("Goto sampling mode");
  su.sensorStart();

  SerialConsole.print("Goto config mode");
  su.sensorStop();

  SerialConsole.println("\nConfiguation Before Software Reset");
  su.sensorConfigDump();
  su.sensorReset();

  SerialConsole.println("Configuration After Software Reset");
  su.sensorConfigDump();

  // Init SU
  SerialConsole.println("Init Device");
  if (!su.sensorInit(CMD_RATEX, CMD_FILTERX)) {
    SerialConsole.println("Error. Can not initialize " EPSON_UNIT_TYPE);
    while (1)
      ;
  }
  SerialConsole.println(EPSON_UNIT_TYPE " Init Done");
  su.sensorConfigDump();
  SerialConsole.println("Goto sampling mode");
  su.sensorStart();
  delay(1000);
  SerialConsole.println("Goto config mode");
  su.sensorStop();
  delay(1000);
  su.registerDump();

  SerialConsole.println("\nHardware Reset");
  su.sensorHWReset();
  su.registerDump();

  // Init SU
  if (!su.sensorInit(CMD_RATEX, CMD_FILTERX)) {
    SerialConsole.println("Error. Can not initialize " EPSON_UNIT_TYPE);
    while (1)
      ;
  }
  SerialConsole.println(EPSON_UNIT_TYPE " Init Done");

  su.sensorConfigDump();
  su.sensorScaleFactorsPrint();
  su.sensorHeaderPrint();

  // Go to SAMPLING mode
  if (!su.sensorStart()) {
    SerialConsole.println("Error. Sensor not entering Sampling mode");
    while (1)
      ;
  }
  while (1) {
    // Burst read one sensor sample set
    if (su.sensorReadBurst(readData, 64)) {
      // Output formatted to console
      su.sensorDataPrint(readData, sampleCount);
    } else {
      SerialConsole.println("#Timeout waiting for DRDY.");
    }
    sampleCount++;

    // If sampleCount reaches limit stop application
    if (sampleCount >= MAXSAMPLE) {
      if (!su.sensorStop()) {
        SerialConsole.println("Error. Sensor not entering Sampling mode");
        while (1)
          ;
      }
      SerialConsole.println("Done.");
      su.registerDump();
      while (1)
        ;  // Stops further execution
    }
  }
}
