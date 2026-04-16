/***********************************************************************
  su_epson_imu_sampling.ino

  This Arduino sketch example application initializes the SU,
  and displays sensor readings.

  This is tested for support on Teensy 3.6 or Arduino Zero.
  However, this software is expected to work on any 3.3V Arduino platform.

  For more information on the Epson Sensing Units visit:

  --> https://www.epsondevice.com/sensing/en/

  This test application assumes that the SU UART signals are connected
  to the Arduino host. The pins are mapped as shown below.

  Circuit Pinmapping:
  Signal Teensy 3.6   ESP32   Arduino Zero/DUE  M-Gseries
  ---------------------------------------------------------------------
  DRDY   pin 6        pin 4       pin 6         pin 13
  CSB    pin 4        pin 5       pin 4         pin 6
  MOSI   pin 11       pin 23      SPI-4         pin 5
  MISO   pin 12       pin 19      SPI-1         pin 2
  SCK    pin 13       pin 18      SPI-3         pin 1
  RST#   pin 7        pin 2       pin 7         pin 16

************************************************************************

  @section LICENSE

  Software License Agreement (BSD License, see license.txt)

  Copyright (c) 2025, 2026 Seiko Epson Corporation.
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

// Epson SU Arduino Library
#include "epson_imu_spi.h"

// Chipselect,DataReady, Reset, and EXT must be assigned as GPIO
// If Reset# pin is not connected, set to -1.
// NOTE: DRDY pin must be assigned, and set drdy_on = 1 & drdy_pol= 1
//  in xxxInitOptions
const int8_t EPSON_CS = 4;
const int8_t EPSON_DRDY = 6;
const int8_t EPSON_RESET = 7;

// Create Epson init structure
struct ImuInitOptions init_options = {
  // MSC_CTRL
  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
  0,  // ext_sel
  0,  // ext_pol

  // drdy_on & drdy_pol must be set to 1 for SPI interface or
  // when DRDY pin is used
  1,  // drdy_on
  1,  // drdy_pol

  // SMPL_CTRL
  EPSON_I_S::CMD_RATE250,  // dout_rate

  // FILTER_CTRL
  EPSON_I_S::CMD_FLTAP32,  // filter_sel

  // UART_CTRL
  // uart_auto must be set to 0 for SPI interface
  0,  // uart_auto

  // BURST_CTRL
  1,  // flag_out
  1,  // temp_out
  1,  // gyro_out
  1,  // accel_out
  0,  // gyro_delta_out
  0,  // accel_delta_out
  0,  // qtn_out, Only valid for devices that support attitude output
  0,  // atti_out, Only valid for devices that support attitude output
  0,  // gpio_out
  1,  // count_out
  0,  // checksum_out

  // Set 0=16bit 1=32bit sensor output.
  // These only have effect if above related "_out = 1"
  1,  // temp_bit
  1,  // gyro_bit
  1,  // accel_bit
  1,  // gyro_delta_bit
  1,  // accel_delta_bit
  1,  // qtn_bit
  1,  // atti_bit

  0,  // invert_xgyro
  0,  // invert_ygyro
  0,  // invert_zgyro
  0,  // invert_xaccel
  0,  // invert_yaccel
  0,  // invert_zaccel

  0,  // a_range_ctrl

  8,  // dlta_range_ctrl
  8,  // dltv_range_ctrl

  // NOTE: The following are only valid when attitude output is enabled
  1,  // atti_mode = 0=Inclination mode 1=Euler mode
  0,  // atti_conv, Attitude Conversion Mode, must be 0 when quaternion
      // output is enabled
  0,  // Attitude Motion Profile 0=modeA 1=modeB 2=modeC
};

// Create struct that will store scaled sensor data
struct ImuScaledData scaled_fields = {};

// Create SPI Epson object
EPSON_IMU_SPI su =
  EPSON_IMU_SPI(SPI, 1000000, EPSON_CS, EPSON_RESET, EPSON_DRDY, Serial);

/*------------------------------------------------------------------------------
 * setup()
 *
 *------------------------------------------------------------------------------*/
void setup() {
#ifdef ARDUINO_ARCH_SAMD
  // Arduino Zero or similar to enable floating point support for printing
  asm(".global _printf_float");
#endif

  // Setup serial console communications
  Serial.begin(921600);
  while (!Serial) {
    ;  // Wait for serial port to connect. Needed for native USB port only
  }
  // Start Epson interface
  if (!su.begin()) {
    Serial.println(
      "Error. Cannot communicate properly with Epson device. Halting...");
    while (1) {
    };
  }
  // Detect and set the sensor model
  if (!su.sensorModelSelect(su.sensorModelDetect())) {
    Serial.println("Error. Unsupported device detected. Halting...");
    while (1) {
    };
  }
  // Initialize device
  su.sensorInitOptions(&init_options);
}

/*------------------------------------------------------------------------------
 * loop()
 *
 *------------------------------------------------------------------------------*/
void loop() {
  // Print column details
  su.sensorHeaderPrint();
  // Place sensor in sampling mode
  su.sensorStart();
  // Burst read sensor packets and print to console
  for (uint32_t i = 0; i < 500; i++) {
    if (su.sensorGetSensorBurst(scaled_fields)) {
      su.sensorDataPrint(&scaled_fields, i);
    }
  }
  // Place sensor in config mode
  su.sensorStop();
  // Print device configuration info
  su.sensorConfigPrint();
  Serial.println("Done");
  while (1) {
    // Wait forever
  };
}
