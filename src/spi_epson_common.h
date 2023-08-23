/**************************************************************************/
/*!
    @file     spi_epson_common.h

    Header file for Epson IMU/Accel Driver for Arduino SPI

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Modify sensorReadBurst() to return timeout status on waitDataReady()
    v1.3 - Change EPSON_FILTER_DELAY value

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

    3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

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
*/
/**************************************************************************/
#ifndef SPI_EPSON_COMMON_H_
#define SPI_EPSON_COMMON_H_

#if ARDUINO >= 100
#include <Arduino.h>  //include for Arduino
#else
#include <WProgram.h>
#endif

#define SerialConsole Serial

#include <SPI.h>  //include for SPI

// Epson specific timing delays
#define EPSON_STALL_DELAY \
  20  // Microseconds, minimum delay to wait between non-write commands
#define BURST_STALL1 \
  45  // Microseconds, minumum delay after initial BURST read command
#define BURST_STALL2 \
  4  // Microseconds, minumum delay after between BURST cycles
#define EPSON_READRATE \
  40  // Microseconds, minimum delay to wait between non-write commands
#define EPSON_SWRESET_DELAY \
  800000  // Microseconds, minimum delay after software reset
#define EPSON_POWER_ON_DELAY \
  800000  // Microseconds, max delay for poweron startup completion
#define EPSON_SELF_TEST_DELAY \
  150000  // Microseconds, max delay for self-test completion (80ms is common
          // for most IMUs, 150ms is for G370)
#define EPSON_FLASH_TEST_DELAY \
  5000  // Microseconds, max delay for flash-test completion
#define EPSON_FILTER_DELAY \
  100000  // Microseconds, max delay for filter setup completion (worst case
          // 100ms for A352 UDF)
#define EPSON_NRESET_LOW_DELAY \
  100000  // Microseconds, min delay for nRESET assertion
const uint32_t EPSON_DRDYCHECK = 1000;  // Define max # of times to check

#define EpsonStall() delayMicroseconds(EPSON_STALL_DELAY)
#define EpsonBurstStall1() delayMicroseconds(BURST_STALL1)
#define EpsonBurstStall2() delayMicroseconds(BURST_STALL2)
#define EpsonPowerOnDelay() delayMicroseconds(EPSON_POWER_ON_DELAY)
#define EpsonSelfTestDelay() delayMicroseconds(EPSON_SELF_TEST_DELAY)
#define EpsonFlashTestDelay() delayMicroseconds(EPSON_FLASH_TEST_DELAY)
#define EpsonSwResetDelay() delayMicroseconds(EPSON_SWRESET_DELAY)
#define EpsonFilterDelay() delayMicroseconds(EPSON_FILTER_DELAY)
#define EpsonResetAssertDelay() delayMicroseconds(EPSON_NRESET_LOW_DELAY)

//------------------------
// SPI_EPSON_COM driver class
//------------------------
class SPI_EPSON_COM {
 public:
  SPI_EPSON_COM(int8_t cs, int8_t drdy, int8_t nrst);  // Hardware SPI
  SPI_EPSON_COM(int8_t clk, int8_t miso, int8_t mosi, int8_t cs, int8_t drdy,
                int8_t nrst);  // Software SPI

  // user methods
  boolean begin(void);
  void regWrite8(uint8_t winid, uint8_t addr, uint8_t value,
                 boolean verbose = false);
  uint16_t regRead16(uint8_t winid, uint8_t addr, boolean verbose = false);
  boolean sensorStart(void);
  boolean sensorStop(void);
  uint16_t sensorSelfTest(void);
  boolean sensorHWReset(void);
  boolean sensorReset(void);
  boolean sensorFlashTest(void);
  boolean sensorReadBurst(uint16_t* arr, uint8_t len);

 protected:
  int8_t getDRDY(void) { return _drdy; };
  void getProdID(
      char* prodID);          // return 9byte null terminated char array pointer
  uint16_t getVersion(void);  // return 16-bit version
  void getSerialNumber(
      char* serialNumber);  // return 9byte null terminated char array pointer
  uint8_t _burstCnt_calculated;

 private:
  boolean waitDataReady(boolean polarity, uint32_t retryMaxCount);
  boolean isConfigMode(void);
  void gotoSamplingMode(void);
  void gotoConfigMode(void);
  void readN(uint16_t* arr, uint8_t addr, uint8_t readLength);
  void readNB(uint16_t* arr, uint8_t addr, uint8_t readLength);
  uint8_t SPItransfer(uint8_t x);

  boolean _initialised;                          // device not initialized yet
  int8_t _cs, _clk, _mosi, _miso, _drdy, _nrst;  // pins for software controlled
};

#include "epson_devices.h"

#endif  //< SPI_EPSON_COMMON_H_
