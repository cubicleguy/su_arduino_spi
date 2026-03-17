/**************************************************************************/
/*!
    @file     epson_spi_common.h

    Header file for Epson SPI class

    @section  HISTORY

    v1.0 - First release restructure

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2025 Seiko Epson Corporation.
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
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>

namespace EPSON_SPI {

constexpr uint16_t ID_RETVAL = 0x5345;
constexpr uint16_t NOT_READY_BIT = 0x0400;
constexpr uint8_t WINDOW0 = 0x00;
constexpr uint8_t WINDOW1 = 0x01;

// Register Addresses
constexpr uint8_t ID = 0x4C;
constexpr uint8_t WIN_CTRL = 0x7E;
constexpr uint8_t GLOB_CMD_LO = 0x0A;

// Microseconds, minimum delay to wait between commands
constexpr unsigned int EPSON_TSTALL_DELAY = 20;

// Microseconds, minimum delay to wait after burst command
constexpr unsigned int EPSON_TSTALL1_DELAY = 45;

// Microseconds, minimum delay to wait between subsequent
// read cycles after initial burst command
constexpr unsigned int EPSON_TSTALL2_DELAY = 4;

// Milliseconds, max delay for power-on startup completion
constexpr unsigned long EPSON_POWER_ON_DELAY = 800;

// Milliseconds, min delay for nRESET assertion
constexpr unsigned long EPSON_NRESET_LOW_DELAY = 100;

// Microseconds, minimum delay between polling for DRDY pin
constexpr unsigned int EPSON_DRDYCHECK_DELAY = 5;

// Number of retries checking dor DRDY pin
constexpr uint32_t EPSON_DRDYCHECK_RETRIES = 500000;

}  // namespace EPSON_SPI

//------------------------
// SPI_EPSON_COM driver class
//------------------------
class SPI_EPSON_COM {
 public:
  SPI_EPSON_COM(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs, int8_t nrst,
                int8_t drdy, Stream& consolePort);

  boolean begin(void);
  uint8_t SPItransfer(uint8_t x);
  void regWrite8(uint8_t winid, uint8_t addr, uint8_t value,
                 boolean verbose = false);
  uint16_t regRead16(uint8_t winid, uint8_t addr, boolean verbose = false);
  boolean toggleReset(void);
  int8_t getDRDY(void) { return _drdy; };
  boolean readNB(uint16_t* arrayOut, uint8_t addr, uint8_t readLength,
                 boolean verbose = false);
  boolean waitDataReady(boolean polarity, uint32_t retryMaxCount);

 private:
  SPIClass& _spiPort;
  uint32_t _spiClkRate = 1000000;
  int8_t _ncs = -1;
  int8_t _nrst = -1;
  int8_t _drdy = -1;
  Stream& _consolePort;
  boolean _initialised = false;
};
