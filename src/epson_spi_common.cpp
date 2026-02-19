/**************************************************************************/
/*!
    @file     epson_spi_common.cpp

    Epson SPI class

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

#include "epson_spi_common.h"

using namespace EPSON_SPI;
/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
SPI_EPSON_COM::SPI_EPSON_COM(SPIClass& spiPort, uint32_t spiClkRate, int8_t ncs,
                             int8_t nrst, int8_t drdy,
                             Stream& consolePort)
    :  // initializer list
      _spiPort(spiPort),
      _spiClkRate(spiClkRate),
      _ncs(ncs),
      _nrst(nrst),
      _drdy(drdy),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Initializes SPI and reads the Product ID register
            for validation (call this function before doing
            anything else)

    @returns   False if DataReady timeout or Product ID is unknown,
               otherwise True
*/
/**************************************************************************/
boolean SPI_EPSON_COM::begin(void) {
  boolean ok = true;

  if (_ncs == -1) {
    _consolePort.println("Error: nCS must be specified.");
    return false;
  }

  _consolePort.print("Platform: ");
#ifdef __SAM3X8E__
  _consolePort.println("Arduino DUE");
  _spiPort.begin(_ncs);
  _spiPort.setClockDivider(_ncs, 84);  // 84/84 = 1 MHz
  _spiPort.setBitOrder(_ncs, MSBFIRST);
  _spiPort.setDataMode(_ncs, SPI_MODE3);
#elif defined(__MK66FX1M0__)  // Teensy 3.6
  _consolePort.println("Teensy 3.6");
  pinMode(_ncs, OUTPUT);
  digitalWrite(_ncs, HIGH);
  _spiPort.begin();
  _spiPort.beginTransaction(SPISettings(_spiClkRate, MSBFIRST, SPI_MODE3));
#else
  _consolePort.println("Normal Arduino");
  pinMode(_ncs, OUTPUT);
  digitalWrite(_ncs, HIGH);
  _spiPort.begin();
  _spiPort.beginTransaction(SPISettings(_spiClkRate, MSBFIRST, SPI_MODE3));
#endif
  _consolePort.print("Open SPI Port for Epson device: ");
  _consolePort.print("nCS on ");
  _consolePort.print(_ncs, DEC);
  _consolePort.print(" @ ");
  _consolePort.print(_spiClkRate, DEC);
  _consolePort.println(" Hz");

  // Configure nRESET Input pin if defined
  if (_nrst != -1) {
    // Set nRESET pin for output HIGH
    _consolePort.print("nRST on ");
    _consolePort.println(_nrst, DEC);
    pinMode(_nrst, OUTPUT);
    digitalWrite(_nrst, HIGH);
    // Issue a Hardware Reset to get device to known state
    _consolePort.println("HW Reset asserted");
    if (!toggleReset()) {
      _consolePort.println("Warning: NOT_READY is HIGH which should be LOW");
    };
  } else {
    _consolePort.println("nRST not used");
  }

  // Configure DataReady Input pin if defined
  if (_drdy != -1) {
    _consolePort.print("DRDY on pin ");
    _consolePort.println(_drdy, DEC);
    pinMode(_drdy, INPUT);

    // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active
    // HIGH) If the device is configured for DRDY active LOW, then this
    // validation check will fail
    _consolePort.println("Check DRDY");
    if (getDRDY() == 1) {
      _consolePort.println(
        "Error: DRDY = HIGH, DataReady pin should be LOW."
        "Check your hardware connection.");
      ok = false;
    }
  } else {
    // if DRDY is not used bypass the DRDY pin check
    _consolePort.println("DRDY not used");
  }

  // Check ID register for IMU/Accel
  _consolePort.print("Checking device is present...");
  uint16_t retVal = regRead16(WINDOW0, ID);
  if (retVal == ID_RETVAL) {
    _consolePort.println("device responded to ID read");
  } else {
    ok = false;
    _consolePort.print("ERROR: Incorrect device response - 0x");
    _consolePort.println(retVal, HEX);
  }
  _initialised = true;
  return ok;
}

/**************************************************************************/
/*!
    @brief  Wrapper for SPI transfers
    @param [in]  x (byte)
                 byte shifted out on MOSI
    @return      byte shifted in on MISO
*/
/**************************************************************************/
uint8_t SPI_EPSON_COM::SPItransfer(uint8_t x) {
#ifdef __SAM3X8E__
  // For Hardware SPI on DUE
  return _spiPort.transfer(_ncs, x);
#else
  // For Hardware SPI on non-DUE
  return _spiPort.transfer(x);
#endif
}

/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address

    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void SPI_EPSON_COM::regWrite8(uint8_t winid, uint8_t addr, uint8_t value,
                              boolean verbose) {
  // Set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the window command & win ID
  SPItransfer(WIN_CTRL | 0x80);
  SPItransfer(winid & 0x01);  // mask off unused bits
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);
  // Send the address (command) to be written
  SPItransfer(addr | 0x80);  // msb is set 1b for register write
  // Write data value
  SPItransfer(value);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);
  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr & 0x7F), HEX);
    _consolePort.print(" W(");
    _consolePort.print(winid, DEC);
    _consolePort.print(")");
    _consolePort.print("] < 0x");
    _consolePort.println(value, HEX);
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address

    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address (must be even, 16-bit aligned)
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
uint16_t SPI_EPSON_COM::regRead16(uint8_t winid, uint8_t addr,
                                  boolean verbose) {
  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // Send the window command & win ID
  SPItransfer(WIN_CTRL | 0x80);
  SPItransfer(winid & 0x01);  // mask off unused bits
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);
  // Send the address
  SPItransfer(addr & 0x7F);  // msb is set 0b for register read
  // Send dummy byte
  SPItransfer(0x00);
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);
  // Initiate 16-bit dummy cycle to return data
  uint16_t readData = SPItransfer(0x00) << 8 | SPItransfer(0x00);
  // Release ChipSelect
  digitalWrite(_ncs, HIGH);
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr & 0x7F), HEX);
    _consolePort.print(" W(");
    _consolePort.print(winid, DEC);
    _consolePort.print(")");
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return readData;
}

/**************************************************************************/
/*!
    @brief  Issues a hardware reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean SPI_EPSON_COM::toggleReset(void) {
  if (_nrst == -1) {
    // bypass because nRESET pin not used
    _consolePort.println("nRESET not enabled, bypassing Reset toggling");
    return true;
  }
  _consolePort.println("Asserting HW Reset");
  // Asserts the nRESET pin LOW
  digitalWrite(_nrst, LOW);
  delay(EPSON_NRESET_LOW_DELAY);

  // Asserts the nRESET pin HIGH
  digitalWrite(_nrst, HIGH);
  // Wait for the sensor re-initialization
  delay(EPSON_POWER_ON_DELAY);
  // Check NOT_READY bit = 0
  if (regRead16(WINDOW1, GLOB_CMD_LO) & NOT_READY_BIT) {
    _consolePort.println("Warning: NOT_READY bit is HIGH");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads a specified number of Burst 16-bit registers
            starting with specified addr, followed by 0x0000 on subsequent
            cycles

    @param [out]  arrayOut (max 64 elements)
                  Array of return 16-bit values

    @param [in]   addr
                  This is typically the burst command (0x80) sent to start
                  the transfer
    @param [in]   readLength (size of arrayOut in bytes, must be <= 128)
                  Specify the length of the burst read transfer in bytes
    @param [in]   verbose
                  boolean to enable debug output of burst access
    @returns      True on success, False if readLength=0, or waitDataReady
                      retries is exceeded
*/
/**************************************************************************/
boolean SPI_EPSON_COM::readNB(uint16_t* arrayOut, uint8_t addr,
                              uint8_t readLength, boolean verbose) {
  if (readLength == 0) {
    _consolePort.println("readLength must be greater than 0");
    return false;
  }
  // If check for DRDY exceeds retries then return false
  if (!waitDataReady(true, EPSON_DRDYCHECK_RETRIES)) return false;

  // set ChipSelect
  digitalWrite(_ncs, LOW);
  // send the starting address to be read
  SPItransfer(addr);  // address (typically BURST_CMD = 0x80)
  SPItransfer(00);    // dummy byte
  // delay after burst command
  delayMicroseconds(EPSON_TSTALL1_DELAY);

  // read the required number of words
  for (uint8_t i = 0; i < (readLength >> 1); i++) {
    arrayOut[i] = SPItransfer(00) << 8 | SPItransfer(00);
    delayMicroseconds(EPSON_TSTALL2_DELAY);
  }
  // release ChipSelect
  digitalWrite(_ncs, HIGH);
  // Delay between commands
  delayMicroseconds(EPSON_TSTALL_DELAY);
  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("Index: ");
    for (uint8_t i = 0; i < (readLength >> 1); i++) {
      _consolePort.print(arrayOut[i], HEX);
      _consolePort.print(", ");
    }
    _consolePort.println();
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Poll waiting for DataReady pin to go to specified state.
            Will continue to retry until the specified retry count is reached.
            There is a delay between polling reads.

    @param [in]  polarity
                 false for LOW, true for HIGH
    @param [in]  retryMaxCount
                 Maximum retries checking DRDY before a timeout

    @returns    True on success, False on timeout

*/
/**************************************************************************/
boolean SPI_EPSON_COM::waitDataReady(boolean polarity, uint32_t retryMaxCount) {
  // Loop continuously until DataReady or timeout
  for (uint32_t retryCount = 0; retryCount < retryMaxCount; retryCount++) {
    if (digitalRead(_drdy) == polarity) {
      return true;
    }
    delayMicroseconds(EPSON_DRDYCHECK_DELAY);
  }
  _consolePort.println("Warning: Retry exceeded waiting for DRDY");
  return false;
}
